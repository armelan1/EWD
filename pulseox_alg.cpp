#include "pulseox_alg.h"
#include "spo2.h"

#include "kiss_fft.h"
#include <float.h>
#include <math.h>

#include "spo2_search_alg.h"

float SENSOR_SAMPLE_RATE = 103.00; // 119.0;

#include "MAXM86161.h"
float Instant_SpO2;

/* new random sampled spo2 algorithm */
spo2_search_alg_t alg;

/* oximeter instance */
pulse_oximeter_t psx;
pulse_oximeter_t psx_green;

/* measurement filters */
median_filter_t ratio_avg_filter;
median_filter_t ratio_median_filter;

median_filter_t spo2_filter;
median_filter_t spo2_avg_filter;
median_filter_t running_SpO2_median_Filter;

median_filter_t spo2_filter_g;
median_filter_t spo2_avg_filter_g;

median_filter_t bpm_filter;

median_filter_t *median_filter_init(median_filter_t *filter, int len)
{
    /* filter length shall be an even number */
    if(len % 2 != 1)
    {
        return NULL;
    }

    filter->len = len;
    filter->buf_samples = (float *)malloc(sizeof(float) * len);
    filter->samples = (float *)malloc(sizeof(float) * len);

    /* memory allocation failed */
    if(!filter->samples || !filter->buf_samples)
    {
        return NULL;
    }

    /* clear sample buffer */
    for(int i = 0; i < len; i++)
    {
        filter->samples[i] = 0;
        filter->buf_samples[i] = 0;
    }

    return filter;
}

static inline
void __int64_swap(float *array, int a, int b)
{
    float temp = array[a];
    array[a] = array[b];
    array[b] = temp;
}

static inline
int __med_filter_partition(float *array, int left, int right)
{
    if (array == NULL)
    {
        return -1;
    }

    int pos = right;
    right--;

    while (left <= right)
    {
        while (left < pos && array[left] <= array[pos])
        {
            left++;
        }

        while (right >= 0 && array[right] > array[pos])
        {
            right--;
        }

        if (left >= right)
        {
            break;
        }

        __int64_swap(array, left, right);
    }

    __int64_swap(array, left, pos);

    return left;
}

static inline
float __med_filter_find_med(float *array, int size)
{
    if (array == NULL || size <= 0)
        return -1;

    int left = 0;
    int right = size - 1;
    int midPos = right / 2;
    int index = -1;

    while (index != midPos)
    {
        index = __med_filter_partition(array, left, right);

        if (index < midPos)
        {
            left = index + 1;
        }
        else if (index > midPos)
        {
            right = index - 1;
        }
        else
        {
            break;
        }
    }

    return array[index];
}

float median_filter_update(median_filter_t *filter, float sample)
{
    /* update samples */
    for(int i = 0; i < filter->len - 1; i++)
    {
        filter->samples[i] = filter->samples[i + 1];
    }

    filter->samples[filter->len - 1] = sample;

    for(int i = 0; i < filter->len; i++)
    {
        filter->buf_samples[i] = filter->samples[i];
    }

    return __med_filter_find_med(filter->buf_samples, filter->len);
}

float median_filter_average(median_filter_t *filter, float sample)
{
    filter->update_len++;

    /* update samples */
    for(int i = 0; i < filter->len - 1; i++)
    {
        filter->samples[i] = filter->samples[i + 1];
    }

    filter->samples[filter->len - 1] = sample;

    float avg = 0.0f;
    for(int i = 0; i < filter->len; i++)
    {
        filter->buf_samples[i] = filter->samples[i];
        avg += filter->buf_samples[i];
    }

    float mean = avg / filter->len;

    if(filter->update_len >= filter->len)
    {
        return mean;
    }
    else
    {
        return 0.0f;
    }
}

int median_filter_is_valid(median_filter_t *filter, float max_dev)
{
    float max = -FLT_MAX;
    float min = FLT_MAX;
    for(int i = 0; i < filter->len; i++)
    {
        if(filter->buf_samples[i] > max) max = filter->buf_samples[i];
        if(filter->buf_samples[i] < min) min = filter->buf_samples[i];
    }
    
    return fabsf(max - min) < max_dev;
}

// ------------

float hann_window[FFT_WINDOW]; // hanning window (Austin comment)
kiss_fft_cfg fft_cfg;

static void fft32(float *in_buf, float *mag_buf)
{
    // run fft
    kiss_fft_cpx in[FFT_WINDOW];
    for(int j = 0; j < FFT_WINDOW; j++)
    {
        in[j].i = 0;
        in[j].r = in_buf[j] * hann_window[j];
    }

    kiss_fft_cpx out[FFT_WINDOW];
    kiss_fft(fft_cfg, in, out);

    for(int j = 0; j < FFT_WINDOW / 2; j++)
    {
        float mag = 2 * sqrtf(out[j].r * out[j].r + out[j].i * out[j].i) / FFT_WINDOW * 2;
        mag = 20 * log10(mag);
        mag_buf[j] = mag;
    }
}

fir_filter_t *fir_filter_init(fir_filter_t *fir, float *coeffs, int len)
{
    fir->coeffs = (float *)malloc(sizeof(float) * len);
    fir->data = (float *)malloc(sizeof(float) * len);
    fir->len = len;

    for(int i = 0; i < len; i++)
    {
        fir->data[i] = 0.0f;
        fir->coeffs[i] = coeffs[i];
    }

    return fir;
}

float fir_update(fir_filter_t *fir, float data)
{
    float response = 0.0;

    for(int i = 0; i < fir->len - 1; i++)
    {
        fir->data[i] = fir->data[i + 1];
    }

    fir->data[fir->len - 1] = data;

    for(int i = 0; i < fir->len; i++)
    {
        response += fir->data[i] * fir->coeffs[i]; // calculate impulse response (Austin comment)
    }

    return response;
}

pid_controller_t *pid_controller_init(pid_controller_t *pid, float p, float i, float d)
{
    pid->integral = 0.0f;

    pid->p = p;
    pid->i = i;
    pid->d = d;
    pid->error_prev = 0.0f;

    return pid;
}

pid_controller_t *pid_controller_reset(pid_controller_t *pid)
{
    pid->integral = 0.0f;
    pid->error_prev = 0.0f;

    return pid;
}

float pi_update(pid_controller_t *pid,
                float error,
                float dt, float min, float max,
                float int_min, float int_max,
                float feed_forward)
{
    /* calculate P term using the given P value and its compensation term */
    float p = pid->p;
    p = p < 0.0 ? 0.0 : p;

    float i = pid->i;
    i = i < 0.0 ? 0.0 : i;

    float p_out = error *p;

    pid->integral += error * dt * i;

    pid->integral = pid->integral > int_max ? int_max : pid->integral;
    pid->integral = pid->integral < int_min ? int_min : pid->integral;

    float result = p_out + pid->integral + feed_forward;

    result = result > max ? max : result;
    result = result < min ? min : result;

    return result;
}

static float lpf_fir_coeff[] = {
    0.008258690479176576,
    0.008929228606468293,
    0.002293577315122251,
   -0.011929202768143523,
   -0.028037390803672561,
   -0.035058618927876414,
   -0.021214555602256022,
    0.019590836430012738,
    0.082694489647712768,
    0.152271245137899996,
    0.206643105726673215,
    0.227203369140625000,
    0.206643105726673215,
    0.152271245137899996,
    0.082694489647712768,
    0.019590836430012738,
   -0.021214555602256022,
   -0.035058618927876414,
   -0.028037390803672561,
   -0.011929202768143523,
    0.002293577315122251,
    0.008929228606468293,
    0.008258690479176576
};

float fft_buff[FFT_WINDOW], fft_mag[FFT_WINDOW];
//float ir_buff[RMS_BUF_LEN], red_buff[RMS_BUF_LEN];

fir_filter_t ir_filter, red_filter, green_filter;
pid_controller_t ir_pid, red_pid, green_pid;
pid_controller_t ir_amp_pid, red_amp_pid;

float dc_offset_ir = 0, dc_offset_red = 0, dc_offset_green = 0;

static float fft_interp(float *mag, int len)
{
    float max_mag = -10000;
    int max_bin = 0;

    for(int i = 1; i < len; i++)
    {
      if(mag[i] > max_mag)
        {
            max_mag = mag[i];
            max_bin = i;
        }
    }
    
    // quadratic interpolation
    float y1 = fabsf(mag[max_bin - 1]);
    float y2 = fabsf(mag[max_bin]);
    float y3 = fabsf(mag[max_bin + 1]);

    float d = (y3 - y1) / (2.0f * (2.0f * y2 - y1 - y3));
    float k = max_bin + d;
    
    return k;
}

static float rms(float *x, int len)
{
    float sum = 0.0f;
    for(int i = 0; i < len; i++)
    {
        sum += x[i] * x[i];
    }

    sum /= len;
    return sqrtf(sum);
}

/*
 *  public functions for external use
 * ----------------------------------
 */

void pulseox_init(void)
{
    spo2_search_init(&alg);

  
    /* initialize oximeter */
    pulse_oximeter_init(&psx);
    pulse_oximeter_init(&psx_green);

    /* initialize measurement filters */
    median_filter_init(&spo2_filter, 3);
    median_filter_init(&spo2_avg_filter, 3);
    median_filter_init(&running_SpO2_median_Filter, 31);
    
    median_filter_init(&ratio_median_filter, 7);
    median_filter_init(&ratio_avg_filter, 11);
    
    median_filter_init(&bpm_filter, 7);

    /* initialize dsp stuff */
    fir_filter_init(&red_filter, lpf_fir_coeff, sizeof(lpf_fir_coeff) / sizeof(float));
    fir_filter_init(&ir_filter, lpf_fir_coeff, sizeof(lpf_fir_coeff) / sizeof(float));
    fir_filter_init(&green_filter, lpf_fir_coeff, sizeof(lpf_fir_coeff) / sizeof(float));


    float pid_ki = 5;
    float pid_kp = 0.2;
    pid_controller_init(&ir_pid, pid_kp, pid_ki, 0);
    pid_controller_init(&red_pid, pid_kp, pid_ki, 0);
    pid_controller_init(&green_pid, pid_kp, pid_ki, 0);

    fft_cfg = kiss_fft_alloc(FFT_WINDOW, 0, NULL, NULL);

    /* generate hann window */
    for(int i = 0; i < FFT_WINDOW; i++)
    {
        float multiplier = 0.5 * (1 - cos(6.2831852 * i / (FFT_WINDOW - 1)));
        hann_window[i] = multiplier;
    }

    // amp control pid's
    pid_controller_init(&ir_amp_pid, 1, 10, 0);
    pid_controller_init(&red_amp_pid, 1, 10, 0);
}



float runnning_SpO2 = 0.0;
float Ratio = 0.0;
float RatioMedian = 0.0;
float RatioAve = 0.0;
float previous_sp = 0;
int acceptable_sp_diff = 10;
int skipped_SpO2_counter =0;
int max_numb_skipped_SpO2_reading = 5;
float previous_bpm = 0;
int acceptable_bpm_diff = 20;
float sp = 0.0;

void pulseox_update(float red_value, float green_value, float ir_value, float stored_Count,
                    float *spo2, float *R, int *spo2_valid,
                    float *spo2_g, float *R_g, int *spo2_valid_g,
                    float *bpm, int *bpm_valid,
                    float *red_LED_DC,  float *IR_LED_DC,
                    psx_dsp_t *psx_dsp)
{
    /* reset valid flags */
    *bpm_valid = 0;
    *spo2_valid = 0;
    *spo2_valid_g = 0;


//     if(stored_Count > 100){
//      SENSOR_SAMPLE_RATE = stored_Count;
//      // Note: "stored_Count" comes from "MAXM86161.cpp"
//    }       

    // ir-red
    bool beat_detected;
    int filterSpO2IRvalueOut;
    pulse_oximeter_update(&psx, ir_value, red_value, &beat_detected, &filterSpO2IRvalueOut);

    spo2_search_result_t result;
    
    //if(psx.spo2_calc.spo2_flag > 0)
    if(spo2_search_update(&alg, ir_value, red_value, &result) != 0)
    {
        psx.spo2_calc.spo2_flag = 0;

//        Ratio=median_filter_average(&ratio_avg_filter, psx.spo2_calc.R);
  
        //RatioMedian = median_filter_update(&ratio_median_filter, psx.spo2_calc.R);
        RatioMedian = median_filter_update(&ratio_median_filter, result.R);
//        if(RatioMedian > 0){
//          Ratio=median_filter_average(&ratio_avg_filter, RatioMedian);
//        }
        Ratio = RatioMedian;
        *R = Ratio;
        
        if(Ratio > 0){
//          sp = Ratio * CALIBRATION_SP_RED_A +  CALIBRATION_SP_RED_B;
          sp =  Ratio * Ratio * CALIBRATION_SP_RED_A + CALIBRATION_SP_RED_B * Ratio + CALIBRATION_SP_RED_C;
//          Serial.print("SpO2(pre-check): ");
//          Serial.println(sp);
        }

        if(sp > 100){
          sp = 100.2f; // Marking as 100.2 to clearly mark SpO2 greater than 100 
        }

        float spo2_inst = result.R * result.R * CALIBRATION_SP_RED_A + CALIBRATION_SP_RED_B * result.R + CALIBRATION_SP_RED_C;
        Instant_SpO2 = spo2_inst;
        Serial.printf("random alg R: %.3f, spo2 inst: %.1f\r\n", result.R, spo2_inst);

//        Serial.print("Ratio('raw'): ");
//        Serial.println(psx.spo2_calc.R,4);
//        Serial.print("Ratio(median): ");
//        Serial.println(Ratio,4);
        
        if(sp > 60){
          runnning_SpO2 = median_filter_update(&running_SpO2_median_Filter, sp);
          if(previous_sp != 0){
            if(abs(sp-previous_sp)<=acceptable_sp_diff){
              sp = median_filter_average(&spo2_avg_filter, sp);
              psx.spo2 = sp;
              *spo2 = sp;
              *spo2_valid = 1;
  
              previous_sp = sp;
              skipped_SpO2_counter = 0;
  //            Serial.print("SpO2: ");
  //            Serial.println(sp);
  //            Serial.println("SpO2 added to filter w/ accept diff");
            }
            else{
              skipped_SpO2_counter = skipped_SpO2_counter+1;
            }
            if(skipped_SpO2_counter > max_numb_skipped_SpO2_reading){
              skipped_SpO2_counter = 0;
              Serial.print("SpO2 previous when reset: ");
              Serial.println(runnning_SpO2);
              if(runnning_SpO2>60){
                previous_sp = runnning_SpO2;
                psx.spo2 = runnning_SpO2;
              }
              *spo2_valid = 1; // Only becasue reset, this will reset DAQ SpO2 timer so user can see
              // **Note: have to send SpO2 otherwise a Zero gets throw into the mix b/c "MAXM86161
            }
          }// end of IF Previuos_sp not = 0
          if(previous_sp == 0){
            sp = median_filter_average(&spo2_avg_filter, sp);
            psx.spo2 = sp;
  
            *spo2 = sp;
            *spo2_valid = 1;
              previous_sp = sp;
  //          Serial.print("SpO2: ");
  //          Serial.println(sp);
  //          Serial.println("SpO2 added to filter");
          } // end of IF Previuos_sp == 0
        }// end of IF SpO2 > 60
    }// end of IF SpO2 flag > 0
    psx_dsp->filterSpO2IRvalueOut=filterSpO2IRvalueOut;

    psx_dsp->beat_detected = beat_detected;


    // -----------------------
    // bpm fft
    static int amp_count;
    static float ir_acc, red_acc, green_acc;

    ir_value /= INPUT_SCALE_FACTOR;
    red_value /= INPUT_SCALE_FACTOR;

    ir_acc += ir_value / 16777.215f;
    red_acc += red_value / 16777.215f;

    amp_count++;
    if(amp_count == 10)
    {
        ir_acc /= amp_count;
        red_acc /= amp_count;
        
        psx_dsp->ir_amp = ir_acc;
        psx_dsp->red_amp = red_acc;
        
        amp_count = 0;
        ir_acc = 0;
        red_acc = 0;
        green_acc = 0;
        psx_dsp->amp_valid = 1;
    }

    /* remove bias */
    ir_value -= dc_offset_ir;
    red_value -= dc_offset_red;
//    green_value -= dc_offset_green;


    dc_offset_ir = pi_update(&ir_pid, ir_value, 1/100.0f, -16777215, 16777215, -16777215, 16777215, 0);
    dc_offset_red = pi_update(&red_pid, red_value, 1/100.0f, -16777215, 16777215, -16777215, 16777215, 0);
//    dc_offset_green = pi_update(&green_pid, green_value, 1/100.0f, -16777215, 16777215, -16777215, 16777215, 0);

    ir_value = fir_update(&ir_filter, ir_value);
    red_value = fir_update(&red_filter, red_value);
//    green_value = fir_update(&green_filter, green_value);

    psx_dsp->ir_ac = ir_value;
    psx_dsp->red_ac = red_value;
//    psx_dsp->green_ac = green_value;

    /* input is 25Hz, decimate to 12.5Hz */
    static int decima_count;
    static int fft_input_count;
    decima_count++;
    if(decima_count == SENSOR_DECIMA_COUNT)
    {
        decima_count = 0;
        fft_buff[fft_input_count++] = ir_value ;

        if(fft_input_count == FFT_WINDOW)
        {
            fft_input_count = 0;

            fft32(fft_buff, fft_mag);
            
            float max_bin = fft_interp(fft_mag, FFT_WINDOW/2 );
            /* sample rate after decimation */
            *bpm = max_bin * ((SENSOR_SAMPLE_RATE / SENSOR_DECIMA_COUNT) / FFT_WINDOW * 60.0f);
//            Serial.print("bmp: ");
//            Serial.println(*bpm);
//            Serial.println("=========");
            
            float bpm_avg;
            if(*bpm > 45 && *bpm < 180 && previous_bpm == 0){
//              *bpm = median_filter_update(&bpm_filter, *bpm);
              bpm_avg = median_filter_average(&bpm_filter, *bpm);
              previous_bpm = bpm_avg;
//              previous_bpm = *bpm;
              *bpm = bpm_avg;
//              Serial.println("bpm added to filter");
              *bpm_valid = 1;
            }
            
            if(*bpm > 45 && *bpm < 180 && previous_bpm != 0){
              if(abs(*bpm-previous_bpm)<=acceptable_bpm_diff){
//                *bpm = median_filter_update(&bpm_filter, *bpm);
                bpm_avg = median_filter_average(&bpm_filter, *bpm);
                previous_bpm = bpm_avg;
//                previous_bpm = *bpm;
                *bpm = bpm_avg;
//                Serial.println("bpm added to filter w/ accept diff");
                *bpm_valid = 1;
              }
            }
//            Serial.println("=========");
        }
    }

}
