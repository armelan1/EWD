#ifndef PULSEOX_ALG_H
#define PULSEOX_ALG_H

//#ifdef __cplusplus
//extern "C" {
//#endif

#include <stdint.h>
#include <Arduino.h>
#include <Wire.h>


#define SENSOR_DECIMA_COUNT 5//was 2

#define FFT_WINDOW  32
#define INPUT_SCALE_FACTOR  1000.0f
//#define RMS_BUF_LEN 200

/* R -> SpO2 calibration */
#define CALIBRATION_SP_RED_A      -187.9761 //-164.38 //-142.0349 // -164.38 //-93.375 //-230.8
#define CALIBRATION_SP_RED_B      250.4783 //229.9488 //212.9488 //219.9488 // 240.49 // 177.76 //301.5

#define CALIBRATION_SP_RED_A      -464.7049
#define CALIBRATION_SP_RED_B      671.018
#define CALIBRATION_SP_RED_C      -145.2166 //-142.2166


#define CALIBRATION_SP_GREEN_A    -57.5
#define CALIBRATION_SP_GREEN_B    161.25

typedef struct
{
    float *samples, *buf_samples;
    int len;
    int update_len;
} median_filter_t;

median_filter_t *median_filter_init(median_filter_t *filter, int len);

float median_filter_update(median_filter_t *filter, float sample);

float median_filter_average(median_filter_t *filter, float sample);

int median_filter_is_valid(median_filter_t *filter, float max_dev);

// -----------------------------------

typedef struct
{
	float ir_amp, red_amp, green_amp;
	int amp_valid;
	float ir_ac, red_ac, green_ac;
  bool beat_detected;
  int filterSpO2IRvalueOut;
} psx_dsp_t;

void pulseox_init(void);

void pulseox_update(float red_value, float green_value, float ir_value, float stored_Count,
                    float *spo2, float *R, int *spo2_valid,
                    float *spo2_g, float *R_g, int *spo2_valid_g,
                    float *bpm, int *bpm_valid,
                    float *red_LED_DC, float *IR_LED_DC,
                    psx_dsp_t *psx_dsp);

/* utils */
typedef struct
{
    float *data;
    float *coeffs;
    int len;
} fir_filter_t;

fir_filter_t *fir_filter_init(fir_filter_t *fir, float *coeffs, int len);

float fir_update(fir_filter_t *fir, float data);

typedef struct
{
    float p, i, d;

    float error_prev;
    float integral;
} pid_controller_t;

//#ifdef __cplusplus
//}
//#endif

#endif // PULSEOX_ALG_H
