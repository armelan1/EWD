#include "spo2.h"
#include <math.h>
#include <stdbool.h>

extern uint32_t millis(void);

#define CALCULATE_EVERY_N_BEATS         2

spo2_t *spo2_init(spo2_t *sp)
{
	sp->irACValueSqSum = 0;
	sp->redACValueSqSum = 0;
	sp->beatsDetectedNum = 0;
	sp->samplesRecorded = 0;
	sp->spO2 = 0;
  sp->spo2_flag = 0;
	
	return sp;
}
void spo2_update(spo2_t *sp, float irACValue, float redACValue, bool beatDetected)
{
    sp->irACValueSqSum += irACValue * irACValue;
    sp->redACValueSqSum += redACValue * redACValue;
    ++sp->samplesRecorded;

    if (beatDetected) 
	{
        ++sp->beatsDetectedNum;
        if (sp->beatsDetectedNum == CALCULATE_EVERY_N_BEATS) 
		{
            sp->R = log(sp->redACValueSqSum/sp->samplesRecorded) / log(sp->irACValueSqSum/sp->samplesRecorded);
            float acSqRatio = 100.0 * sp->R;
            uint8_t index = 0;

            if (acSqRatio > 66) 
			{
                index = (uint8_t)acSqRatio - 66;
            } 
			else if (acSqRatio > 50) 
			{
                index = (uint8_t)acSqRatio - 50;
            }
            spo2_init(sp);

            sp->spo2_flag = 1;
        }
    }
}

#define BEATDETECTOR_INIT_HOLDOFF                2000    // in ms, how long to wait before counting
#define BEATDETECTOR_MASKING_HOLDOFF             200     // in ms, non-retriggerable window after beat detection
#define BEATDETECTOR_BPFILTER_ALPHA              0.6     // EMA factor for the beat period value
#define BEATDETECTOR_MIN_THRESHOLD               20      // minimum threshold (filtered) value
#define BEATDETECTOR_MAX_THRESHOLD               800     // maximum threshold (filtered) value
#define BEATDETECTOR_STEP_RESILIENCY             30      // maximum negative jump that triggers the beat edge
#define BEATDETECTOR_THRESHOLD_FALLOFF_TARGET    0.3     // thr chasing factor of the max value when beat
#define BEATDETECTOR_THRESHOLD_DECAY_FACTOR      0.99    // thr chasing factor when no beat
#define BEATDETECTOR_INVALID_READOUT_DELAY       2000    // in ms, no-beat time to cause a reset
#define BEATDETECTOR_SAMPLES_PERIOD              43      // in ms, 1/Fs

#define min(x, y) ((x) < (y) ? (x) : (y))

bool checkForBeat(beat_detector *bd, float sample);
void decreaseThreshold(beat_detector *bd);


beat_detector *beat_detector_init(beat_detector *bd)
{
	bd->beatPeriod = 0;
	bd->lastMaxValue = 0;
	bd->tsLastBeat = 0;
	bd->state = BEATDETECTOR_STATE_INIT;
	bd->threshold = BEATDETECTOR_MIN_THRESHOLD;
	
	return bd;
}

bool addSample(beat_detector *bd, float sample)
{
    return checkForBeat(bd, sample);
}

float getRate(beat_detector *bd)
{
    if (bd->beatPeriod != 0) 
	{
        return 1 / bd->beatPeriod * 1000 * 60;
    } 
	else 
	{
        return 0;
    }
}

float getCurrentThreshold(beat_detector *bd)
{
    return bd->threshold;
}

bool checkForBeat(beat_detector *bd, float sample)
{
    bool beatDetected = false;

    switch (bd->state) 
	{
        case BEATDETECTOR_STATE_INIT:
            if (millis() > BEATDETECTOR_INIT_HOLDOFF) 
			{
                bd->state = BEATDETECTOR_STATE_WAITING;
            }
            break;

        case BEATDETECTOR_STATE_WAITING:
            if (sample > bd->threshold) 
			{
                bd->threshold = min(sample, BEATDETECTOR_MAX_THRESHOLD);
                bd->state = BEATDETECTOR_STATE_FOLLOWING_SLOPE;
            }

            // Tracking lost, resetting
            if (millis() - bd->tsLastBeat > BEATDETECTOR_INVALID_READOUT_DELAY) 
			{
                bd->beatPeriod = 0;
                bd->lastMaxValue = 0;
            }

            decreaseThreshold(bd);
            break;

        case BEATDETECTOR_STATE_FOLLOWING_SLOPE:
            if (sample < bd->threshold) 
			{
                bd->state = BEATDETECTOR_STATE_MAYBE_DETECTED;
            } 
			else 
			{
                bd->threshold = min(sample, BEATDETECTOR_MAX_THRESHOLD);
            }
            break;

        case BEATDETECTOR_STATE_MAYBE_DETECTED:
            if (sample + BEATDETECTOR_STEP_RESILIENCY < bd->threshold) 
			{
                // Found a beat
                beatDetected = true;
                bd->lastMaxValue = sample;
                bd->state = BEATDETECTOR_STATE_MASKING;
                float delta = millis() - bd->tsLastBeat;
                if (delta) 
				{
                    bd->beatPeriod = BEATDETECTOR_BPFILTER_ALPHA * delta +
                            (1 - BEATDETECTOR_BPFILTER_ALPHA) * bd->beatPeriod;
                }

                bd->tsLastBeat = millis();
            } 
			else 
			{
                bd->state = BEATDETECTOR_STATE_FOLLOWING_SLOPE;
            }
            break;

        case BEATDETECTOR_STATE_MASKING:
            if (millis() - bd->tsLastBeat > BEATDETECTOR_MASKING_HOLDOFF) 
			{
                bd->state = BEATDETECTOR_STATE_WAITING;
            }
            decreaseThreshold(bd);
            break;
    }

    return beatDetected;
}

void decreaseThreshold(beat_detector *bd)
{
    // When a valid beat rate readout is present, target the
    if (bd->lastMaxValue > 0 && bd->beatPeriod > 0) 
	{
        bd->threshold -= bd->lastMaxValue * (1 - BEATDETECTOR_THRESHOLD_FALLOFF_TARGET) /
                (bd->beatPeriod / BEATDETECTOR_SAMPLES_PERIOD);
    } 
	else 
	{
        // Asymptotic decay
        bd->threshold *= BEATDETECTOR_THRESHOLD_DECAY_FACTOR;
    }

    if (bd->threshold < BEATDETECTOR_MIN_THRESHOLD) 
	{
        bd->threshold = BEATDETECTOR_MIN_THRESHOLD;
    }
}

FilterBuLp1 *lpf_init(FilterBuLp1 *filter)
{
	filter->v[0] = 0.0f;
	return filter;
}

float lpf_step(FilterBuLp1 *filter, float x)
{
	filter->v[0] = filter->v[1];
	filter->v[1] = (2.452372752527856026e-1 * x)
				 + (0.50952544949442879485 * filter->v[0]);
	return
		 (filter->v[0] + filter->v[1]);
}

dc_remover *dc_remover_init(dc_remover *dr, float alpha)
{
	dr->alpha = alpha;
	dr->dcw = 0.0f;
	return dr;
}

float dc_remover_step(dc_remover *dr, float x)
{
	float olddcw = dr->dcw;
	dr->dcw = (float)x + dr->alpha * dr->dcw;

	return dr->dcw - olddcw;
}

pulse_oximeter_t *pulse_oximeter_init(pulse_oximeter_t *p)
{
	lpf_init(&p->lpf);
	
	dc_remover_init(&p->irDCRemover, 0.95f);
	dc_remover_init(&p->redDCRemover, 0.95f);
	
	beat_detector_init(&p->bd);
	
	spo2_init(&p->spo2_calc);
    p->beat_flag = 0;
	
	return p;
}

pulse_oximeter_t *pulse_oximeter_update(pulse_oximeter_t *p, int adc_ir, int adc_red, bool *beatDetectedOut, int *filterSpO2IRvalueOut)
{
	float irACValue = dc_remover_step(&p->irDCRemover, adc_ir);
	float redACValue = dc_remover_step(&p->redDCRemover, adc_red);

	float filteredPulseValue = lpf_step(&p->lpf, -irACValue);
	bool beatDetected = addSample(&p->bd, filteredPulseValue * 4000);
  *filterSpO2IRvalueOut = filteredPulseValue;

	if(beatDetectedOut != NULL)
	{
		*beatDetectedOut = beatDetected;
	}

    if(beatDetected)
    {
        p->beat_flag = 2;
    }

	if(getRate(&p->bd) > 0)
	{
		spo2_update(&p->spo2_calc, irACValue, redACValue, beatDetected);
		//p->spo2 = p->spo2_calc.spO2;
		//p->bpm = getRate(&p->bd);
	}
	else
	{
		spo2_init(&p->spo2_calc);
		//p->spo2 = -1;
		//p->bpm = -1;
	}
	
	return p;
}
