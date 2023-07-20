#ifndef SPO2_H__
#define SPO2_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
	float irACValueSqSum;
	float redACValueSqSum;
	uint8_t beatsDetectedNum;
	uint32_t samplesRecorded;
	uint8_t spO2;
	float R;
	uint8_t spo2_flag;
	float spO2Acc;
} spo2_t;

spo2_t *spo2_init(spo2_t *sp);
void spo2_update(spo2_t *sp, float irACValue, float redACValue, bool beatDetected);

#include <stdbool.h>

#define min(x, y) ((x) < (y) ? (x) : (y))

typedef enum BeatDetectorState {
    BEATDETECTOR_STATE_INIT,
    BEATDETECTOR_STATE_WAITING,
    BEATDETECTOR_STATE_FOLLOWING_SLOPE,
    BEATDETECTOR_STATE_MAYBE_DETECTED,
    BEATDETECTOR_STATE_MASKING
} BeatDetectorState;


typedef struct
{
	BeatDetectorState state;
    float threshold;
    float beatPeriod;
    float lastMaxValue;
    uint32_t tsLastBeat;
} beat_detector;

bool checkForBeat(beat_detector *bd, float sample);
void decreaseThreshold(beat_detector *bd);


beat_detector *beat_detector_init(beat_detector *bd);

bool addSample(beat_detector *bd, float sample);

float getRate(beat_detector *bd);

float getCurrentThreshold(beat_detector *bd);
bool checkForBeat(beat_detector *bd, float sample);

void decreaseThreshold(beat_detector *bd);

typedef struct
{
	float v[2];
} FilterBuLp1;

FilterBuLp1 *lpf_init(FilterBuLp1 *filter);

float lpf_step(FilterBuLp1 *filter, float x);

typedef struct
{
	float alpha;
	float dcw;
} dc_remover;

dc_remover *dc_remover_init(dc_remover *dr, float alpha);
float dc_remover_step(dc_remover *dr, float x);

typedef struct
{
	FilterBuLp1 lpf;
	dc_remover irDCRemover, redDCRemover;
	beat_detector bd;
	spo2_t spo2_calc;
	
	float ir_amp, red_amp;

	int beat_flag;
	int spo2;
	float bpm;
} pulse_oximeter_t;

pulse_oximeter_t *pulse_oximeter_init(pulse_oximeter_t *p);
pulse_oximeter_t *pulse_oximeter_update(pulse_oximeter_t *p, int adc_ir, int adc_red, bool *beatDetectedOut, int *filterSpO2IRvalueOut);

#ifdef __cplusplus
}
#endif

#endif /* SPO2_H__ */
