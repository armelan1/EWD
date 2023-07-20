//
//  spo2_search_alg.h
//  spo2_test
//
//  Created by Yuchong Li on 3/21/22.
//

#ifndef spo2_search_alg_h
#define spo2_search_alg_h

#define SPO2_SEARCH_WINDOW      300
#define SPO2_SEARCH_ATTEMPTS    100

#ifdef __cplusplus
extern "C" {
#endif


typedef struct
{
    float alpha;
    float dcw;
} _builtin_dc_remover;

typedef struct
{
    float ir_buf[SPO2_SEARCH_WINDOW];
    float red_buf[SPO2_SEARCH_WINDOW];
} spo2_buffer_t;

typedef struct
{
    spo2_buffer_t buf;
    _builtin_dc_remover dr_ir, dr_red;
    float ir_ac, red_ac;

    int current_count;
} spo2_search_alg_t;

typedef struct
{
    float R;
} spo2_search_result_t;

void spo2_search_init(spo2_search_alg_t *alg);
float spo2_search_R(float *ir, float *red, int start, int length, float *corr);
int spo2_search_update(spo2_search_alg_t *alg, float ir, float red, spo2_search_result_t *result);

#ifdef __cplusplus
}
#endif


#endif /* spo2_search_alg_h */
