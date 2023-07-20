//
//  spo2_search_alg.c
//  spo2_test
//
//  Created by Yuchong Li on 3/21/22.
//

#include "spo2_search_alg.h"
#include <math.h>
#include <stdlib.h>
#include <assert.h>

static _builtin_dc_remover *_builtin_dc_remover_init(_builtin_dc_remover *dr, float alpha)
{
    dr->alpha = alpha;
    dr->dcw = 0.0f;
    return dr;
}

static float _builtin_dc_remover_step(_builtin_dc_remover *dr, float x)
{
    float olddcw = dr->dcw;
    dr->dcw = (float)x + dr->alpha * dr->dcw;

    return dr->dcw - olddcw;
}

float calculate_R(float *ir, float *red, int start, int length, float *corr)
{
    float sum_red = 0.0f;
    float sum_ir = 0.0f;
    
    float c = 0.0f;
    
    for(int i = start; i < start + length; i++)
    {
        sum_ir += ir[i] * ir[i];
        sum_red += red[i] * red[i];
        c += ir[i] * red[i];
    }
    
    sum_red /= length; 
    sum_ir /= length;
    
    c /= length;
    if(corr != NULL)
    {
        *corr = c;
    }
    
    return log(sum_red) / log(sum_ir);
}

void spo2_search_init(spo2_search_alg_t *alg)
{
    _builtin_dc_remover_init(&alg->dr_ir, 0.95f);
    _builtin_dc_remover_init(&alg->dr_red, 0.95f);
    alg->current_count = 0;
}

float spo2_search_R(float *ir, float *red, int start, int length, float *corr)
{
    int search_window = length;
        
    float r_min = 10000;
    int attempts = 0;
    while(attempts < SPO2_SEARCH_ATTEMPTS)
    {
        int shift_start = rand() % search_window;
        int shift_stop = rand() % search_window;
        int len = abs(shift_stop - shift_start);
        
        if(len > length * 0.1)
        {
            attempts++;
            if(shift_start > shift_stop)
            {
                int temp = shift_stop;
                shift_stop = shift_start;
                shift_start = temp;
            }
            
            assert(len > 0);
            assert(shift_stop > shift_start);
            
            float r_curr = calculate_R(ir, red, start + shift_start, len, corr);
            if(r_curr < r_min) r_min = r_curr;
        }
    }
    
    return r_min;
}

int spo2_search_update(spo2_search_alg_t *alg, float ir, float red, spo2_search_result_t *result)
{
    alg->ir_ac = _builtin_dc_remover_step(&alg->dr_ir, ir);
    alg->red_ac = _builtin_dc_remover_step(&alg->dr_red, red);
    
    alg->buf.ir_buf[alg->current_count] = alg->ir_ac;
    alg->buf.red_buf[alg->current_count] = alg->red_ac;
    
    alg->current_count++;
    if(alg->current_count == SPO2_SEARCH_WINDOW)
    {
        alg->current_count = 0;
        float R = spo2_search_R(alg->buf.ir_buf, alg->buf.red_buf, 0, SPO2_SEARCH_WINDOW, NULL);
        result->R = R;
        return 1;
    }
    return 0;
}
