#ifndef _ASRC_TASK_H_
#define _ASRC_TASK_H_

#include <stdint.h>
#include "src.h"

#define     MAX_ASRC_CHANNELS_TOTAL             8 // Used for buffer sizing and FIFO sizing (static)
#define     MAX_ASRC_THREADS                    4 // Sets upper limit of worker threads for ASRC task
#define     SRC_MAX_SRC_CHANNELS_PER_INSTANCE   4 // Sets maximum number of SRC per thread. Allocates all ASRC storage so minimise to save memeory

#define     SRC_N_IN_SAMPLES                    4 // Number of samples per channel in each block passed into SRC each call
                                                  // Must be a power of 2 and minimum value is 4 (due to two /2 decimation stages)
#define     SRC_N_OUT_IN_RATIO_MAX              5 // Max ratio between samples out:in per processing step (44.1->192 is worst case)
#define     SRC_DITHER_SETTING                  0 // Enables or disables quantisation of output with dithering to 24b
#define     SRC_MAX_NUM_SAMPS_OUT               (SRC_N_OUT_IN_RATIO_MAX * SRC_N_IN_SAMPLES)

/* Stuff that must be defined for lib_src */
#define     ASRC_N_IN_SAMPLES               (SRC_N_IN_SAMPLES) /* Used by SRC_STACK_LENGTH_MULT in src_mrhf_asrc.h */
#define     ASRC_N_CHANNELS                 (SRC_MAX_SRC_CHANNELS_PER_INSTANCE) /* Used by SRC_STACK_LENGTH_MULT in src_mrhf_asrc.h */


typedef struct asrc_in_out_t{
    int32_t input_samples[ASRC_N_IN_SAMPLES * MAX_ASRC_CHANNELS_TOTAL];
    int32_t input_timestamp;
    int32_t output_samples[SRC_MAX_NUM_SAMPS_OUT * MAX_ASRC_CHANNELS_TOTAL];
    uint32_t num_output_samples;
    int32_t output_time_stamp;
}asrc_in_out_t;

#ifdef __XC__
void asrc_processor(chanend c_asrc_input);
int pull_samples(int32_t * unsafe samples, int32_t consume_timestamp);
#else
#include <xcore/chanend.h>
void asrc_processor(chanend_t c_asrc_input);
int pull_samples(int32_t *samples, int32_t consume_timestamp);
#endif


#endif // _ASRC_TASK_H_