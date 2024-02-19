#ifndef _ASRC_TASK_H_
#define _ASRC_TASK_H_

#include <stdint.h>
#include "src.h"

#define     MAX_ASRC_CHANNELS_TOTAL             8 // Used for buffer sizing and FIFO sizing (static)
#define     MAX_ASRC_THREADS                    4 // Sets upper limit of worker threads for ASRC task
#define     SRC_MAX_SRC_CHANNELS_PER_INSTANCE   4 // Sets maximum number of SRC per thread. Allocates all ASRC storage so minimise to save memeory

#define     SRC_N_IN_SAMPLES                    4 // Number of samples per channel in each block passed into SRC each call
                                                  // Must be a power of 2 and minimum value is 4 (due to two /2 decimation stages)
                                                  // Lower improves latency and memory usage but costs MIPS
#define     SRC_N_OUT_IN_RATIO_MAX              5 // Max ratio between samples out:in per processing step (44.1->192 is worst case)
#define     SRC_DITHER_SETTING                  0 // Enables or disables quantisation of output with dithering to 24b
#define     SRC_MAX_NUM_SAMPS_OUT               (SRC_N_OUT_IN_RATIO_MAX * SRC_N_IN_SAMPLES)

/* Stuff that must be defined for lib_src */
#define     ASRC_N_IN_SAMPLES                   (SRC_N_IN_SAMPLES) /* Used by SRC_STACK_LENGTH_MULT in src_mrhf_asrc.h */
#define     ASRC_N_CHANNELS                     (SRC_MAX_SRC_CHANNELS_PER_INSTANCE) /* Used by SRC_STACK_LENGTH_MULT in src_mrhf_asrc.h */


typedef struct asrc_in_out_t{
    int32_t input_samples[2][ASRC_N_IN_SAMPLES * MAX_ASRC_CHANNELS_TOTAL];  // Double buffer input array
    unsigned input_write_idx;                                               // Double buffer idx
    int ready_flag;                                                         // Flag to indicate ASRC ready to accept samples
    int32_t input_timestamp;                                                // Timestamp of last received input sample
    unsigned input_frequency;                                               // Nominal input sample rate 
    unsigned input_channel_count;                                           // As named..
    int32_t output_samples[SRC_MAX_NUM_SAMPS_OUT * MAX_ASRC_CHANNELS_TOTAL];// Output sample array
    uint32_t num_output_samples;                                            // How many sample periods worth of channels
    int32_t output_time_stamp;                                              // The consumption timestamp (set by consumer)
}asrc_in_out_t;

#ifdef __XC__
void asrc_processor(chanend c_asrc_input);
int pull_samples(int32_t * unsafe samples, int32_t consume_timestamp);
#else
#include <xcore/chanend.h>
void asrc_processor(chanend_t c_asrc_input);
int pull_samples(int32_t *samples, int32_t consume_timestamp);
#endif
void reset_asrc_fifo(void);

#endif // _ASRC_TASK_H_