#include <xcore/channel.h>
#include <xcore/parallel.h>
#include <xcore/assert.h>
#include <xcore/hwtimer.h>
#include <stdio.h>

#include "src.h"
#include "asynchronous_fifo.h"
#include "asrc_timestamp_interpolation.h"



#define     SRC_N_CHANNELS                  (1)   // Total number of audio channels to be processed by SRC (minimum 1)
#define     SRC_N_INSTANCES                 (1)   // Number of instances (each usually run a logical core) used to process audio (minimum 1)
#define     SRC_CHANNELS_PER_INSTANCE       (SRC_N_CHANNELS/SRC_N_INSTANCES) // Calculated number of audio channels processed by each core
#define     SRC_N_IN_SAMPLES                (4)   // Number of samples per channel in each block passed into SRC each call
                                                  // Must be a power of 2 and minimum value is 4 (due to two /2 decimation stages)
#define     SRC_N_OUT_IN_RATIO_MAX          (5)   // Max ratio between samples out:in per processing step (44.1->192 is worst case)
#define     SRC_DITHER_SETTING              (0)   // Enables or disables quantisation of output with dithering to 24b
#define     SRC_MAX_NUM_SAMPS_OUT           (SRC_N_OUT_IN_RATIO_MAX * SRC_N_IN_SAMPLES)
#define     SRC_OUT_BUFF_SIZE               (SRC_CHANNELS_PER_INSTANCE * SRC_MAX_NUM_SAMPS_OUT) // Size of output buffer for SRC for each instance

/* Stuff that must be defined for lib_src */
#define     ASRC_N_IN_SAMPLES               (SRC_N_IN_SAMPLES) /* Used by SRC_STACK_LENGTH_MULT in src_mrhf_asrc.h */
#define     ASRC_N_CHANNELS                 (SRC_CHANNELS_PER_INSTANCE) /* Used by SRC_STACK_LENGTH_MULT in src_mrhf_asrc.h */



int fs_code(int frequency) {
    if(frequency == 44100) {
        return  FS_CODE_44;
    } else if(frequency == 48000) {
        return  FS_CODE_48;
    } else if(frequency == 88200) {
        return  FS_CODE_88;
    } else if(frequency == 96000) {
        return  FS_CODE_96;
    } else if(frequency == 176400) {
        return  FS_CODE_176;
    } else if(frequency == 192000) {
        return  FS_CODE_192;
    } else {
        xassert(0);
    }
    return -1;
}


typedef struct shedule_info_t{
    int num_channels;
    int channel_start_idx;
}shedule_info_t;


int calculate_job_share(   const int max_jobs,
                            int channels,
                            shedule_info_t *schedule){
    int channels_per_first_job = (channels + max_jobs - 1) / max_jobs; // Rounded up channels per max jobs
    int channels_per_last_job = 0;

    int num_jobs = 0;
    while(channels > 0){
        channels_per_last_job = channels;
        channels -= channels_per_first_job;
        schedule[num_jobs].num_channels = channels_per_first_job;
        schedule[num_jobs].channel_start_idx = channels_per_first_job * num_jobs;
        num_jobs++;
    };
    schedule[num_jobs - 1].num_channels = channels_per_last_job;

    // printf("num_jobs = %u channels_per_job: %u, channels_per_last_job: %u\n", num_jobs, channels_per_first_job, channels_per_last_job);

    return num_jobs;
}


DECLARE_JOB(do_asrc_group, (shedule_info_t*, void*));
void do_asrc_group(shedule_info_t *schedule, void *args){
    // printf("do_asrc_groupstart_idx: %u n_channels: %u\n", schedule->channel_start_idx, schedule->num_channels);
}



void par_function(int num_jobs, shedule_info_t shedule[], void * args){
    switch(num_jobs){
        case 0:
            return;
        break;
        case 1:
            PAR_JOBS(
                PJOB(do_asrc_group, (&shedule[0], args))
            );
        break;
        case 2:
            PAR_JOBS(
                PJOB(do_asrc_group, (&shedule[0], args)),
                PJOB(do_asrc_group, (&shedule[1], args))
            );
        break;
        case 3:
            PAR_JOBS(
                PJOB(do_asrc_group, (&shedule[0], args)),
                PJOB(do_asrc_group, (&shedule[1], args)),
                PJOB(do_asrc_group, (&shedule[2], args))
            );
        break;
        case 4:
            PAR_JOBS(
                PJOB(do_asrc_group, (&shedule[0], args)),
                PJOB(do_asrc_group, (&shedule[1], args)),
                PJOB(do_asrc_group, (&shedule[2], args)),
                PJOB(do_asrc_group, (&shedule[3], args))
            );
        break;
        case 5:
            PAR_JOBS(
                PJOB(do_asrc_group, (&shedule[0], args)),
                PJOB(do_asrc_group, (&shedule[1], args)),
                PJOB(do_asrc_group, (&shedule[2], args)),
                PJOB(do_asrc_group, (&shedule[3], args)),
                PJOB(do_asrc_group, (&shedule[4], args))
            );
        break;
        // case 6:
        //     PAR_JOBS(
        //         PJOB(do_asrc_group, (&shedule[0], args)),
        //         PJOB(do_asrc_group, (&shedule[1], args)),
        //         PJOB(do_asrc_group, (&shedule[2], args)),
        //         PJOB(do_asrc_group, (&shedule[3], args)),
        //         PJOB(do_asrc_group, (&shedule[4], args)),
        //         PJOB(do_asrc_group, (&shedule[5], args))
        //     );
        // break;
        default:
            xassert(0); // Too many jobs specified
        break;
    }
}

void par_asrc(int channels, void* args){

    const int max_jobs = 4;
    shedule_info_t shedule[max_jobs];

    int num_jobs = calculate_job_share(max_jobs, channels, shedule); // 36 ticks at 120MIPS
    int32_t t0 = get_reference_time();

    par_function(num_jobs, shedule, args); // about 55 ticks ticks overhead at 120MIPS 8 channels/ 4 threads
    int32_t t1 = get_reference_time();
    printf("time: %ld\n", t1 - t0);
 
}


void asrc_processor(chanend_t c_adat_rx_demux){
    while(1){

        asrc_state_t sASRCState[SRC_CHANNELS_PER_INSTANCE];                                   // ASRC state machine state
        int iASRCStack[SRC_CHANNELS_PER_INSTANCE][ASRC_STACK_LENGTH_MULT * SRC_N_IN_SAMPLES * 100]; // Buffer between filter stages
        asrc_ctrl_t sASRCCtrl[SRC_CHANNELS_PER_INSTANCE];                                     // Control structure
        asrc_adfir_coefs_t asrc_adfir_coefs;                                                  // Adaptive filter coefficients

        for(int ui = 0; ui < SRC_CHANNELS_PER_INSTANCE; ui++){
            // Set state, stack and coefs into ctrl structure
            sASRCCtrl[ui].psState                   = &sASRCState[ui];
            sASRCCtrl[ui].piStack                   = iASRCStack[ui];
            sASRCCtrl[ui].piADCoefs                 = asrc_adfir_coefs.iASRCADFIRCoefs;
        }

        uint32_t input_frequency = 48000;
        uint32_t output_frequency = 48000;

        uint64_t fs_ratio = 0;
        int ideal_fs_ratio = 0;


        // int sample_rate_set = 1;
        int inputFsCode = fs_code(input_frequency);
        int outputFsCode = fs_code(output_frequency);

        // asrc_input_t asrc_input = {{{0}}};
        // asrc_output_t asrc_output = {{{0}}};

        fs_ratio = asrc_init(inputFsCode, outputFsCode, sASRCCtrl, SRC_CHANNELS_PER_INSTANCE, SRC_N_IN_SAMPLES, SRC_DITHER_SETTING);
        ideal_fs_ratio = (fs_ratio + (1<<31)) >> 32;

        // // demuxed ADAT Rx
        // uint32_t rx_smux_setting = 0;       // No SMUX
        // int32_t adat_rx_samples[8] = {0};
        // uint32_t adat_rx_rate = 0;
        // unsigned word = 0;


        static unsigned channels = 0;
        printf("Channels: %u\n", channels);
        int32_t * args = NULL;

        par_asrc(channels, args);
        channels++;

        delay_milliseconds(500);

        // while(sample_rate_set){
        //     select{
        //         case inuint_byref(c_adat_rx_demux, word):
        //             unsigned new_adat_rx_rate = word;
        //             unsigned adat_rx_channels = inuint(c_adat_rx_demux);
        //             for(unsigned ch = 0; ch < adat_rx_channels; ch++){
        //                 adat_rx_samples[ch] = inuint(c_adat_rx_demux);
        //             }
        //             outct(c_adat_rx_demux, XS1_CT_END);
        //             chkct(c_adat_rx_demux, XS1_CT_END);

        //             if(new_adat_rx_rate != adat_rx_rate){
        //                 printstr("ADAT Rx sample rate change: "); printintln(new_adat_rx_rate);
        //                 adat_rx_rate = new_adat_rx_rate;
        //             }
        //         break;

        //         case c_asrc :> word:
        //             uint32_t current_i2s_rate = word;
        //             c_asrc <: adat_rx_samples[0];
        //             c_asrc <: adat_rx_samples[1];
        //             // printintln(adat_rx_samples[0]);
        //         break;
        //     } // select
        // }
    }
}


