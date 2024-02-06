#include <xcore/chanend.h>
#include <xcore/parallel.h>
#include <xcore/assert.h>
#include <xcore/hwtimer.h>
#include <stdio.h>

#include "src.h"
#include "asynchronous_fifo.h"
#include "asrc_timestamp_interpolation.h"

#include "app_config.h"

const int max_asrc_threads = 1;
const unsigned max_asrc_channels_total = 2; // Usef for FIFO

#define     SRC_MAX_SRC_CHANNELS_PER_INSTANCE   2 //unused


#define     SRC_N_CHANNELS                  (2)   // Total number of audio channels to be processed by SRC (minimum 1)
#define     SRC_N_INSTANCES                 (1)   // Number of instances (each usually run a logical core) used to process audio (minimum 1)
#define     SRC_CHANNELS_PER_INSTANCE       (SRC_N_CHANNELS/SRC_N_INSTANCES) // Calculated number of audio channels processed by each core
#define     SRC_N_IN_SAMPLES                (4)   // Number of samples per channel in each block passed into SRC each call
                                                  // Must be a power of 2 and minimum value is 4 (due to two /2 decimation stages)
#define     SRC_N_OUT_IN_RATIO_MAX          (5)   // Max ratio between samples out:in per processing step (44.1->192 is worst case)
#define     SRC_DITHER_SETTING              (0)   // Enables or disables quantisation of output with dithering to 24b
#define     SRC_MAX_NUM_SAMPS_OUT           (SRC_N_OUT_IN_RATIO_MAX * SRC_N_IN_SAMPLES)

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


typedef struct schedule_info_t{
    int num_channels;
    int channel_start_idx;
}schedule_info_t;


int calculate_job_share(const int max_asrc_threads,
                        int channels,
                        schedule_info_t *schedule){
    int channels_per_first_job = (channels + max_asrc_threads - 1) / max_asrc_threads; // Rounded up channels per max jobs
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

typedef struct asrc_in_out_t{
    unsigned nominal_input_rate;
    unsigned nominal_output_rate;
    int32_t input_samples[ASRC_N_IN_SAMPLES][SRC_N_CHANNELS];
    int32_t input_timestamp;
    int32_t output_samples[SRC_MAX_NUM_SAMPS_OUT][SRC_N_CHANNELS];
    uint32_t num_output_samples;
    int32_t output_time_stamp;
}asrc_in_out_t;



DECLARE_JOB(do_asrc_group, (schedule_info_t*, uint64_t, void*, int*, void*));
void do_asrc_group(schedule_info_t *schedule, uint64_t fs_ratio, void *args, int* num_output_samples, void * asrc_state){
    // printf("do_asrc_groupstart_idx: %u n_channels: %u\n", schedule->channel_start_idx, schedule->num_channels);
    asrc_in_out_t *asrc_io = args;
    *num_output_samples = asrc_process((int*)asrc_io->input_samples, (int*)asrc_io->output_samples, fs_ratio, asrc_state);
    // *num_output_samples = 4;
    // putchar('a');
    // printf("fs_ratio: %f\n", (float)fs_ratio / (float)(1ULL<<60));
    // printintln(*num_output_samples);
}


// about 55 ticks ticks overhead to fork and join at 120MIPS 8 channels/ 4 threads
int par_asrc(int num_jobs, schedule_info_t schedule[], uint64_t fs_ratio, void * args, void * asrc_state){
    int num_output_samples = 0;
    switch(num_jobs){
        case 0:
            return 0;
        break;
        case 1:
            PAR_JOBS(
                PJOB(do_asrc_group, (&schedule[0], fs_ratio, args, &num_output_samples, asrc_state))
            );
        break;
        case 2:
            PAR_JOBS(
                PJOB(do_asrc_group, (&schedule[0], fs_ratio, args, &num_output_samples, asrc_state)),
                PJOB(do_asrc_group, (&schedule[1], fs_ratio, args, &num_output_samples, asrc_state))
            );
        break;
        case 3:
            PAR_JOBS(
                PJOB(do_asrc_group, (&schedule[0], fs_ratio, args, &num_output_samples, asrc_state)),
                PJOB(do_asrc_group, (&schedule[1], fs_ratio, args, &num_output_samples, asrc_state)),
                PJOB(do_asrc_group, (&schedule[2], fs_ratio, args, &num_output_samples, asrc_state))
            );
        break;
        case 4:
            PAR_JOBS(
                PJOB(do_asrc_group, (&schedule[0], fs_ratio, args, &num_output_samples, asrc_state)),
                PJOB(do_asrc_group, (&schedule[1], fs_ratio, args, &num_output_samples, asrc_state)),
                PJOB(do_asrc_group, (&schedule[2], fs_ratio, args, &num_output_samples, asrc_state)),
                PJOB(do_asrc_group, (&schedule[3], fs_ratio, args, &num_output_samples, asrc_state))
            );
        break;
        case 5:
            PAR_JOBS(
                PJOB(do_asrc_group, (&schedule[0], fs_ratio, args, &num_output_samples, asrc_state)),
                PJOB(do_asrc_group, (&schedule[1], fs_ratio, args, &num_output_samples, asrc_state)),
                PJOB(do_asrc_group, (&schedule[2], fs_ratio, args, &num_output_samples, asrc_state)),
                PJOB(do_asrc_group, (&schedule[3], fs_ratio, args, &num_output_samples, asrc_state)),
                PJOB(do_asrc_group, (&schedule[4], fs_ratio, args, &num_output_samples, asrc_state))
            );
        break;
        // case 6:
        //     PAR_JOBS(
        //         PJOB(do_asrc_group, (&schedule[0], fs_ratio, args, &num_output_samples, asrc_state)),
        //         PJOB(do_asrc_group, (&schedule[1], fs_ratio, args, &num_output_samples, asrc_state)),
        //         PJOB(do_asrc_group, (&schedule[2], fs_ratio, args, &num_output_samples, asrc_state)),
        //         PJOB(do_asrc_group, (&schedule[3], fs_ratio, args, &num_output_samples, asrc_state)),
        //         PJOB(do_asrc_group, (&schedule[4], fs_ratio, args, &num_output_samples, asrc_state)),
        //         PJOB(do_asrc_group, (&schedule[5], fs_ratio, args, &num_output_samples, asrc_state))
        //     );
        // break;
        default:
            xassert(0); // Too many jobs specified
        break;
    }
    return num_output_samples;
}



///// FIFO
#define FIFO_LENGTH   100
int64_t array[ASYNCHRONOUS_FIFO_INT64_ELEMENTS(FIFO_LENGTH, max_asrc_channels_total)];
asynchronous_fifo_t *fifo = (asynchronous_fifo_t *)array;


void pull_samples(int32_t *samples, int32_t consume_timestamp){
    asynchronous_fifo_consume(fifo, samples, consume_timestamp);
    // printf("pull @ %ld\n", consume_timestamp);
}

extern uint32_t current_i2s_rate;

void asrc_processor(chanend_t c_adat_rx_demux){
    uint32_t input_frequency = 48000;
    uint32_t output_frequency = 48000;

    int channels = 2;

    static int interpolation_ticks_2D[6][6] = {
        {  2268, 2268, 2268, 2268, 2268, 2268},
        {  2083, 2083, 2083, 2083, 2083, 2083},
        {  2268, 2268, 1134, 1134, 1134, 1134},
        {  2083, 2083, 1042, 1042, 1042, 1042},
        {  2268, 2268, 1134, 1134,  567,  567},
        {  2083, 2083, 1042, 1042,  521,  521}
    };
    

    while(1){
        int audio_format_change = 0;

        asrc_in_out_t asrc_io = {0};

        // Frequency info
        int inputFsCode = fs_code(input_frequency);
        int outputFsCode = fs_code(output_frequency);
        int interpolation_ticks = interpolation_ticks_2D[inputFsCode][outputFsCode];
        
        ///// FIFO
        asynchronous_fifo_init(fifo, channels, FIFO_LENGTH);
        asynchronous_fifo_init_PID_fs_codes(fifo, inputFsCode, outputFsCode);

        // demuxed ADAT Rx
        int32_t adat_rx_samples[8] = {0};

        // SCHEDULER
        schedule_info_t schedule[max_asrc_threads];
        int num_jobs = calculate_job_share(max_asrc_threads, channels, schedule);
        printf("num_jobs: %d, max_asrc_threads: %d, channels: %d\n", num_jobs, max_asrc_threads, channels);
        for(int i = 0; i < num_jobs; i++){
            printf("schedule: %d, num_channels: %d, channel_start_idx: %d\n", i, schedule[i].num_channels, schedule[i].channel_start_idx);
        }

        int channels_per_instance = schedule[0].num_channels;

        // ASRC
        asrc_state_t sASRCState[max_asrc_threads][SRC_CHANNELS_PER_INSTANCE];                                   // ASRC state machine state
        int iASRCStack[max_asrc_threads][SRC_CHANNELS_PER_INSTANCE][ASRC_STACK_LENGTH_MULT * SRC_N_IN_SAMPLES * 1]; // Buffer between filter stages
        asrc_ctrl_t sASRCCtrl[max_asrc_threads][SRC_CHANNELS_PER_INSTANCE];                                     // Control structure
        asrc_adfir_coefs_t asrc_adfir_coefs[max_asrc_threads];                                                  // Adaptive filter coefficients

        for(int instance = 0; instance < max_asrc_threads; instance++){
            for(int ch = 0; ch < channels_per_instance; ch++){
                // Set state, stack and coefs into ctrl structure
                sASRCCtrl[instance][ch].psState                   = &sASRCState[instance][ch];
                sASRCCtrl[instance][ch].piStack                   = iASRCStack[instance][ch];
                sASRCCtrl[instance][ch].piADCoefs                 = asrc_adfir_coefs[instance].iASRCADFIRCoefs;
            }
        }

        uint64_t fs_ratio = 0;
        int ideal_fs_ratio = 0;

        fs_ratio = asrc_init(inputFsCode, outputFsCode, sASRCCtrl[0], channels_per_instance, SRC_N_IN_SAMPLES, SRC_DITHER_SETTING);
        ideal_fs_ratio = (fs_ratio + (1<<31)) >> 32;
        printf("ideal_fs_ratio: %d\n", ideal_fs_ratio);

        unsigned asrc_in_counter = 0;
        const int xscope_used = 0;

        while(!audio_format_change){

            unsigned new_adat_rx_rate = chanend_in_word(c_adat_rx_demux);
            asrc_io.input_timestamp = get_reference_time();
            unsigned adat_rx_channels = chanend_in_word(c_adat_rx_demux);
            for(unsigned ch = 0; ch < adat_rx_channels; ch++){
                adat_rx_samples[ch] = chanend_in_word(c_adat_rx_demux);
            }

            for(int i = 0; i < SRC_N_CHANNELS; i++){
                asrc_io.input_samples[asrc_in_counter][i] = adat_rx_samples[i];
            }

            if(++asrc_in_counter == SRC_N_IN_SAMPLES){
                asrc_in_counter = 0;
                int num_output_samples = par_asrc(num_jobs, schedule, fs_ratio, &asrc_io, sASRCCtrl[0]);
                int ts = asrc_timestamp_interpolation(asrc_io.input_timestamp, sASRCCtrl[0], interpolation_ticks);
                int error = asynchronous_fifo_produce(fifo, &asrc_io.output_samples[0][0], num_output_samples, ts, xscope_used);
                // printintln(num_output_samples);
                // printf("push @ %d\n", asrc_io.input_timestamp);
                fs_ratio = (((int64_t)ideal_fs_ratio) << 32) + (error * (int64_t) ideal_fs_ratio);
                // printintln(error);
                static int print_counter = 0;
                if(++print_counter == 50000){
                    printf("depth: %lu\n", (fifo->write_ptr - fifo->read_ptr + fifo->max_fifo_depth) % fifo->max_fifo_depth);
                    print_counter = 0;
                }
                // printf("ratio: %llu\n", fs_ratio);

            }

            if(new_adat_rx_rate != input_frequency){
                if(new_adat_rx_rate != 0){
                    printf("ADAT Rx sample rate change: %u", new_adat_rx_rate);
                    input_frequency = new_adat_rx_rate;
                    audio_format_change = 1;
                }
            }

            if(current_i2s_rate != output_frequency){
                if(current_i2s_rate != 0){
                    output_frequency = current_i2s_rate;
                    audio_format_change = 1;
                }
            }
        }
    }
}


