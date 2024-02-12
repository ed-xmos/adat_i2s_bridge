#include <xcore/chanend.h>
#include <xcore/parallel.h>
#include <xcore/assert.h>
#include <xcore/hwtimer.h>
#include <xcore/interrupt.h>
#include <xcore/interrupt_wrappers.h>
#include <xcore/triggerable.h>

#include <stdio.h>
#include <print.h>
#include <string.h>

#include "asrc_task.h"
#include "asynchronous_fifo.h"
#include "asrc_timestamp_interpolation.h"

#include "adat_wrapper.h"

#include "app_config.h"

int asrc_channel_count = 8;                 // Current channel count (dynamic)

 __attribute__ ((weak))
unsigned receive_asrc_input_samples(chanend c_asrc_input_samples, asrc_in_out_t *asrc_io, unsigned asrc_channel_count, unsigned *new_input_rate){
    printstrln("ERROR: Please define an appropriate ASRC receive samples function.");
    while(1);

    /* Something like this:

    timer tmr;
    tmr :> asrc_io.input_timestamp;
    new_input_rate = inuint(c_asrc_input_samples);

    // Pack into array properly LRLRLRLR or 123412341234 etc.
    for(int i = 0; i < asrc_channel_count; i++){
        int idx = i + asrc_channel_count * asrc_in_counter;
        asrc_io.input_samples[input_write_idx][idx] = adat_rx_samples[i];
    }

    if(++asrc_in_counter == SRC_N_IN_SAMPLES){
        asrc_in_counter = 0;
    }

    return asrc_in_counter;
    */
}


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

typedef struct isr_ctx_t{
    chanend_t c_asrc_input;
    chanend_t c_buff_idx;
    asrc_in_out_t *asrc_io;
} isr_ctx_t;


typedef struct schedule_info_t{
    int num_channels;
    int channel_start_idx;
}schedule_info_t;


int calculate_job_share(int asrc_channel_count,
                        schedule_info_t *schedule){
    int channels_per_first_job = (asrc_channel_count + MAX_ASRC_THREADS - 1) / MAX_ASRC_THREADS; // Rounded up channels per max jobs
    int channels_per_last_job = 0;

    int num_jobs = 0;
    while(asrc_channel_count > 0){
        channels_per_last_job = asrc_channel_count;
        asrc_channel_count -= channels_per_first_job;
        schedule[num_jobs].num_channels = channels_per_first_job;
        schedule[num_jobs].channel_start_idx = channels_per_first_job * num_jobs;
        num_jobs++;
    };
    schedule[num_jobs - 1].num_channels = channels_per_last_job;

    // printf("num_jobs = %u channels_per_job: %u, channels_per_last_job: %u\n", num_jobs, channels_per_first_job, channels_per_last_job);

    return num_jobs;
}


DECLARE_JOB(do_asrc_group, (schedule_info_t*, uint64_t, asrc_in_out_t*, unsigned, int*, asrc_ctrl_t*));
void do_asrc_group(schedule_info_t *schedule, uint64_t fs_ratio, asrc_in_out_t * asrc_io, unsigned input_write_idx, int* num_output_samples, asrc_ctrl_t asrc_ctrl[]){

    int num_worker_channels = schedule->num_channels;
    int worker_channel_start_idx = schedule->channel_start_idx;

    // Pack into the frame this instance of ASRC expects
    int input_samples[ASRC_N_IN_SAMPLES * MAX_ASRC_CHANNELS_TOTAL];
    for(int i = 0; i < ASRC_N_IN_SAMPLES * num_worker_channels; i++){
        int rd_idx = i % num_worker_channels + (i / num_worker_channels) * asrc_channel_count + worker_channel_start_idx;
        input_samples[i] = asrc_io->input_samples[input_write_idx][rd_idx];
    }

    int output_samples[SRC_MAX_NUM_SAMPS_OUT * MAX_ASRC_CHANNELS_TOTAL];

    *num_output_samples = asrc_process(input_samples, output_samples, fs_ratio, asrc_ctrl);

    // Unpack
    for(int i = 0; i < *num_output_samples * num_worker_channels; i++){
        int wr_idx = i % num_worker_channels + (i / num_worker_channels) * asrc_channel_count + worker_channel_start_idx;
        asrc_io->output_samples[wr_idx] = output_samples[i];
    }
}


// about 55 ticks ticks overhead to fork and join at 120MIPS 8 channels/ 4 threads
int par_asrc(int num_jobs, schedule_info_t schedule[], uint64_t fs_ratio, asrc_in_out_t * asrc_io, unsigned input_write_idx, asrc_ctrl_t asrc_ctrl[MAX_ASRC_THREADS][SRC_MAX_SRC_CHANNELS_PER_INSTANCE]){
    int num_output_samples = 0;

    switch(num_jobs){
        case 0:
            return 0; // Nothing to do
        break;
        case 1:
            PAR_JOBS(
                PJOB(do_asrc_group, (&schedule[0], fs_ratio, asrc_io, input_write_idx, &num_output_samples, asrc_ctrl[0]))
            );
        break;
#if MAX_ASRC_THREADS > 1
        case 2:
            PAR_JOBS(
                PJOB(do_asrc_group, (&schedule[0], fs_ratio, asrc_io, input_write_idx, &num_output_samples, asrc_ctrl[0])),
                PJOB(do_asrc_group, (&schedule[1], fs_ratio, asrc_io, input_write_idx, &num_output_samples, asrc_ctrl[1]))
            );
        break;
#endif
#if MAX_ASRC_THREADS > 2
        case 3:
            PAR_JOBS(
                PJOB(do_asrc_group, (&schedule[0], fs_ratio, asrc_io, input_write_idx, &num_output_samples, asrc_ctrl[0])),
                PJOB(do_asrc_group, (&schedule[1], fs_ratio, asrc_io, input_write_idx, &num_output_samples, asrc_ctrl[1])),
                PJOB(do_asrc_group, (&schedule[2], fs_ratio, asrc_io, input_write_idx, &num_output_samples, asrc_ctrl[2]))
            );
        break;
#endif
#if MAX_ASRC_THREADS > 3
        case 4:
            PAR_JOBS(
                PJOB(do_asrc_group, (&schedule[0], fs_ratio, asrc_io, input_write_idx, &num_output_samples, asrc_ctrl[0])),
                PJOB(do_asrc_group, (&schedule[1], fs_ratio, asrc_io, input_write_idx, &num_output_samples, asrc_ctrl[1])),
                PJOB(do_asrc_group, (&schedule[2], fs_ratio, asrc_io, input_write_idx, &num_output_samples, asrc_ctrl[2])),
                PJOB(do_asrc_group, (&schedule[3], fs_ratio, asrc_io, input_write_idx, &num_output_samples, asrc_ctrl[3]))
            );
        break;
#endif
#if MAX_ASRC_THREADS > 4
        case 5:
            PAR_JOBS(
                PJOB(do_asrc_group, (&schedule[0], fs_ratio, asrc_io, input_write_idx, &num_output_samples, asrc_ctrl[0])),
                PJOB(do_asrc_group, (&schedule[1], fs_ratio, asrc_io, input_write_idx, &num_output_samples, asrc_ctrl[1])),
                PJOB(do_asrc_group, (&schedule[2], fs_ratio, asrc_io, input_write_idx, &num_output_samples, asrc_ctrl[2])),
                PJOB(do_asrc_group, (&schedule[3], fs_ratio, asrc_io, input_write_idx, &num_output_samples, asrc_ctrl[3])),
                PJOB(do_asrc_group, (&schedule[4], fs_ratio, asrc_io, input_write_idx, &num_output_samples, asrc_ctrl[4]))
            );
        break;
#endif
#if MAX_ASRC_THREADS > 5
        case 6:
            PAR_JOBS(
                PJOB(do_asrc_group, (&schedule[0], fs_ratio, asrc_io, input_write_idx, &num_output_samples, asrc_ctrl[0])),
                PJOB(do_asrc_group, (&schedule[1], fs_ratio, asrc_io, input_write_idx, &num_output_samples, asrc_ctrl[1])),
                PJOB(do_asrc_group, (&schedule[2], fs_ratio, asrc_io, input_write_idx, &num_output_samples, asrc_ctrl[2])),
                PJOB(do_asrc_group, (&schedule[3], fs_ratio, asrc_io, input_write_idx, &num_output_samples, asrc_ctrl[3])),
                PJOB(do_asrc_group, (&schedule[4], fs_ratio, asrc_io, input_write_idx, &num_output_samples, asrc_ctrl[4])),
                PJOB(do_asrc_group, (&schedule[5], fs_ratio, asrc_io, input_write_idx, &num_output_samples, asrc_ctrl[5]))
            );
        break;
#endif
        default:
            xassert(0); // Too many or no jobs specified
        break;
    }
    return num_output_samples;
}



///// FIFO declaration. Global to allow producer and consumer to access it
#define FIFO_LENGTH   100
int64_t array[ASYNCHRONOUS_FIFO_INT64_ELEMENTS(FIFO_LENGTH, MAX_ASRC_CHANNELS_TOTAL)];
asynchronous_fifo_t *fifo = (asynchronous_fifo_t *)array;


int pull_samples(int32_t *samples, int32_t consume_timestamp){
    asynchronous_fifo_consume(fifo, samples, consume_timestamp);

    return asrc_channel_count;
}

void reset_fifo(void){
    asynchronous_fifo_reset_consumer(fifo);
    memset(fifo->buffer, 0, fifo->channel_count * fifo->max_fifo_depth * sizeof(int));
}

// Set by audio_hub
extern uint32_t current_i2s_rate;


volatile int ready_flag = 0;

// This is fired each time a sample is received
DEFINE_INTERRUPT_CALLBACK(ASRC_ISR_GRP, asrc_samples_rx_isr_handler, app_data)
{
    isr_ctx_t *isr_ctx = app_data;
    chanend_t c_asrc_input = isr_ctx->c_asrc_input;
    chanend_t c_buff_idx = isr_ctx->c_buff_idx;
    asrc_in_out_t *asrc_io = isr_ctx->asrc_io;
    
    unsigned asrc_in_counter = receive_asrc_input_samples(c_asrc_input, asrc_io, asrc_channel_count, &(asrc_io->input_frequency));

    if(asrc_in_counter == 0 && ready_flag){
        chanend_out_byte(c_buff_idx, (uint8_t)asrc_io->input_write_idx);
        asrc_io->input_write_idx ^= 1;
    }

}




// DECLARE_INTERRUPT_PERMITTED(void, asrc_processor_, chanend_t c_asrc_input);
DEFINE_INTERRUPT_PERMITTED(ASRC_ISR_GRP, void, asrc_processor_, chanend_t c_asrc_input, chanend_t c_buff_idx){
// void asrc_processor_(chanend_t c_asrc_input){
    uint32_t input_frequency = 48000;
    uint32_t output_frequency = 48000;

    asrc_in_out_t asrc_io = {{{0}}};


    static int interpolation_ticks_2D[6][6] = {
        {  2268, 2268, 2268, 2268, 2268, 2268},
        {  2083, 2083, 2083, 2083, 2083, 2083},
        {  2268, 2268, 1134, 1134, 1134, 1134},
        {  2083, 2083, 1042, 1042, 1042, 1042},
        {  2268, 2268, 1134, 1134,  567,  567},
        {  2083, 2083, 1042, 1042,  521,  521}
    };
    
    isr_ctx_t isr_ctx = {c_asrc_input, c_buff_idx, &asrc_io};

    triggerable_setup_interrupt_callback(c_asrc_input, &isr_ctx, INTERRUPT_CALLBACK(asrc_samples_rx_isr_handler));
    triggerable_enable_trigger(c_asrc_input);
    interrupt_unmask_all();

    while(1){
        int audio_format_change = 0;

        // Frequency info
        int inputFsCode = fs_code(input_frequency);
        int outputFsCode = fs_code(output_frequency);
        int interpolation_ticks = interpolation_ticks_2D[inputFsCode][outputFsCode];
        printf("Input fs: %lu Output fs: %lu\n", input_frequency, output_frequency);
        
        ///// FIFO init
        asynchronous_fifo_init(fifo, asrc_channel_count, FIFO_LENGTH);
        asynchronous_fifo_init_PID_fs_codes(fifo, inputFsCode, outputFsCode);
        printf("FIFO init channels: %d\n", asrc_channel_count);


        // SCHEDULER init
        schedule_info_t schedule[MAX_ASRC_THREADS];
        int num_jobs = calculate_job_share(asrc_channel_count, schedule);
        printf("num_jobs: %d, MAX_ASRC_THREADS: %d, asrc_channel_count: %d\n", num_jobs, MAX_ASRC_THREADS, asrc_channel_count);
        for(int i = 0; i < num_jobs; i++){
            printf("schedule: %d, num_channels: %d, channel_start_idx: %d\n", i, schedule[i].num_channels, schedule[i].channel_start_idx);
        }

        int max_channels_per_instance = schedule[0].num_channels;
        printf("max_channels_per_instance: %d\n", max_channels_per_instance);

        // ASRC init
        asrc_state_t sASRCState[MAX_ASRC_THREADS][SRC_MAX_SRC_CHANNELS_PER_INSTANCE];                                   // ASRC state machine state
        int iASRCStack[MAX_ASRC_THREADS][SRC_MAX_SRC_CHANNELS_PER_INSTANCE][ASRC_STACK_LENGTH_MULT * SRC_N_IN_SAMPLES ];// Buffer between filter stages
        asrc_ctrl_t sASRCCtrl[MAX_ASRC_THREADS][SRC_MAX_SRC_CHANNELS_PER_INSTANCE];                                     // Control structure
        asrc_adfir_coefs_t asrc_adfir_coefs[MAX_ASRC_THREADS];                                                          // Adaptive filter coefficients

        uint64_t fs_ratio = 0;
        int ideal_fs_ratio = 0;

        for(int instance = 0; instance < num_jobs; instance++){
            for(int ch = 0; ch < max_channels_per_instance; ch++){
                // Set state, stack and coefs into ctrl structure
                sASRCCtrl[instance][ch].psState                   = &sASRCState[instance][ch];
                sASRCCtrl[instance][ch].piStack                   = iASRCStack[instance][ch];
                sASRCCtrl[instance][ch].piADCoefs                 = asrc_adfir_coefs[instance].iASRCADFIRCoefs;
            }
            fs_ratio = asrc_init(inputFsCode, outputFsCode, sASRCCtrl[instance], max_channels_per_instance, SRC_N_IN_SAMPLES, SRC_DITHER_SETTING);
            printf("ASRC init instance: %d ptr: %p\n", instance, sASRCCtrl[instance]);
        }

        // Timing check vars. Includes ASRC, timestamp interpolation and FIFO push
        int32_t asrc_process_time_limit = (XS1_TIMER_HZ / input_frequency) * SRC_N_IN_SAMPLES;
        printf("ASRC process_time_limit: %ld\n", asrc_process_time_limit);
        int32_t asrc_peak_processing_time = 0;


        ideal_fs_ratio = (fs_ratio + (1<<31)) >> 32;
        printf("ideal_fs_ratio: %d\n", ideal_fs_ratio);

        const int xscope_used = 0;

        while(!audio_format_change){

            ready_flag = 1; // Signal we are ready to consume a frame of input samples

            // Wait for block of samples
            unsigned input_write_idx = (unsigned)chanend_in_byte(c_buff_idx);
            unsigned new_input_rate = asrc_io.input_frequency;

            int32_t t0 = get_reference_time();
            int num_output_samples = par_asrc(num_jobs, schedule, fs_ratio, &asrc_io, input_write_idx, sASRCCtrl);
            int ts = asrc_timestamp_interpolation(asrc_io.input_timestamp, sASRCCtrl[0], interpolation_ticks);
            int error = asynchronous_fifo_produce(fifo, &asrc_io.output_samples[0], num_output_samples, ts, xscope_used);

            fs_ratio = (((int64_t)ideal_fs_ratio) << 32) + (error * (int64_t) ideal_fs_ratio);

            int32_t t1 = get_reference_time();
            if(t1 - t0 > asrc_peak_processing_time){
                asrc_peak_processing_time = t1 - t0;
                printintln(asrc_peak_processing_time);
            }

            // Remove me. This is here to monitor PID loop convergence during dev
            static int print_counter = 0;
            if(++print_counter == 50000){
                printf("depth: %lu\n", (fifo->write_ptr - fifo->read_ptr + fifo->max_fifo_depth) % fifo->max_fifo_depth);
                print_counter = 0;
            }

            if(new_input_rate != input_frequency){
                if(new_input_rate != 0){
                    input_frequency = new_input_rate;
                    audio_format_change = 1;
                }
            }

            if(current_i2s_rate != output_frequency){
                if(current_i2s_rate != 0){
                    output_frequency = current_i2s_rate;
                    audio_format_change = 1;
                }
            }
        } // while !audio_format_change

        // We have broken out of the loop due to a format change
        ready_flag = 0;
        asynchronous_fifo_reset_producer(fifo);
    } // while 1
}

void asrc_processor(chanend_t c_asrc_input){
    // We use a single chanend to send the buffer IDX from the ISR of this task back to asrc task and sync
    chanend_t c_buff_idx = chanend_alloc();
    chanend_set_dest(c_buff_idx, c_buff_idx);

    // Run the ASRC task with stack set aside for an ISR
    INTERRUPT_PERMITTED(asrc_processor_)(c_asrc_input, c_buff_idx);
}


