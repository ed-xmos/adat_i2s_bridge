#include <platform.h>
#include <xs1.h>
#include <stdint.h>
#include <string.h>
#include <print.h>
#include "xassert.h"

#include "i2s.h"
#include "i2c.h"
#include "adat_rx.h"
#include "adat_tx.h"
#include "adat_tx.h"

extern void board_setup(void);
extern void AudioHwInit(void);
extern void AudioHwConfig(unsigned samFreq, unsigned mClk, unsigned dsdMode, unsigned sampRes_DAC, unsigned sampRes_ADC);
extern unsafe client interface i2c_master_if i_i2c_client;

// ADAT resources
on tile[0]: buffered in port:32 p_adat_in =                 PORT_ADAT_IN;
on tile[1]: buffered out port:32 p_adat_out =               PORT_ADAT_OUT;
on tile[1]: clock mck_blk =                                 XS1_CLKBLK_2;

// I2C resources (defined in audiohw.c)
extern port p_scl;
extern port p_sda;

// GPIO resources
on tile[0]: in port p_buttons =                             XS1_PORT_4E;
on tile[0]: out port p_leds =                               XS1_PORT_4F;

// I2S resources
#define NUM_I2S_DAC_LINES                                   2
#define NUM_I2S_ADC_LINES                                   1
#define I2S_DATA_BITS                                       32
on tile[1]: in port p_mclk =                                PORT_MCLK_IN;
on tile[1]: buffered out port:32 p_lrclk =                  PORT_I2S_LRCLK;
on tile[1]: out port p_bclk =                               PORT_I2S_BCLK;
on tile[1]: buffered out port:32 p_dac[NUM_I2S_DAC_LINES] = {PORT_I2S_DAC0, PORT_I2S_DAC1};
on tile[1]: buffered in port:32 p_adc[NUM_I2S_ADC_LINES] =  {PORT_I2S_ADC0};
on tile[1]: clock bclk =                                    XS1_CLKBLK_1;

#define ADAT_MAX_SAMPLES        8
typedef struct adat_state_t{
    int32_t samples[ADAT_MAX_SAMPLES];
    int32_t rx_time_latest;
    int32_t rx_time_last;
    uint8_t user_bits;
    uint8_t channel;
}adat_state_t;

enum audio_port_idx{
    IO_I2S = 0,
    IO_ADAT_RX,
    IO_ADAT_TX
};

enum adat_smux_setting{
    SMUX_NONE = 0,
    SMUX_II,
    SMUX_IV
};



#define RATE_LOWER(rate, ppm) ((uint32_t)((float)rate * (1.0 - (float)ppm / 1e6) + 0.5))
#define RATE_HIGHER(rate, ppm) ((uint32_t)((float)rate * (1.0 + (float)ppm / 1e6)+ 0.5))
#define CHECK_RATE(sps, rate, ppm) if (sps > RATE_LOWER(rate, ppm) && sps < RATE_HIGHER(rate, ppm)){return rate;}

inline uint32_t get_normal_sample_rate(uint32_t samples_per_second){
    const uint32_t ppm_tolerance = 10000;

    // Check in reverse order so high rates take fewer cycles
    CHECK_RATE(samples_per_second, 192000, ppm_tolerance);
    CHECK_RATE(samples_per_second, 176400, ppm_tolerance);
    CHECK_RATE(samples_per_second, 96000, ppm_tolerance);
    CHECK_RATE(samples_per_second, 88200, ppm_tolerance);
    CHECK_RATE(samples_per_second, 48000, ppm_tolerance);
    CHECK_RATE(samples_per_second, 44100, ppm_tolerance);

    return 0; // Invalid rate found
}

// in asrc_task.c
extern "C" {void pull_samples(int32_t *samples, int32_t consume_timestamp);}

// Global to allow asrc_task to poll it
uint32_t current_i2s_rate = 0;                  // Set to invalid


void audio_hub( chanend c_adat_tx,
                server i2s_frame_callback_if i_i2s,
                chanend c_sr_change
#if DIRECT_ADAT_RX
                , chanend c_adat_rx_demux
#endif
) {
    // Approx rate calculation based on sample counting
    timer tmr;
    int32_t rate_measurement_trigger = 0;
    const uint32_t measurement_rate_hz = 10;
    const int32_t rate_measurement_period = XS1_TIMER_HZ / measurement_rate_hz;
    tmr :> rate_measurement_trigger;
    rate_measurement_trigger += rate_measurement_period;

    // i2s master state
    uint32_t master_clock_frequency = 24576000;
    uint32_t i2s_set_sample_rate = 48000;
    uint8_t i2s_master_sample_rate_change = 1; // Force config first time around

    // i2s sample rate measurement state
    uint32_t i2s_sample_period_count = 0;
    uint8_t measured_i2s_sample_rate_change = 1;    // Force new SR as measured

    uint8_t mute = 1;

    while(1) {
        select{
            case tmr when timerafter(rate_measurement_trigger) :> int _:
                // Measure I2S
                unsigned samples_per_second = i2s_sample_period_count * measurement_rate_hz;
                unsigned measured_i2s_rate = get_normal_sample_rate(samples_per_second);
                i2s_sample_period_count = 0;
                if((measured_i2s_rate != 0) && (current_i2s_rate != measured_i2s_rate)){
                    mute = 1;
                    measured_i2s_sample_rate_change = 1;
                    current_i2s_rate = measured_i2s_rate;
                    // asrc_input.nominal_output_rate = measured_i2s_rate;
                    printstrln("Measured I2S sample rate change: "); printintln(measured_i2s_rate);
                }
                rate_measurement_trigger += rate_measurement_period;
            break;

            case i_i2s.init(i2s_config_t &?i2s_config, tdm_config_t &?tdm_config):
                i2s_config.mclk_bclk_ratio = (master_clock_frequency / (i2s_set_sample_rate*2*I2S_DATA_BITS));
                printstr("i2s init: "); printintln(i2s_set_sample_rate);
                AudioHwConfig(i2s_set_sample_rate, master_clock_frequency, 0, 24, 24);
                i2s_config.mode = I2S_MODE_I2S;
                mute = 0;
            break;

            case i_i2s.restart_check() -> i2s_restart_t restart:
                // Inform the I2S slave whether it should restart or exit
                i2s_sample_period_count++;
                if(measured_i2s_sample_rate_change){
                    printstrln("measured_i2s_sample_rate_change");
                    measured_i2s_sample_rate_change = 0;
                    restart = I2S_RESTART;
                    break;
                }
                if(i2s_master_sample_rate_change){
                    printstrln("i2s_master_sample_rate_change");
                    i2s_master_sample_rate_change = 0;
                    restart = I2S_RESTART;
                    break;
                }
                restart = I2S_NO_RESTART;
                // printchar('i');
            break;

            case i_i2s.receive(size_t num_in, int32_t samples[num_in]):
                // Handle a received sample
            break;

            case i_i2s.send(size_t num_out, int32_t samples[num_out]):
                int32_t consume_timestamp;
                tmr :> consume_timestamp;
                // samples[0] = adat_rx_samples[0];
                // samples[1] = adat_rx_samples[1];
                int32_t asrc_out;
                pull_samples(&asrc_out, consume_timestamp);
                samples[0] = asrc_out;
                samples[1] = 0;
                samples[2] = asrc_out;
                samples[3] = 0;
            break;

            case c_sr_change :> unsigned id:
                if(id == IO_I2S){
                    unsigned new_sr;
                    c_sr_change :> new_sr;
                    mute = 1;
                    master_clock_frequency = (new_sr % 48000 == 0) ? 24576000 : 22579200;
                    // Ensure mclk is correct. This will not be needed for slave as set externally
                    // AudioHwConfig(new_sr, master_clock_frequency, 0, 24, 24);
                    i2s_set_sample_rate = new_sr;
                    i2s_master_sample_rate_change = 1;
                }
                if(id == IO_ADAT_TX){
                    unsigned new_smux;
                    c_sr_change :> new_smux;
                    printstr("adat tx smux: ");printintln(new_smux);
                }
            break;
        } // select
    }// while(1)
}


void adat_rx_demux(chanend c_adat_rx, chanend c_adat_rx_demux, chanend c_smux_change_adat_rx)
{
    unsigned word = 0;
    adat_state_t adat_state[2] = {{{0}}};
    uint32_t adat_state_idx = 0;
    uint32_t adat_group_idx = 0;
    uint32_t smux_setting = SMUX_NONE;
    uint32_t new_smux_setting = smux_setting;

    timer tmr;
    int32_t rate_measurement_trigger = 0;
    const uint32_t measurement_rate_hz = 10;
    const int32_t rate_measurement_period = XS1_TIMER_HZ / measurement_rate_hz;
    uint32_t measured_adat_rate = 0;
    uint32_t sample_period_count = 0;

    tmr :> rate_measurement_trigger;
    rate_measurement_trigger += rate_measurement_period;

    while(1)
    {
        select
        {
            case inuint_byref(c_adat_rx, word):
                // grab timestamp first for minimum jitter
                uint32_t time_stamp;
                tmr :> time_stamp;
                uint8_t other_buff_idx = adat_state_idx ^ 1;
                if(word & 0x01){
                    adat_state[adat_state_idx].rx_time_last = adat_state[other_buff_idx].rx_time_latest;
                    adat_state[adat_state_idx].rx_time_latest = time_stamp;
                    adat_state[adat_state_idx].channel = 0;
                    adat_state[adat_state_idx].user_bits = word >> 4;
                } else {
                    adat_state[adat_state_idx].samples[adat_state[adat_state_idx].channel] = word << 4;
                    adat_state[adat_state_idx].channel++;

                    switch(smux_setting){
                        // No SMUX - 12345678
                        case SMUX_NONE:
                            if(adat_state[adat_state_idx].channel == 8){
                                outuint(c_adat_rx_demux, measured_adat_rate);
                                outuint(c_adat_rx_demux, 8);
                                outuint(c_adat_rx_demux, adat_state[other_buff_idx].samples[0]);
                                outuint(c_adat_rx_demux, adat_state[other_buff_idx].samples[1]);
                                outuint(c_adat_rx_demux, adat_state[other_buff_idx].samples[2]);
                                outuint(c_adat_rx_demux, adat_state[other_buff_idx].samples[3]);
                                outuint(c_adat_rx_demux, adat_state[other_buff_idx].samples[4]);
                                outuint(c_adat_rx_demux, adat_state[other_buff_idx].samples[5]);
                                outuint(c_adat_rx_demux, adat_state[other_buff_idx].samples[6]);
                                outuint(c_adat_rx_demux, adat_state[other_buff_idx].samples[7]);
                                // outct(c_adat_rx_demux, XS1_CT_END);
                                // chkct(c_adat_rx_demux, XS1_CT_END);
                                adat_state[adat_state_idx].channel = 0;
                                adat_state_idx ^= 1;
                                smux_setting = new_smux_setting;
                                sample_period_count++;
                            }
                        break;

                        // SMUX-II 11223344
                        case SMUX_II:
                            if(adat_state[adat_state_idx].channel == 4){
                                outuint(c_adat_rx_demux, measured_adat_rate);
                                outuint(c_adat_rx_demux, 4);
                                if(adat_group_idx == 0){
                                    outuint(c_adat_rx_demux, adat_state[other_buff_idx].samples[0]);
                                    outuint(c_adat_rx_demux, adat_state[other_buff_idx].samples[2]);
                                    outuint(c_adat_rx_demux, adat_state[other_buff_idx].samples[4]);
                                    outuint(c_adat_rx_demux, adat_state[other_buff_idx].samples[6]);
                                } else {
                                    outuint(c_adat_rx_demux, adat_state[other_buff_idx].samples[1]);
                                    outuint(c_adat_rx_demux, adat_state[other_buff_idx].samples[3]);
                                    outuint(c_adat_rx_demux, adat_state[other_buff_idx].samples[5]);
                                    outuint(c_adat_rx_demux, adat_state[other_buff_idx].samples[7]);
                                }
                                // outct(c_adat_rx_demux, XS1_CT_END);
                                // chkct(c_adat_rx_demux, XS1_CT_END);
                                adat_state[adat_state_idx].channel = 0;
                                sample_period_count++;
                                adat_group_idx++;
                                if(adat_group_idx == 2){
                                    adat_group_idx = 0;
                                    adat_state_idx ^= 1;
                                    smux_setting = new_smux_setting;
                                }
                            }
                        break;

                        // SMUX-IV 11112222
                        case SMUX_IV:
                            if(adat_state[adat_state_idx].channel == 2){
                                outuint(c_adat_rx_demux, measured_adat_rate);
                                outuint(c_adat_rx_demux, 2);
                                unsigned offset = adat_group_idx * 2;
                                outuint(c_adat_rx_demux, adat_state[other_buff_idx].samples[offset + 0]);
                                outuint(c_adat_rx_demux, adat_state[other_buff_idx].samples[offset + 4]);
                                // outct(c_adat_rx_demux, XS1_CT_END);
                                // chkct(c_adat_rx_demux, XS1_CT_END);
                                adat_state[adat_state_idx].channel = 0;
                                sample_period_count++;
                                adat_group_idx++;
                                if(adat_group_idx == 4){
                                    adat_group_idx = 0;
                                    adat_state_idx ^= 1;
                                    smux_setting = new_smux_setting;
                                }
                            }
                        break; 
                    }
                }
            break;

            case c_smux_change_adat_rx :> word:
                if(word == IO_ADAT_RX){
                    unsigned new_smux_setting;
                    c_smux_change_adat_rx :> new_smux_setting;
                    printstr("adat rx smux: ");printintln(new_smux_setting);
                }
            break;

            case tmr when timerafter(rate_measurement_trigger) :> int _:
                // Measure ADAT
                uint32_t samples_per_second = sample_period_count * measurement_rate_hz;
                measured_adat_rate = get_normal_sample_rate(samples_per_second);
                sample_period_count = 0;
                rate_measurement_trigger += rate_measurement_period;
            break;
        } // select
    } // while
}



void adat_tx_setup_task(chanend c_adat_tx, buffered out port:32 p_adat_out){
    set_clock_src(mck_blk, p_mclk);
    set_clock_fall_delay(mck_blk, 7);   // XAI2 board, set to appropriate value for board.

    set_port_clock(p_adat_out, mck_blk);
    start_clock(mck_blk);
}


void adat_rx_task(chanend c_adat_rx) {
    while(1) {
        adatReceiver48000(p_adat_in, c_adat_rx);
        adatReceiver44100(p_adat_in, c_adat_rx);
        printstr("adatrx restart\n");
    }
}


void audio_hw_setup(void){
    // This is for the XU316MC board. Needs to be called from tile[0].
    printstrln("AudioHwInit");
    AudioHwInit();
    printstrln("AudioHwInit fin");
}

typedef struct button_state_t{
    uint8_t active_level;   // gpio level for active
    uint8_t counter;        // counts down
    uint8_t bit_shift;
}button_state_t;


void button_action(chanend c_sr_change_i2s, chanend c_smux_change_adat_rx, int idx){
    const unsigned sr_list[] = {44100, 48000, 88200, 96000, 176400, 192000};
    const unsigned smux_list[] = {SMUX_NONE, SMUX_II, SMUX_IV};

    if(idx == 2){
        static unsigned curr_sr_idx = 1; //48k default
        curr_sr_idx++;
        if(curr_sr_idx == sizeof(sr_list) / sizeof(sr_list[0])){
            curr_sr_idx = 0;
        }
        // Send to audiohub
        c_sr_change_i2s <: IO_I2S;
        c_sr_change_i2s <: sr_list[curr_sr_idx];
    }
    if(idx == 1){
        static unsigned curr_sm_idx = 0;
        curr_sm_idx++;
        if(curr_sm_idx == sizeof(smux_list) / sizeof(smux_list[0])){
            curr_sm_idx = 0;
        }
        // Send to ADAT Rx demux
        c_smux_change_adat_rx <: IO_ADAT_RX;
        c_smux_change_adat_rx <: smux_list[curr_sm_idx];
    }
    if(idx == 0){
        static unsigned curr_sm_idx = 0;
        curr_sm_idx++;
        if(curr_sm_idx == sizeof(smux_list) / sizeof(smux_list[0])){
            curr_sm_idx = 0;
        }
        c_sr_change_i2s <: IO_ADAT_TX;
        c_sr_change_i2s <: smux_list[curr_sm_idx];
    }
}

#define NUM_BUTTONS 3
void gpio(chanend c_sr_change_i2s, chanend c_smux_change_adat_rx, in port p_buttons, out port p_leds){
    const uint8_t counts_for_active = 20;
    const uint8_t active_level = 0;

    button_state_t button_state[NUM_BUTTONS] =  {{active_level, counts_for_active, 0},
                                                 {active_level, counts_for_active, 1},
                                                 {active_level, counts_for_active, 2}};     

    timer tmr;
    int32_t sample_time;
    tmr :> sample_time;

    while (1) {
        select {
            // when we the timer reaches the timeout to renter a stable period
            case tmr when timerafter(sample_time) :> void:
                uint32_t port_val;
                p_buttons :> port_val;
                for(int i = 0; i < NUM_BUTTONS; i++){
                    uint8_t mask = 1 << button_state[i].bit_shift;
                    if((port_val & mask) == (button_state[i].active_level << button_state[i].bit_shift)){
                        if(button_state[i].counter){
                            button_state[i].counter--;
                            if(button_state[i].counter == 0){
                                button_action(c_sr_change_i2s, c_smux_change_adat_rx, i);
                            }
                        }
                    } else {
                        button_state[i].counter = counts_for_active;
                    }
                }
            sample_time += XS1_TIMER_KHZ * 1;
            break;
        }
    }
}

extern void asrc_processor(chanend c_asrc);

int main(void) {
    chan c_adat_rx;
    chan c_adat_rx_demux;
    chan c_adat_tx;
    i2s_frame_callback_if i_i2s;
    chan c_sr_change_i2s;
    chan c_smux_change_adat_rx;
    interface i2c_master_if i2c[1];

    par {
        on tile[0]: {
            board_setup();
            par{
                adat_rx_task(c_adat_rx);
                adat_rx_demux(c_adat_rx, c_adat_rx_demux, c_smux_change_adat_rx);
                i2c_master(i2c, 1, p_scl, p_sda, 100);
                gpio(c_sr_change_i2s, c_smux_change_adat_rx, p_buttons, p_leds);

            }
        }
        on tile[1]: {
            unsafe{
                i_i2c_client = i2c[0];
            }
            delay_milliseconds(100); // Wait for board_setup() to complete
            audio_hw_setup();
            adat_tx_setup_task(c_adat_tx, p_adat_out);

            par {
                audio_hub(c_adat_tx, i_i2s, c_sr_change_i2s);
                adat_tx_port(c_adat_tx, p_adat_out);
                i2s_frame_master(i_i2s, p_dac, NUM_I2S_DAC_LINES, p_adc, NUM_I2S_ADC_LINES, I2S_DATA_BITS, p_bclk, p_lrclk, p_mclk, bclk);
                asrc_processor(c_adat_rx_demux);
            }
        }
    }
    return 0;
}
