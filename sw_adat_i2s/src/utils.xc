#include <platform.h>
#include <xs1.h>
#include <stdint.h>
#include <print.h>

#include "app_config.h"
#include "adat_wrapper.h"
#include "utils.h"


extern void AudioHwInit(client interface i2c_master_if i_i2c);
extern void AudioHwConfig(client interface i2c_master_if i_i2c, unsigned samFreq, unsigned mClk, unsigned dsdMode, unsigned sampRes_DAC, unsigned sampRes_ADC);

typedef struct button_state_t{
    uint8_t active_level;   // gpio level for active
    uint8_t counter;        // counts down
    uint8_t bit_shift;
}button_state_t;

const unsigned sr_list[] = {44100, 48000, 88200, 96000, 176400, 192000};
const unsigned smux_list[] = {SMUX_NONE, SMUX_II, SMUX_IV};

{unsigned, int} button_action(client interface i2c_master_if i_i2c, chanend c_smux_change_adat_rx, int idx){
    static unsigned curr_sr_idx = 1; //48k default
    static unsigned curr_rx_sm_idx = 0;
    static unsigned curr_tx_sm_idx = 0;

    if(idx == 2){
        curr_sr_idx++;
        if(curr_sr_idx == sizeof(sr_list) / sizeof(sr_list[0])){
            curr_sr_idx = 0;
        }
        // Set new sample rate on CODEC (I2S master)
        unsigned master_clock_frequency = get_master_clock_from_samp_rate(sr_list[curr_sr_idx]);
        AudioHwConfig(i_i2c, sr_list[curr_sr_idx], master_clock_frequency, 0, 24, 24);
    }
    if(idx == 1){
        curr_rx_sm_idx++;
        if(curr_rx_sm_idx == sizeof(smux_list) / sizeof(smux_list[0])){
            curr_rx_sm_idx = 0;
        }
        // Send to ADAT Rx demux
        c_smux_change_adat_rx <: IO_ADAT_RX;
        c_smux_change_adat_rx <: smux_list[curr_rx_sm_idx];
    }
    if(idx == 0){
        curr_tx_sm_idx++;
        if(curr_tx_sm_idx == sizeof(smux_list) / sizeof(smux_list[0])){
            curr_tx_sm_idx = 0;
        }
    }

    return {curr_sr_idx, curr_rx_sm_idx};
}


#define NUM_BUTTONS 3
void gpio(chanend c_smux_change_adat_rx, in port p_buttons, out port p_leds, client interface i2c_master_if i_i2c){

    // Initial audio HW setup
    AudioHwInit(i_i2c);
    AudioHwConfig(i_i2c, DEFAULT_FREQ, get_master_clock_from_samp_rate(DEFAULT_FREQ), 0, 24, 24);

    // Buttons
    const uint8_t counts_for_active = 20;
    const uint8_t active_level = 0;
    button_state_t button_state[NUM_BUTTONS] =  {{active_level, counts_for_active, 0},
                                                 {active_level, counts_for_active, 1},
                                                 {active_level, counts_for_active, 2}};     
    timer tmr_buttons;
    int32_t sample_time;
    tmr_buttons :> sample_time;


    // LEDs
    const unsigned led_sr = 0x1;
    const unsigned led_sm = 0x8;

    timer tmr_leds;
    int32_t led_transition_time;
    tmr_leds :> led_transition_time;
    unsigned led_val = 0; // all off
    unsigned led_sequence_idx = 0;

    // State
    unsigned sr_i2s_idx = 0; // This will be overwritten by next function
    unsigned adat_rx_smux_idx = 0; // no SMUX

    // Reverse lookup DEFAULT_FREQ
    for(int i = 0; i < sizeof(sr_list) / sizeof(sr_list[0]); i++){
        if(sr_list[i] == DEFAULT_FREQ) sr_i2s_idx = i;
    }

    while (1) {
        select {
            // when we the timer reaches the timeout to renter a stable period
            case tmr_buttons when timerafter(sample_time) :> void:
                uint32_t port_val;
                p_buttons :> port_val;
                for(int i = 0; i < NUM_BUTTONS; i++){
                    uint8_t mask = 1 << button_state[i].bit_shift;
                    if((port_val & mask) == (button_state[i].active_level << button_state[i].bit_shift)){
                        if(button_state[i].counter){
                            button_state[i].counter--;
                            if(button_state[i].counter == 0){
                                {sr_i2s_idx, adat_rx_smux_idx} = button_action(i_i2c, c_smux_change_adat_rx, i);
                            }
                        }
                    } else {
                        button_state[i].counter = counts_for_active;
                    }
                }
            sample_time += XS1_TIMER_KHZ * 1;
            break;

            case tmr_leds when timerafter(led_transition_time) :> void:
                led_transition_time += XS1_TIMER_KHZ * 150;

                if(led_sequence_idx < 2 * (sr_i2s_idx+1)){
                    if(led_sequence_idx % 2 == 0) led_val |= led_sr;
                    else led_val &= ~led_sr;
                }

                if(led_sequence_idx < 2 * (adat_rx_smux_idx+1)){
                    if(led_sequence_idx % 2 == 0) led_val |= led_sm;
                    else led_val &= ~led_sm;
                }
                led_sequence_idx++;
                if (led_sequence_idx > 2 * 7){
                    led_sequence_idx = 0;
                }

                p_leds <: led_val;
            break;
        }
    }
}
