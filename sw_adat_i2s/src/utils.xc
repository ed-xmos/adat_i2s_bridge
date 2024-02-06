#include <platform.h>
#include <xs1.h>
#include <stdint.h>
#include <print.h>

#include "app_config.h"
#include "adat.h"


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
