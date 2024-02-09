#include <print.h>

#include "adat_rx.h"

#include "adat_wrapper.h"
#include "utils.h"
#include "app_config.h"


void receive_adat_samples(chanend c_adat_rx_demux, asrc_in_out_t &asrc_io, unsigned asrc_channel_count, unsigned asrc_in_counter, unsigned &new_input_rate){
    // demuxed ADAT Rx
    int32_t adat_rx_samples[8] = {0};

    // Get ADAT samples
    timer tmr;
    new_input_rate = inuint(c_adat_rx_demux);
    tmr :> asrc_io.input_timestamp;
    unsigned adat_rx_channels = inuint(c_adat_rx_demux);
    for(unsigned ch = 0; ch < adat_rx_channels; ch++){
        adat_rx_samples[ch] = inuint(c_adat_rx_demux);
    }

    // Pack into array properly LRLRLRLR or 123412341234 etc.
    for(int i = 0; i < asrc_channel_count; i++){
        int idx = i + asrc_channel_count * asrc_in_counter;
        asrc_io.input_samples[idx] = adat_rx_samples[i];
    }
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



void adat_tx_setup_task(chanend c_adat_tx, clock mck_blk, in port p_mclk, buffered out port:32 p_adat_out){
    set_clock_src(mck_blk, p_mclk);
    set_clock_fall_delay(mck_blk, 7);   // XAI2 board, set to appropriate value for board.

    set_port_clock(p_adat_out, mck_blk);
    start_clock(mck_blk);
}


void adat_rx_task(chanend c_adat_rx, buffered in port:32 p_adat_in) {
    while(1) {
        adatReceiver48000(p_adat_in, c_adat_rx);
        printstr("adatrx restart\n");
        adatReceiver44100(p_adat_in, c_adat_rx);
        printstr("adatrx restart\n");
    }
}
