#include <print.h>

#include "adat_rx.h"

#include "adat_wrapper.h"
#include "utils.h"
#include "app_config.h"
#ifndef ADAT_TX_USE_SHARED_BUFF
#error Designed for ADAT tx shared buffer mode ONLY
#endif
#include "adat_tx.h"

// Called from a different tile hence channel usage. This overrides the weak function in asrc_task.c
unsigned receive_asrc_input_samples(chanend c_adat_rx_demux, asrc_in_out_t &asrc_io, unsigned &asrc_channel_count, unsigned &new_input_rate){
    static unsigned asrc_in_counter = 0;
    unsigned input_write_idx = asrc_io.input_write_idx;

    // demuxed ADAT Rx
    int32_t adat_rx_samples[8] = {0};

    // Get ADAT samples from channel
    new_input_rate = inuint(c_adat_rx_demux);
    asrc_io.input_timestamp = inuint(c_adat_rx_demux);
    asrc_io.input_channel_count = inuint(c_adat_rx_demux);

    #pragma unsafe arrays
    for(unsigned ch = 0; ch < asrc_io.input_channel_count; ch++){
        adat_rx_samples[ch] = inuint(c_adat_rx_demux);
    }

    // Pack into array properly LRLRLRLR or 123412341234 etc.
    for(int i = 0; i < asrc_channel_count; i++){
        int idx = i + asrc_channel_count * asrc_in_counter;
        asrc_io.input_samples[input_write_idx][idx] = adat_rx_samples[i];
    }

    if(++asrc_in_counter == SRC_N_IN_SAMPLES){
        asrc_in_counter = 0;
    }

    return asrc_in_counter;
}


#pragma unsafe arrays // Remove array checks for speed
void adat_rx_demux(chanend c_adat_rx, chanend c_adat_rx_demux, chanend c_smux_change_adat_rx)
{

    unsigned word = 0;                          // Used for first word Rx in select
    adat_state_t adat_state[2] = {{{0}}};
    uint8_t adat_state_wr_idx = 0;              // Double buffer index used for writing rx'd samples
    uint32_t smux_setting = SMUX_NONE;
    uint32_t new_smux_setting = smux_setting;   // Used for clean changeover of SMUX

    // Sample rate measurement vars
    timer tmr;
    int32_t rate_measurement_trigger = 0;
    const uint32_t measurement_rate_hz = 10;
    const int32_t rate_measurement_period = XS1_TIMER_HZ / measurement_rate_hz;
    uint32_t measured_adat_rate = 0;
    uint32_t rate_sample_count = 0;

    tmr :> rate_measurement_trigger;
    rate_measurement_trigger += rate_measurement_period;

    while(1)
    {
        select
        {
            case inuint_byref(c_adat_rx, word):
                // grab timestamp first for minimum jitter
                int32_t time_stamp;
                tmr :> time_stamp;
                // The "other" buffer for reading from
                uint8_t adat_state_rd_idx = adat_state_wr_idx ^ 1;
                // Decode ADAT Rx output
                if(word & 0x01){
                    adat_state[adat_state_wr_idx].sample_number = 0;
                    adat_state[adat_state_wr_idx].user_bits = word >> 4;
                } else {
                    // Store sample in linear array. We expect 8 of these always.
                    adat_state[adat_state_wr_idx].samples[adat_state[adat_state_wr_idx].sample_number] = word << 4;
                    adat_state[adat_state_wr_idx].sample_number++;
                    adat_state[adat_state_wr_idx].rx_time_last_sample = time_stamp;

                    switch(smux_setting){
                        // No SMUX - 12345678
                        case SMUX_NONE:
                            if(adat_state[adat_state_wr_idx].sample_number >= 8){
                                outuint(c_adat_rx_demux, measured_adat_rate);
                                outuint(c_adat_rx_demux, adat_state[adat_state_rd_idx].rx_time_last_sample);
                                outuint(c_adat_rx_demux, 8);
                                outuint(c_adat_rx_demux, adat_state[adat_state_rd_idx].samples[0]);
                                outuint(c_adat_rx_demux, adat_state[adat_state_rd_idx].samples[1]);
                                outuint(c_adat_rx_demux, adat_state[adat_state_rd_idx].samples[2]);
                                outuint(c_adat_rx_demux, adat_state[adat_state_rd_idx].samples[3]);
                                outuint(c_adat_rx_demux, adat_state[adat_state_rd_idx].samples[4]);
                                outuint(c_adat_rx_demux, adat_state[adat_state_rd_idx].samples[5]);
                                outuint(c_adat_rx_demux, adat_state[adat_state_rd_idx].samples[6]);
                                outuint(c_adat_rx_demux, adat_state[adat_state_rd_idx].samples[7]);

                                adat_state_wr_idx = adat_state_rd_idx;
                                smux_setting = new_smux_setting;
                                rate_sample_count++;
                            }
                        break;

                        // SMUX-II 11223344
                        case SMUX_II:
                            if(adat_state[adat_state_wr_idx].sample_number == 4){
                                outuint(c_adat_rx_demux, measured_adat_rate);
                                outuint(c_adat_rx_demux, adat_state[adat_state_rd_idx].rx_time_last_sample);
                                outuint(c_adat_rx_demux, 4);
                                outuint(c_adat_rx_demux, adat_state[adat_state_rd_idx].samples[0]);
                                outuint(c_adat_rx_demux, adat_state[adat_state_rd_idx].samples[2]);
                                outuint(c_adat_rx_demux, adat_state[adat_state_rd_idx].samples[4]);
                                outuint(c_adat_rx_demux, adat_state[adat_state_rd_idx].samples[6]);
                                rate_sample_count++;
                            }
                            else if(adat_state[adat_state_wr_idx].sample_number >= 8){
                                outuint(c_adat_rx_demux, measured_adat_rate);
                                outuint(c_adat_rx_demux, adat_state[adat_state_rd_idx].rx_time_last_sample);
                                outuint(c_adat_rx_demux, 4);
                                outuint(c_adat_rx_demux, adat_state[adat_state_rd_idx].samples[1]);
                                outuint(c_adat_rx_demux, adat_state[adat_state_rd_idx].samples[3]);
                                outuint(c_adat_rx_demux, adat_state[adat_state_rd_idx].samples[5]);
                                outuint(c_adat_rx_demux, adat_state[adat_state_rd_idx].samples[7]);
            
                                adat_state_wr_idx = adat_state_rd_idx;
                                smux_setting = new_smux_setting;
                                rate_sample_count++;
                            }
                        break;

                        // SMUX-IV 11112222
                        case SMUX_IV:
                            if(adat_state[adat_state_wr_idx].sample_number == 2){
                                outuint(c_adat_rx_demux, measured_adat_rate);
                                outuint(c_adat_rx_demux, adat_state[adat_state_rd_idx].rx_time_last_sample);
                                outuint(c_adat_rx_demux, 2);
                                outuint(c_adat_rx_demux, adat_state[adat_state_rd_idx].samples[0]);
                                outuint(c_adat_rx_demux, adat_state[adat_state_rd_idx].samples[4]);
                                rate_sample_count++;
                            }
                            else if(adat_state[adat_state_wr_idx].sample_number == 4){
                                outuint(c_adat_rx_demux, measured_adat_rate);
                                outuint(c_adat_rx_demux, adat_state[adat_state_rd_idx].rx_time_last_sample);
                                outuint(c_adat_rx_demux, 2);
                                outuint(c_adat_rx_demux, adat_state[adat_state_rd_idx].samples[1]);
                                outuint(c_adat_rx_demux, adat_state[adat_state_rd_idx].samples[5]);
                                rate_sample_count++;
                            }
                            else if(adat_state[adat_state_wr_idx].sample_number == 6){
                                outuint(c_adat_rx_demux, measured_adat_rate);
                                outuint(c_adat_rx_demux, adat_state[adat_state_rd_idx].rx_time_last_sample);
                                outuint(c_adat_rx_demux, 2);
                                outuint(c_adat_rx_demux, adat_state[adat_state_rd_idx].samples[2]);
                                outuint(c_adat_rx_demux, adat_state[adat_state_rd_idx].samples[6]);
                                rate_sample_count++;
                            }
                            else if(adat_state[adat_state_wr_idx].sample_number >= 8){
                                outuint(c_adat_rx_demux, measured_adat_rate);
                                outuint(c_adat_rx_demux, adat_state[adat_state_rd_idx].rx_time_last_sample);
                                outuint(c_adat_rx_demux, 2);
                                outuint(c_adat_rx_demux, adat_state[adat_state_rd_idx].samples[3]);
                                outuint(c_adat_rx_demux, adat_state[adat_state_rd_idx].samples[7]);

                                adat_state_wr_idx = adat_state_rd_idx;
                                smux_setting = new_smux_setting;
                                rate_sample_count++;
                            }
                        break; 
                    }
                }
            break;

            case c_smux_change_adat_rx :> word:
                if(word == IO_ADAT_RX){
                    c_smux_change_adat_rx :> new_smux_setting;
                    measured_adat_rate = 0; // Invalidate sample rate as it is now wrong. It will be recalculated.
                    printstr("ADAT rx SMUX change: ");printintln(new_smux_setting);
                }
            break;

            case tmr when timerafter(rate_measurement_trigger) :> int _:
                // Measure ADAT
                uint32_t samples_per_second = rate_sample_count * measurement_rate_hz;
                measured_adat_rate = get_normal_sample_rate(samples_per_second);
                rate_sample_count = 0;
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
    delay_milliseconds(1000); //TODO remove me. Waiting for ASRC to consume..
    while(1) {
        adatReceiver48000(p_adat_in, c_adat_rx);
        printstr("adatrx restart\n");
        adatReceiver44100(p_adat_in, c_adat_rx);
        printstr("adatrx restart\n");
    }
}

void adat_tx_task(chanend c_adat_tx, buffered out port:32 p_adat_out){
    while(1){
        adat_tx_port(c_adat_tx, p_adat_out);
    }
}


void init_adat_tx(chanend c_adat_tx, unsigned adat_tx_smux, int32_t *adat_tx_samples){
    // Setup ADAT Tx
    // ADAT parameters ...
    //
    // adat_oversampling =  256 for MCLK = 12M288 or 11M2896
    //                   =  512 for MCLK = 24M576 or 22M5792
    //                   = 1024 for MCLK = 49M152 or 45M1584
    //
    // adatSmuxMode   = 1 for FS =  44K1 or  48K0
    //                = 2 for FS =  88K2 or  96K0
    //                = 4 for FS = 176K4 or 192K0

    outuint(c_adat_tx, ADAT_MULTIPLIER);
    outuint(c_adat_tx, adat_tx_smux);
    // Send initial frame so protocol for TransferAdatTxSamples is synched
    unsafe{
        volatile unsigned * unsafe sample_ptr = (volatile unsigned * unsafe)adat_tx_samples;
        printhexln((unsigned) sample_ptr);
        printintln(*sample_ptr);
        outuint(c_adat_tx, (unsigned) sample_ptr);
    }

    printstr("ADAT tx init smux: "); printintln(adat_tx_smux);
}

void deinit_adat_tx(chanend c_adat_tx){
    // Take outstanding handshake from ADAT core
    inuint(c_adat_tx);
    // Notify ADAT Tx thread of impending new freq...
    outct(c_adat_tx, XS1_CT_END);
}

