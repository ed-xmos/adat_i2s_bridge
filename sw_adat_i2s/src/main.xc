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

#include "adat_wrapper.h"
extern "C"{
    #include "asrc_task.h"
}
#include "utils.h"
#include "app_config.h"


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
on tile[1]: in port p_mclk =                                PORT_MCLK_IN;
on tile[1]: buffered in port:32 p_lrclk =                   PORT_I2S_LRCLK;
on tile[1]: in port p_bclk =                                PORT_I2S_BCLK;
on tile[1]: buffered out port:32 p_dac[NUM_I2S_DAC_LINES] = {PORT_I2S_DAC0, PORT_I2S_DAC1, PORT_I2S_DAC2, PORT_I2S_DAC3};
on tile[1]: buffered in port:32 p_adc[NUM_I2S_ADC_LINES] =  {PORT_I2S_ADC0 ,PORT_I2S_ADC1, PORT_I2S_ADC2, PORT_I2S_ADC3};
on tile[1]: clock bclk =                                    XS1_CLKBLK_1;


// Global to allow asrc_task to read it
uint32_t new_output_rate = 0;                  // Set to invalid initially

#if POLL_CONTROL_IN_SEND
[[distributable]]
#endif
void audio_hub( chanend c_adat_tx,
                server i2s_frame_callback_if i_i2s,
                chanend c_sr_change) {
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
    uint32_t current_i2s_rate = 0;

    uint32_t mute = 0;
    int32_t latest_timestamp = 0; 
    int32_t last_timestamp = 0;

    while(1) {
        select{
            case i_i2s.init(i2s_config_t &?i2s_config, tdm_config_t &?tdm_config):
                i2s_config.mclk_bclk_ratio = (master_clock_frequency / (i2s_set_sample_rate*2*I2S_DATA_BITS));
                printstr("i2s init: "); printintln(i2s_set_sample_rate);
                mute = i2s_set_sample_rate >> 2; // 250ms
                AudioHwConfig(i2s_set_sample_rate, master_clock_frequency, 0, 24, 24);
                i2s_config.mode = I2S_MODE_I2S;
                reset_fifo();
            break;

            case i_i2s.restart_check() -> i2s_restart_t restart:
                // This if the first callback so the least jitter measurement of timestamp should be here
                tmr :> latest_timestamp;

                // Inform the I2S slave whether it should restart or exit
                i2s_sample_period_count++;
                if(measured_i2s_sample_rate_change){
                    printstr("measured_i2s_sample_rate_change: "); printintln(current_i2s_rate);
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
            break;

            case i_i2s.receive(size_t num_in, int32_t samples[num_in]):
                // Handle a received sample
            break;

            case i_i2s.send(size_t num_out, int32_t samples[num_out]):
                for(int ch = 0; ch < num_out; ch++){
                    samples[ch] = 0;
                }

                int32_t asrc_out[8]; // TODO make max ASRC channels
                int asrc_channel_count = pull_samples(asrc_out, latest_timestamp);
                if(mute > 0){
                    for(int ch = 0; ch < num_out; ch++){
                        samples[ch] = 0;
                    }
                    mute--;
                }
                else {
                    for(int ch = 0; ch < num_out; ch++){
                        samples[ch] = asrc_out[ch];
                    }
                    samples[4] = asrc_out[0]; // TODO remove me. Just here for a signal copy.
                }

                uint32_t measured_i2s_rate = calc_sample_rate(&last_timestamp, latest_timestamp, current_i2s_rate, &i2s_sample_period_count);
                if((measured_i2s_rate != 0) && (current_i2s_rate != measured_i2s_rate)){
                    measured_i2s_sample_rate_change = 1;
                    current_i2s_rate = measured_i2s_rate;
                    new_output_rate = current_i2s_rate;
                }
                // Poll SR change channel
                select{
                    case c_sr_change :> unsigned id:
                        if(id == IO_I2S){
                            unsigned new_sr;
                            c_sr_change :> new_sr;
                            master_clock_frequency = (new_sr % 48000 == 0) ? MCLK_48 : MCLK_441;

                            i2s_set_sample_rate = new_sr;
                            i2s_master_sample_rate_change = 1;
                        }
                        if(id == IO_ADAT_TX){
                            unsigned new_smux;
                            c_sr_change :> new_smux;
                            printstr("adat tx smux: ");printintln(new_smux);
                        }
                    break;
                    // Fallthrough
                    default:
                        // Do nothing. No format change.
                    break;
                }
            break;
        } // select
    }// while(1)
}

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
                adat_rx_task(c_adat_rx, p_adat_in);
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
            AudioHwInit();
            adat_tx_setup_task(c_adat_tx, mck_blk, p_mclk, p_adat_out);

            par {
                [[distribute]]
                audio_hub(c_adat_tx, i_i2s, c_sr_change_i2s);
                i2s_frame_slave(i_i2s, p_dac, NUM_I2S_DAC_LINES, p_adc, NUM_I2S_ADC_LINES, I2S_DATA_BITS, p_bclk, p_lrclk, bclk);
                adat_tx_port(c_adat_tx, p_adat_out);
                asrc_processor(c_adat_rx_demux);
            }
        }
    }
    return 0;
}
