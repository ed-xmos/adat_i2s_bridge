#include <platform.h>
#include <xs1.h>
#include <stdint.h>
#include <string.h>
#include <print.h>
#include "xassert.h"

#include "app_config.h"

#include "i2s.h"
#include "i2c.h"

#include "adat_wrapper.h"
extern "C"{
    #include "asrc_task.h"
}
#include "utils.h"



extern void board_setup(void);

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
unsafe{
    volatile uint32_t * unsafe new_output_rate_ptr = &new_output_rate;
}

[[distributable]]
void audio_hub( chanend c_adat_tx,
                server i2s_frame_callback_if i_i2s) {
    // Approx rate calculation based on sample counting
    timer tmr;
    int32_t rate_measurement_trigger = 0;
    const uint32_t measurement_rate_hz = 10;
    const int32_t rate_measurement_period = XS1_TIMER_HZ / measurement_rate_hz;
    tmr :> rate_measurement_trigger;
    rate_measurement_trigger += rate_measurement_period;

    // I2S sample rate measurement state
    uint32_t i2s_sample_period_count = 0;
    uint8_t measured_i2s_sample_rate_change = 1;    // Force new SR as measured
    uint32_t current_i2s_rate = 0;

    // General control
    uint32_t mute = 0;
    int32_t latest_timestamp = 0; 
    int32_t last_timestamp = 0;

    // ADAT Tx
    int adat_tx_smux = SMUX_NONE;
    int32_t adat_tx_samples[ADAT_MAX_SAMPLES] = {0};

    // adat_tx_startup(c_adat_tx, i2s_set_sample_rate, adat_tx_samples);

    while(1) {
        select{
            case i_i2s.init(i2s_config_t &?i2s_config, tdm_config_t &?tdm_config):
                printstr("i2s init: "); printintln(current_i2s_rate);

                // adat_tx_shutdown(c_adat_tx);
                
                mute = (current_i2s_rate * FORMAT_CHANGE_MUTE_MS) / 1000;
                i2s_config.mode = I2S_MODE_I2S;

                reset_asrc_fifo();

                // adat_tx_smux = adat_tx_startup(c_adat_tx, i2s_set_sample_rate, adat_tx_samples);
            break;

            case i_i2s.restart_check() -> i2s_restart_t restart:
                // This if the first callback so the least jitter measurement of timestamp should be here
                tmr :> latest_timestamp;

                // Inform the I2S slave whether it should restart or exit
                i2s_sample_period_count++;
                if(measured_i2s_sample_rate_change){
                    measured_i2s_sample_rate_change = 0;
                    restart = I2S_RESTART;
                    break;
                }
                restart = I2S_NO_RESTART;
            break;

            case i_i2s.receive(size_t num_in, int32_t samples[num_in]):
                if(mute == 0){
                    memcpy(adat_tx_samples, samples, num_in * sizeof(int32_t));
                    // send_adat_tx_samples(c_adat_tx, (unsigned *)adat_tx_samples, adat_tx_smux);
                }
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
                }

                uint32_t measured_i2s_rate = calc_sample_rate(&last_timestamp, latest_timestamp, current_i2s_rate, &i2s_sample_period_count);
                if((measured_i2s_rate != 0) && (current_i2s_rate != measured_i2s_rate)){
                    measured_i2s_sample_rate_change = 1;
                    current_i2s_rate = measured_i2s_rate;
                    unsafe{*new_output_rate_ptr = current_i2s_rate;}
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
    chan c_smux_change_adat_rx;
    interface i2c_master_if i2c[1];

    par {
        on tile[0]: {
            board_setup();
            par{
                adat_rx_task(c_adat_rx, p_adat_in);
                adat_rx_demux(c_adat_rx, c_adat_rx_demux, c_smux_change_adat_rx);
                i2c_master(i2c, 1, p_scl, p_sda, 100);
                gpio(c_smux_change_adat_rx, p_buttons, p_leds, i2c[0]);
            }
        }
        on tile[1]: {
            adat_tx_hw_setup(c_adat_tx, mck_blk, p_mclk, p_adat_out);

            par {
                [[distribute]]
                audio_hub(c_adat_tx, i_i2s);
                asrc_processor(c_adat_rx_demux);
                i2s_frame_slave(i_i2s, p_dac, NUM_I2S_DAC_LINES, p_adc, NUM_I2S_ADC_LINES, I2S_DATA_BITS, p_bclk, p_lrclk, bclk);
                adat_tx_task(c_adat_tx, p_adat_out);
            }
        }
    }
    return 0;
}
