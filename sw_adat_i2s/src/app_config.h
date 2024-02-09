
#ifndef _APP_CONFIG_H_
#define _APP_CONFIG_H_

#define ADAT_MAX_SAMPLES                                    8

#define NUM_I2S_DAC_LINES                                   3
#define NUM_I2S_ADC_LINES                                   1
#define I2S_DATA_BITS                                       32

enum audio_port_idx{
    IO_I2S = 0,
    IO_ADAT_RX,
    IO_ADAT_TX
};

#endif