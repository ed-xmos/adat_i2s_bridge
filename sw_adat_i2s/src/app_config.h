
#ifndef _APP_CONFIG_H_
#define _APP_CONFIG_H_

#define ADAT_MAX_SAMPLES                                    8

#define NUM_I2S_DAC_LINES                                   4
#define NUM_I2S_ADC_LINES                                   4
#define I2S_DATA_BITS                                       32

// For I2S master only (currently the CODEC is setup to be this). Can be removed when I2S slave
#define MCLK_441                                            22579200
#define MCLK_48                                             24576000
#define DEFAULT_FREQ                                        44100

#define ADAT_MULTIPLIER                                     512 // For 22/24M master clock

#define FORMAT_CHANGE_MUTE_MS                               250

#endif