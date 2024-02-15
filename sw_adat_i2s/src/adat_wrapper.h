#ifndef _ADAT_H_
#define _ADAT_H_

#include <stdint.h>
#include <platform.h>
#include "app_config.h"
#include "asrc_task.h"
#ifndef __XC__
#include "xcore/channel.h"
#endif

enum adat_smux_setting{
    SMUX_NONE = 1,
    SMUX_II = 2,
    SMUX_IV = 4
};

typedef struct adat_state_t{
    int32_t samples[ADAT_MAX_SAMPLES];
    int32_t rx_time_last_sample;
    uint8_t user_bits;
    uint8_t sample_number;
}adat_state_t;

#ifndef __XC__
unsigned receive_adat_samples(chanend_t c_asrc_input, asrc_in_out_t *asrc_io, unsigned asrc_channel_count, unsigned *new_input_rate);
#else

void adat_rx_demux(chanend c_adat_rx, chanend c_adat_rx_demux, chanend c_smux_change_adat_rx);

void adat_tx_setup_task(chanend c_adat_tx, clock mck_blk, in port p_mclk, buffered out port:32 p_adat_out);

void adat_rx_task(chanend c_adat_rx, buffered in port:32 p_adat_in);

void adat_tx_task(chanend c_adat_tx, buffered out port:32 p_adat_out);

void init_adat_tx(chanend c_adat_tx, unsigned adat_tx_smux, int32_t *adat_tx_samples);

void deinit_adat_tx(chanend c_adat_tx);


#pragma unsafe arrays
static inline void send_adat_tx_samples(chanend c_adat_tx, const unsigned adat_tx_samples[], int smux, int handshake)
{
    static unsigned adatCounter = 0;
    unsigned adatSamples[8];


    // Do some re-arranging for SMUX..
    unsafe{
        // Note, when smux == 1 this loop just does a straight 1:1 copy
        int adatSampleIndex = adatCounter;
        for(int i = 0; i < (8 / smux); i++){
            adatSamples[adatSampleIndex] = adat_tx_samples[i];
            adatSampleIndex += smux;
        }
    }

    adatCounter++;

    if(adatCounter == smux) unsafe {
        // Wait for ADAT core to be done with buffer
        // Note, we are "running ahead" of the ADAT core
        
        inuint(c_adat_tx);

        // Send buffer pointer over to ADAT core
        volatile unsigned * unsafe samplePtr = (unsigned * unsafe) adatSamples;
        outuint(c_adat_tx, (unsigned) samplePtr);
        adatCounter = 0;
    }
}
#endif

#endif // _ADAT_H_
