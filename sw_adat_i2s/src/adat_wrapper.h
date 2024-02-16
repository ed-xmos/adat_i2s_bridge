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
void adat_tx_startup(chanend c_adat_tx, unsigned adat_tx_smux, int32_t *adat_tx_samples);

void adat_tx_shutdown(chanend c_adat_tx);


#pragma unsafe arrays
static inline void send_adat_tx_samples(chanend c_adat_tx, const unsigned adat_tx_samples[], int smux)
{
    static unsigned adat_counter = 0;
    unsigned adat_tx_muxed[8];

    // Do some re-arranging for SMUX..
    unsafe{
        // Note, when smux == 1 this loop just does a straight 1:1 copy
        int adat_sample_index = adat_counter;
        for(int i = 0; i < (8 / smux); i++){
            adat_tx_muxed[adat_sample_index] = adat_tx_samples[i];
            adat_sample_index += smux;
        }
    }

    adat_counter++;

    if(adat_counter == smux) unsafe {
        // Wait for ADAT core to be done with buffer
        // Note, we are "running ahead" of the ADAT core
        inuint(c_adat_tx);

        // Send buffer pointer over to ADAT core
        volatile unsigned * unsafe sample_ptr = (unsigned * unsafe) adat_tx_muxed;
        outuint(c_adat_tx, (unsigned) sample_ptr);
        adat_counter = 0;
    }
}
#endif // __XC__

#endif // _ADAT_H_
