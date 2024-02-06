#ifndef _ADAT_H_
#define _ADAT_H_

#include <stdint.h>
#include <platform.h>

enum adat_smux_setting{
    SMUX_NONE = 0,
    SMUX_II,
    SMUX_IV
};

#define ADAT_MAX_SAMPLES                8
typedef struct adat_state_t{
    int32_t samples[ADAT_MAX_SAMPLES];
    int32_t rx_time_latest;
    int32_t rx_time_last;
    uint8_t user_bits;
    uint8_t channel;
}adat_state_t;



void adat_rx_demux(chanend c_adat_rx, chanend c_adat_rx_demux, chanend c_smux_change_adat_rx);

void adat_tx_setup_task(chanend c_adat_tx, clock mck_blk, in port p_mclk, buffered out port:32 p_adat_out);

void adat_rx_task(chanend c_adat_rx, buffered in port:32 p_adat_in);

#endif