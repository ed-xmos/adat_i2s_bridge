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

#endif

#endif // _ADAT_H_
