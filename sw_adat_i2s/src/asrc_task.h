#include <stdint.h>


int pull_samples(int32_t *samples, int32_t consume_timestamp);

#ifdef __XC__
void asrc_processor(chanend c_adat_rx_demux);
#else
#include <xcore/chanend.h>
void asrc_processor(chanend_t c_adat_rx_demux);
#endif