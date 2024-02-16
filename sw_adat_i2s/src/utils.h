#ifndef _UTILS_H_
#define _UTILS_H_

#include <platform.h>
#include <xs1.h>
#include <stdint.h>

#include "app_config.h"

enum audio_port_idx{
    IO_I2S = 0,
    IO_ADAT_RX,
    IO_ADAT_TX
};

#define RATE_LOWER(rate, ppm) ((uint32_t)((float)rate * (1.0 - (float)ppm / 1e6) + 0.5))
#define RATE_HIGHER(rate, ppm) ((uint32_t)((float)rate * (1.0 + (float)ppm / 1e6)+ 0.5))
#define CHECK_RATE(sps, rate, ppm) if (sps > RATE_LOWER(rate, ppm) && sps < RATE_HIGHER(rate, ppm)){return rate;}

inline uint32_t get_normal_sample_rate(uint32_t samples_per_second){
    const uint32_t ppm_tolerance = 10000;

    // Check in reverse order so high rates take fewer cycles
    CHECK_RATE(samples_per_second, 192000, ppm_tolerance);
    CHECK_RATE(samples_per_second, 176400, ppm_tolerance);
    CHECK_RATE(samples_per_second, 96000, ppm_tolerance);
    CHECK_RATE(samples_per_second, 88200, ppm_tolerance);
    CHECK_RATE(samples_per_second, 48000, ppm_tolerance);
    CHECK_RATE(samples_per_second, 44100, ppm_tolerance);

    return 0; // Invalid rate found
}

inline uint32_t sample_rate_from_ts_diff(int32_t t0, int32_t t1){
    int32_t ts_diff = t1 - t0;
    if(ts_diff == 0){
        return 0;
    }
    uint32_t approx_rate = XS1_TIMER_HZ / ts_diff;

    return get_normal_sample_rate(approx_rate);
}

inline uint32_t calc_sample_rate(int32_t *last_timestamp, int32_t latest_timestamp, uint32_t current_i2s_rate, uint32_t *i2s_sample_period_count){
    const uint32_t measurement_rate_hz = 10;
    const int32_t rate_measurement_period = XS1_TIMER_HZ / measurement_rate_hz;
    
    if(timeafter(latest_timestamp, *last_timestamp + rate_measurement_period)){
        *last_timestamp = latest_timestamp;
        uint32_t samples_per_second = *i2s_sample_period_count * measurement_rate_hz;
        *i2s_sample_period_count = 0;

        return get_normal_sample_rate(samples_per_second);
    }

    return current_i2s_rate;
}

void gpio(chanend c_sr_change_i2s, chanend c_smux_change_adat_rx, in port p_buttons, out port p_leds);

#endif