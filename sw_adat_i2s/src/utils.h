#ifndef _UTILS_H_
#define _UTILS_H_

#include <platform.h>
#include <xs1.h>
#include <stdint.h>

#include "app_config.h"
#include "i2c.h"

enum audio_port_idx{
    IO_I2S = 0,
    IO_ADAT_RX,
    IO_ADAT_TX
};

// These are statically calculated by the compiler for speed
#define RATE_LOWER(rate, ppm) ((uint32_t)((float)rate * (1.0 - (float)ppm / 1e6) + 0.5))
#define RATE_UPPER(rate, ppm) ((uint32_t)((float)rate * (1.0 + (float)ppm / 1e6) + 0.5))
#define CHECK_RATE(sps, rate, ppm) if (sps > RATE_LOWER(rate, ppm) && sps < RATE_UPPER(rate, ppm)){return rate;}

inline uint32_t get_normal_sample_rate(uint32_t samples_per_second){
    const uint32_t ppm_tolerance = 20000;

    // Check in reverse order so high rates take fewer cycles
    CHECK_RATE(samples_per_second, 192000, ppm_tolerance);
    CHECK_RATE(samples_per_second, 176400, ppm_tolerance);
    CHECK_RATE(samples_per_second, 96000, ppm_tolerance);
    CHECK_RATE(samples_per_second, 88200, ppm_tolerance);
    CHECK_RATE(samples_per_second, 48000, ppm_tolerance);
    CHECK_RATE(samples_per_second, 44100, ppm_tolerance);

    return 0; // Invalid rate found
}
#include <stdio.h>
inline uint32_t get_master_clock_from_samp_rate(uint32_t sample_rate){
    printf("%lu %lu\n", RATE_LOWER(44100, 10000), RATE_UPPER(44100, 10000));
    printf("%lu %lu\n", RATE_LOWER(48000, 10000), RATE_UPPER(48000, 10000));
    printf("%lu %lu\n", RATE_LOWER(88200, 10000), RATE_UPPER(88200, 10000));
    printf("%lu %lu\n", RATE_LOWER(96000, 10000), RATE_UPPER(96000, 10000));
    printf("%lu %lu\n", RATE_LOWER(176400, 10000), RATE_UPPER(176400, 10000));
    printf("%lu %lu\n", RATE_LOWER(192000, 10000), RATE_UPPER(192000, 10000));
    return (sample_rate % 48000 == 0) ? MCLK_48 : MCLK_441;
}

inline uint32_t sample_rate_from_ts_diff(int32_t &last_timestamp, int32_t latest_timestamp, uint8_t &i2s_rate_measurement_counter){
    int32_t ts_diff = latest_timestamp - last_timestamp;
    last_timestamp = latest_timestamp;

    if(i2s_rate_measurement_counter > 0){
        i2s_rate_measurement_counter--;
    }

    // Avoid div by 0 
    if(ts_diff == 0){
        return 0;
    }

    uint32_t approx_rate = XS1_TIMER_HZ / ts_diff;
    uint32_t nominal_rate = get_normal_sample_rate(approx_rate);

    return get_normal_sample_rate(nominal_rate);
}

inline uint32_t calc_sample_rate_over_period(int32_t *last_timestamp, int32_t latest_timestamp, uint32_t current_i2s_rate, uint32_t *i2s_sample_period_count){
    const int32_t rate_measurement_period = XS1_TIMER_HZ / I2S_RATE_MEASUREMENT_HZ;
    
    if(timeafter(latest_timestamp, *last_timestamp + rate_measurement_period)){
        *last_timestamp = latest_timestamp;
        uint32_t samples_per_second = *i2s_sample_period_count * I2S_RATE_MEASUREMENT_HZ;
        *i2s_sample_period_count = 0;

        return get_normal_sample_rate(samples_per_second);
    }

    return current_i2s_rate;
}

void gpio(chanend c_smux_change_adat_rx, in port p_buttons, out port p_leds, client interface i2c_master_if i_i2c);

#endif