#ifndef _UTILS_H_
#define _UTILS_H_

#include <platform.h>
#include <xs1.h>
#include <stdint.h>

#include "app_config.h"

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


void gpio(chanend c_sr_change_i2s, chanend c_smux_change_adat_rx, in port p_buttons, out port p_leds);

#endif