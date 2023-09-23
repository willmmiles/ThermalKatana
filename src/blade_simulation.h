// Blade simulation external API
#include <cstdint>
#include "config.h"
#include "eigen.h"

#pragma once

void set_target_temperature(int degrees_k);
void set_kd(float kd);
void set_ki(float ki);

// LED values are returned between 0 and 16384
// These values can be converted to "temperature" values via FastLED::HeatColor
typedef Eigen::Array<uint16_t,NUM_LEDS,1> led_value_t;
led_value_t simulate_temperature();
