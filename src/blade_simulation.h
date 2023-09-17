// Blade simulation external API
#include <cstdint>
#include "config.h"
#include "eigen.h"

#pragma once

// LED values are returned between 0 and 255
// These values can be converted to "temperature" values via FastLED::HeatColor
typedef Eigen::Array<uint8_t,NUM_LEDS,1> led_value_t;
led_value_t simulate_temperature();
