// EEPROM-backed parameters structure
// Most of these are managed by Blynk.

#pragma once
#include <cstdint>

#pragma pack(push, 1)
struct params_t {
  int32_t magic;  // are we good? also a version indicator
  // V1 structure
  int16_t gyro_offset[3]; // 94, -20, -20 for katana;  53,  -18, 30 for test board
  int16_t accel_offset[3];  // test board: -1250, -6433, 1345

  uint8_t max_brightness;  // overall scale
  uint8_t base_brightness; // base level before effects
  uint16_t palette;        // really an enum  
  uint32_t target_temperature;
  float blade_ki, blade_kd;
  float acc_sensitivity;
  float gyro_sensitivity;
  int32_t fwd_color_scale;
  int32_t back_color_scale;
};
#pragma pack(pop)

// Global parameters structure
extern params_t params;

// Methods
void init_params();
void save_params();