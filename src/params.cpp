#include <Arduino.h>
#include <EEPROM.h>
#include "params.h"

// Magic values
constexpr auto EEPROM_CONFIG_START = 0x100U;
constexpr auto EEPROM_PARAM_STRIDE = 0x100U;
constexpr auto NUM_SLOTS = 10U;
constexpr auto EEPROM_MAGIC = (int32_t) 0xDDAADD00;
constexpr auto EEPROM_CURRENT_VERSION = (int32_t) 1;

static_assert(EEPROM_PARAM_STRIDE >= sizeof(params_t), "Param structure is too large!");
static_assert(EEPROM_CONFIG_START + (NUM_SLOTS * EEPROM_PARAM_STRIDE) <= SPI_FLASH_SEC_SIZE);   // or other eeprom size limit?

// Old parameter types
struct params_v0 {
  int32_t magic;  // are we good?
  int16_t gyro_offset[3]; // 94, -20, -20 for katana;  53,  -18, 30 for test board
  int16_t accel_offset[3];  // test board: -1250, -6433, 1345
};


// Global parameters object
params_t params = {
    .magic = EEPROM_MAGIC + EEPROM_CURRENT_VERSION,
    .gyro_offset = { INT16_MIN, INT16_MIN, INT16_MIN },    // "null" value meaning read MPU
    .accel_offset ={ INT16_MIN, INT16_MIN, INT16_MIN },    // "null" value meaning read MPU
    .max_brightness = 255,
    .base_brightness = 32,
    .palette = 0,
    .target_temperature = 8000,
    .blade_ki = 0.0,
    .blade_kd = 0.1,    
    .acc_sensitivity = 100,
    .gyro_sensitivity = 0.001,
    .fwd_color_scale = 100,
    .back_color_scale = -50,
    .surge_falloff = 0.9,
    .surge_sensitivity = 20,
    .cool_min = 0, .cool_max = 5,
};


// Methods
void init_params() {
    EEPROM.begin(EEPROM_CONFIG_START + (NUM_SLOTS * EEPROM_PARAM_STRIDE));
    params = load_params(0);
}

params_t load_params(unsigned slot) {
  if (slot >= NUM_SLOTS) return params_t { 0 };  
  params_t result;

  auto addr = EEPROM_CONFIG_START + slot * EEPROM_PARAM_STRIDE;
  // Check the magic number
  EEPROM.get(addr, result.magic);

  if (result.magic == EEPROM_MAGIC + EEPROM_CURRENT_VERSION) {
      // Just read 'em in
      EEPROM.get(addr, result);
  }
  // No dice, check old versions
  else if (result.magic == EEPROM_MAGIC + 0) {
      auto old_params = params_v0 {};
      EEPROM.get(addr, old_params);      
      // Copy the known values
      result = params;  // keep other values
      memcpy(&result.gyro_offset, &old_params.gyro_offset, sizeof(result.gyro_offset));
      memcpy(&result.accel_offset, &old_params.accel_offset, sizeof(result.accel_offset));
  }
  return result;
}

bool save_params(unsigned slot, const params_t& values) {
  if (slot >= NUM_SLOTS) return false;
  EEPROM.put(EEPROM_CONFIG_START + slot * EEPROM_PARAM_STRIDE, values);
  EEPROM.commit();
  return true;
}
