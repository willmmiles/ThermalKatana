#include <Arduino.h>
#include <EEPROM.h>
#include "params.h"

// Magic values
constexpr auto EEPROM_CONFIG_START = 0x100;
constexpr auto EEPROM_MAGIC = (int32_t) 0xDDAADD00;
constexpr auto EEPROM_CURRENT_VERSION = (int32_t) 1;

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
};

// Methods
void init_params() {
    // Check the magic number
    decltype(params.magic) magic;
    EEPROM.get(EEPROM_CONFIG_START, magic);

    if (magic == EEPROM_MAGIC + EEPROM_CURRENT_VERSION) {
        // Just read 'em in
        EEPROM.get(EEPROM_CONFIG_START, params);
    }
    // No dice, check old versions
    else if (magic == EEPROM_MAGIC + 0) {
        auto old_params = params_v0 {};
        EEPROM.get(EEPROM_CONFIG_START, old_params);
        // Copy the known values
        memcpy(&params.gyro_offset, &old_params.gyro_offset, sizeof(params.gyro_offset));
        memcpy(&params.accel_offset, &old_params.accel_offset, sizeof(params.accel_offset));
    }
    // Everything else is defaulted        
    // so all done!
}

void save_params() {
  EEPROM.put(EEPROM_CONFIG_START, params);
  EEPROM.commit();
}
