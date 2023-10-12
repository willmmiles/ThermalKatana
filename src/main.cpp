#include "config.h"

// FastLED configuration
#define FASTLED_ESP8266_NODEMCU_PIN_ORDER
#define FASTLED_ALL_PINS_HARDWARE_SPI
#define LED_PIN     3                    // which pin? -- pin 4 is used by Edgent
#define LED_TYPE    WS2812B              // preset for the WS2812B
#define COLOR_ORDER GRB                  // change if needed

// Blynk configuration
/* Fill-in information from Blynk Device Info here */
#define BLYNK_TEMPLATE_ID "TMPL2GI5SaWlg"
#define BLYNK_TEMPLATE_NAME "Thermal Katana"
//#define BLYNK_AUTH_TOKEN "dj4ewPU05Ra1Du9kHgQP_eOpjyPKVYk7"

#define BLYNK_FIRMWARE_VERSION        "0.1.0"

/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial

#define APP_DEBUG
#define USE_NODE_MCU_BOARD


#include <Arduino.h>
#include <FastLED.h>                     // needed for WS2812B LEDs
#include "BlynkEdgent.h"
#include "blade_simulation.h"
#include "dmp.h"

BlynkTimer sim_timer;

std::array<CRGB,NUM_LEDS> leds;
typedef Eigen::Array<uint16_t, NUM_LEDS, 1> brightness_array_t;

float acc_sensitivity = 200.;
float gyro_sensitivity = 0.001;
int fwd_color_scale = 100;
int back_color_scale = -50;


// Enum index to ap
const std::array<decltype(HeatColors_p)*, 7> palette_map = {
  &HeatColors_p,
  &PartyColors_p,
  &RainbowColors_p,
  &CloudColors_p,
  &LavaColors_p,
  &OceanColors_p,
  &ForestColors_p
};
decltype(HeatColors_p)* active_palette = palette_map[1];

// This function is called every time the Virtual Pin 0 state changes
BLYNK_WRITE(V0)
{
  // Set incoming value from pin V0 to a variable
  set_target_temperature(param.asInt());
}

BLYNK_WRITE(V1) { set_kd(param.asFloat()); }
BLYNK_WRITE(V2) { set_ki(param.asFloat()); }
 
BLYNK_WRITE(V3)
{
  int value = param.asInt();
  if ((value >= 0) && (value < (int)palette_map.size())) {
    active_palette = palette_map[value];
  }
}

BLYNK_WRITE(V5)
{
  int value = param.asInt();
  FastLED.setBrightness(value);
}

BLYNK_WRITE(V6) { acc_sensitivity = param.asFloat(); };
BLYNK_WRITE(V7) { gyro_sensitivity = param.asFloat(); };

BLYNK_WRITE(V10) { fwd_color_scale = param.asInt(); };
BLYNK_WRITE(V11) { back_color_scale = param.asInt(); };



BLYNK_WRITE(V50) { dmp_set_offset(static_cast<dmp_axis>(request.pin - 50), param.asInt()); };  
BLYNK_WRITE(V51) { dmp_set_offset(static_cast<dmp_axis>(request.pin - 50), param.asInt()); };  
BLYNK_WRITE(V52) { dmp_set_offset(static_cast<dmp_axis>(request.pin - 50), param.asInt()); };  
BLYNK_WRITE(V53) { dmp_set_offset(static_cast<dmp_axis>(request.pin - 50), param.asInt()); };  
BLYNK_WRITE(V54) { dmp_set_offset(static_cast<dmp_axis>(request.pin - 50), param.asInt()); };  
BLYNK_WRITE(V55) { dmp_set_offset(static_cast<dmp_axis>(request.pin - 50), param.asInt()); };  
BLYNK_WRITE(V56) { dmp_save_offset(); };

// This function is called every time the device is connected to the Blynk.Cloud
BLYNK_CONNECTED()
{
}

void sparkle(brightness_array_t& array) {
  constexpr int pixel_count[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 2 };
  static brightness_array_t history;
  history /= 5;
  
  auto n_pixels = random(sizeof(pixel_count) / sizeof(int));
  for (int i = 0; i < pixel_count[n_pixels]; i++) {
    auto j = random(NUM_LEDS);
    history[j] = 120;
  }
  array += history;
};

void wave(led_value_t& values) {
  static size_t start_index = 0;
  constexpr uint16_t wave_values[] = { 1, 1, 2, 3, 4, 5, 4, 3, 2, 1, 1 };
  for (auto i = 0U; i < sizeof(wave_values) / sizeof(uint16_t); ++i) {
    values[(start_index + i) % NUM_LEDS] += wave_values[i] * 1000;
  };
  start_index = (start_index + 1) % NUM_LEDS;
}

void energy_forward(led_value_t& values, brightness_array_t& brightness, float new_energy, uint16_t color_scale) {
  static brightness_array_t state;
  static size_t index = 0;
  if (new_energy < 0.) new_energy = 0;
  if (new_energy > 120) new_energy = 120;

  state[index] = (uint16_t) new_energy;
  for(auto led_index = 0; led_index < NUM_LEDS; ++led_index) {
    auto state_index = (led_index + index) % NUM_LEDS;
    brightness[led_index] += state[state_index];
    values[led_index] += state[state_index] * color_scale;
  }
  if (index) { --index; } else { index = NUM_LEDS - 1; };
};

void energy_backward(led_value_t& values, brightness_array_t& brightness, float new_energy, uint16_t color_scale) {
  static brightness_array_t state;
  static size_t index = NUM_LEDS - 1;
  if (new_energy < 0.) new_energy = 0;
  if (new_energy > 120) new_energy = 120;

  state[index] = (uint16_t) new_energy;
  for(auto led_index = 0; led_index < NUM_LEDS; ++led_index) {
    auto state_index = (led_index + index) % NUM_LEDS;
    brightness[led_index] += state[state_index];
    values[led_index] += state[state_index] * color_scale;
  }
  if (index == NUM_LEDS) { index = 0; } else { ++index; };
};

void surge(brightness_array_t& brightness, float amount) {
  static brightness_array_t state;
  if (amount < 0.) amount = 0;
  if (amount > 120) amount = 120;

  state = (state/2) + static_cast<uint16_t>(amount);
  brightness += state;
};

// This function sends Arduino's uptime every second to Virtual Pin 2.
void sim_timer_event()
{
    auto accel_values = read_dmp();
    auto led_values = simulate_temperature();
    auto led_brightness = brightness_array_t { brightness_array_t::Constant(32) };

    Serial.printf("[%d] %f, %f, %f\n", millis(), accel_values[0], accel_values[1], accel_values[2]);

    // Apply effects
    sparkle(led_brightness);
    //wave(led_values);

    //energy_forward(led_values, led_brightness, accel_values[0] / acc_sensitivity, fwd_color_scale);
    //energy_forward(led_values, led_brightness, fabs(accel_values[2]) / gyro_sensitivity, fwd_color_scale);
    surge(led_brightness, accel_values[1] / acc_sensitivity);

    for(auto i = 0U; i < NUM_LEDS; ++i) {
      //leds[i] = HeatColor(led_values[i]);
      leds[i] = ColorFromPalette(*active_palette, (led_values[i] / 40) % 256, min(led_brightness[i], (uint16_t) 255));
    }
    FastLED.show();
}


void setup() {
  Serial.begin(115200);
  Serial.printf("Starting up!\n");

  // Show an indicator LED while we start up
  leds.fill({0,0,0});
  leds[0].r = 128;
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds.data(), leds.size()).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness(20);
  FastLED.show();

  sim_timer.setInterval(20L, sim_timer_event);    
  BlynkEdgent.begin();

  // embiggen the eeprom range for dmp calibration
  EEPROM.begin(0x200);
  init_dmp();
};


void app_loop()
{
  edgentTimer.run();
  edgentConsole.run();
  sim_timer.run();  
}

void loop() {
  BlynkEdgent.run();
}
