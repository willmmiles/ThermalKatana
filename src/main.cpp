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
#define BLYNK_SSL_TX_BUF_SIZE 1024
#define BLYNK_MSG_LIMIT 0
#define BLYNK_FIRMWARE_VERSION        "0.9.0"

/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial

#define APP_DEBUG
#define USE_NODE_MCU_BOARD

#include <eigen.h>  // must be before Arduino.h
#include <Arduino.h>
#include <FastLED.h>                     // needed for WS2812B LEDs
#include "BlynkEdgent.h"
#include "blade_simulation.h"
#include "dmp.h"
#include "effects.h"
#include "params.h"

BlynkTimer timer_core;

std::array<CRGB,NUM_LEDS> leds;

struct palette_info {
  decltype(HeatColors_p)& palette;
  TBlendType blend_type;
};


// Enum index to ap
const std::array<palette_info, 7> palette_map = {{
  {HeatColors_p, LINEARBLEND_NOWRAP},
  {PartyColors_p, LINEARBLEND},
  {RainbowColors_p, LINEARBLEND},
  {CloudColors_p, LINEARBLEND_NOWRAP},
  {LavaColors_p, LINEARBLEND_NOWRAP},
  {OceanColors_p, LINEARBLEND_NOWRAP},
  {ForestColors_p, LINEARBLEND_NOWRAP}
}};

void delayed_eeprom_write() {
  constexpr auto EEPROM_DELAY = 10 * 1000;  // 10s
  static auto timer_handle =  BlynkTimer::Handle {};
  if (timer_handle.isValid()) {
    timer_handle.restartTimer();
  } else {
    timer_handle = timer_core.setTimeout(EEPROM_DELAY, [&](){
      Serial.printf("[%ld] Saving state\n", millis());
      save_params(0);
      // invalidate handle
      timer_handle = BlynkTimer::Handle {}; 
    });
  }
}


// This function is called every time the Virtual Pin 0 state changes
BLYNK_WRITE(V0)
{
  // Set incoming value from pin V0 to a variable
  params.target_temperature = param.asInt();
  set_target_temperature(params.target_temperature);
  delayed_eeprom_write();
}

BLYNK_WRITE(V1) { params.blade_kd = param.asFloat(); set_kd(params.blade_kd); delayed_eeprom_write(); }
BLYNK_WRITE(V2) { params.blade_ki = param.asFloat(); set_ki(params.blade_ki); delayed_eeprom_write(); }
 
BLYNK_WRITE(V3)
{
  int value = param.asInt();
  if ((value >= 0) && (value < (int)palette_map.size())) {
    params.palette = value;
    delayed_eeprom_write();
  }
}

BLYNK_WRITE(V4) { params.base_brightness = param.asInt(); delayed_eeprom_write(); };

BLYNK_WRITE(V5)
{
  params.max_brightness = param.asInt();
  delayed_eeprom_write();
}



BLYNK_WRITE(V6) { params.acc_sensitivity = param.asFloat();  delayed_eeprom_write();};
BLYNK_WRITE(V7) {  params.gyro_sensitivity = param.asFloat(); delayed_eeprom_write(); };
BLYNK_WRITE(V8) {  params.surge_sensitivity  = param.asFloat();  delayed_eeprom_write();};

BLYNK_WRITE(V10) {  params.fwd_color_scale = param.asInt(); delayed_eeprom_write(); };
BLYNK_WRITE(V11) {  params.back_color_scale = param.asInt(); delayed_eeprom_write(); };
BLYNK_WRITE(V12) {  params.surge_falloff  = param.asFloat(); delayed_eeprom_write(); };

BLYNK_WRITE(V13) {  params.cool_min  = param.asFloat(); set_cool_min(params.cool_min); delayed_eeprom_write(); };
BLYNK_WRITE(V14) {  params.cool_max  = param.asFloat(); set_cool_max(params.cool_max); delayed_eeprom_write(); };


static void apply_params() {
  // Apply the params we just loaded
  set_target_temperature(params.target_temperature);
  set_kd(params.blade_kd);
  set_ki(params.blade_ki);
  set_cool_min(params.cool_min);
  set_cool_max(params.cool_max);
  active_palette = palette_map[params.palette];

  // We don't load the gyro parameters - restore them from the device
  for (auto i = 0; i < 3; ++i) {
    params.gyro_offset[i] = dmp_get_offset(static_cast<dmp_axis>(i));
    params.accel_offset[i] = dmp_get_offset(static_cast<dmp_axis>(i+3));
  }

  // Notify Blynk of our newly-loaded values
  static auto timer_handle =  BlynkTimer::Handle {};
  if (timer_handle.isValid()) {
    timer_handle.deleteTimer();
  }
  auto update_pin = 0U;
  timer_handle = timer_core.setTimer(100, [=]() mutable {
    Serial.printf("[%ld] Updating %d\n", millis(), update_pin);
    switch(update_pin) {
      case 0: Blynk.virtualWrite(V0, params.target_temperature); break;
      case 1: Blynk.virtualWrite(V1, params.blade_kd); break;
      case 2: Blynk.virtualWrite(V2, params.blade_ki); break;
      case 3: Blynk.virtualWrite(V3, params.palette); break;
      case 4: Blynk.virtualWrite(V4, params.base_brightness); break;
      case 5: Blynk.virtualWrite(V5, params.max_brightness); break;
      case 6: Blynk.virtualWrite(V6, params.acc_sensitivity); break;
      case 7: Blynk.virtualWrite(V7, params.gyro_sensitivity); break;
      case 8: Blynk.virtualWrite(V8, params.surge_sensitivity); break;
      case 9: Blynk.virtualWrite(V10, params.fwd_color_scale); break;
      case 10: Blynk.virtualWrite(V11, params.back_color_scale); break;
      case 11: Blynk.virtualWrite(V12, params.surge_falloff); break;
      case 12: Blynk.virtualWrite(V13, params.cool_min); break;
      case 13: Blynk.virtualWrite(V14, params.cool_max); break;
      default:
        // nope
        ;
    }
    Serial.printf("[%ld] Updated %d\n", millis(), update_pin);
    ++update_pin;    
  }, 14) ;
}



BLYNK_WRITE_DEFAULT() {
  int pin = request.pin;

  // Common pin blocks

  if ((pin >= 50) && (pin < 56)) {
    // dmp calibration pin
    dmp_set_offset(static_cast<dmp_axis>(request.pin - 50), param.asInt()); delayed_eeprom_write();
    return;
  }
  
  if ((pin >= 60) && (pin < 70)) {
    // save params
    if (param.asInt() != 0) {
      save_params(pin - 60);
      Serial.printf("[%ld] Saved slot %d\n",millis(),pin-60);
    }
    return;
  }


  if ((pin >= 70) && (pin < 80)) {
    // save params
    if (param.asInt() != 0) {
      Serial.printf("[%ld] Loading slot %d\n",millis(),pin-70);
      load_params(pin - 70);
      apply_params();
    }
    return;
  }
}

// This function is called every time the device is connected to the Blynk.Cloud
BLYNK_CONNECTED()
{
}

// This function sends Arduino's uptime every second to Virtual Pin 2.
void sim_timer_event()
{
    auto accel_values = read_dmp();
    auto led_values = simulate_temperature();

    //Serial.printf("[%ld] %f, %f, %f\n", millis(), accel_values[0], accel_values[1], accel_values[2]);

    // Apply effects    
    static energy<NUM_LEDS> energy_forward;
    energy_forward(led_values, accel_values[0] * params.acc_sensitivity);

    static surge surge_state;
    auto surge_brightness = surge_state(accel_values[1] * params.surge_sensitivity);
    auto total_brightness = params.base_brightness + (params.max_brightness - params.base_brightness) * surge_brightness;

    auto& active_palette = palette_map[params.palette];
    for(auto i = 0U; i < NUM_LEDS; ++i) {
      // Local "nowrap"
      auto led_value = led_values[i] / 40;  // temperature to color scale
      if (active_palette.blend_type == LINEARBLEND_NOWRAP) {
        // clamp values
        led_value = min(240, max(0, led_value));  // Cap at 240, because 240-255 is "wrapping" back to 0.
      } else {
      // just use LSBs - let noise "wrap"
        led_value &= 0xFF;
      }

      leds[i] = ColorFromPalette(active_palette.palette, static_cast<uint8>(led_value), std::min((int) total_brightness, 255), LINEARBLEND);
    }

    //sparkle(leds);

    //FastLED.setBrightness();
    //FastLED.show();
}


void setup() {
  Serial.begin(115200);
  Serial.printf("Starting up!\n");

  // Show an indicator LED while we start up
  leds.fill({0,0,0});
  leds[0].r = 128;
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds.data(), leds.size());//.setCorrection( TypicalLEDStrip );    
  FastLED.setDither(BINARY_DITHER);
  BlynkEdgent.begin();

  // embiggen the eeprom range for dmp calibration
  init_params();
  init_dmp(params.gyro_offset, params.accel_offset);
  apply_params(); // apply other parameters

  timer_core.setInterval(20L, sim_timer_event);    
};


void app_loop()
{
  edgentTimer.run();
  edgentConsole.run();
  timer_core.run();  
  FastLED.show();
}

void loop() {
  BlynkEdgent.run();
}
