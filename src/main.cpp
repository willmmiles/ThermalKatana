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

extern void init_dmp();
extern void read_dmp(); // TODO

BlynkTimer sim_timer;

std::array<CRGB,NUM_LEDS> leds;

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
decltype(HeatColors_p)* active_palette = palette_map[0];

// This function is called every time the Virtual Pin 0 state changes
BLYNK_WRITE(V0)
{
  // Set incoming value from pin V0 to a variable
  int value = param.asInt();
  setTargetTemperature(value);
}
 
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



// This function is called every time the device is connected to the Blynk.Cloud
BLYNK_CONNECTED()
{
}

// This function sends Arduino's uptime every second to Virtual Pin 2.
void sim_timer_event()
{
    static auto bright_pixel = 0U;
    read_dmp();

    auto led_values = simulate_temperature();
    for(auto i = 0U; i < NUM_LEDS; ++i) {
      //leds[i] = HeatColor(led_values[i]);
      leds[i] = ColorFromPalette(*active_palette, led_values[i], i == bright_pixel ? 255 : 128);
    }
    bright_pixel = (bright_pixel + 1) % NUM_LEDS;
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

  init_dmp();

  sim_timer.setInterval(20L, sim_timer_event);    
  BlynkEdgent.begin();



void app_loop()
{
  edgentTimer.run();
  edgentConsole.run();
  sim_timer.run();  
}

void loop() {
  BlynkEdgent.run();
}
