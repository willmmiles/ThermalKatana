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

CRGB leds[NUM_LEDS];

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
  // Change Web Link Button message to "Congratulations!"
  Blynk.setProperty(V3, "offImageUrl", "https://static-image.nyc3.cdn.digitaloceanspaces.com/general/fte/congratulations.png");
  Blynk.setProperty(V3, "onImageUrl",  "https://static-image.nyc3.cdn.digitaloceanspaces.com/general/fte/congratulations_pressed.png");
  Blynk.setProperty(V3, "url", "https://docs.blynk.io/en/getting-started/what-do-i-need-to-blynk/how-quickstart-device-was-made");
}

// This function sends Arduino's uptime every second to Virtual Pin 2.
void sim_timer_event()
{
    read_dmp();

    auto led_values = simulate_temperature();
    for(auto i = 0U; i < NUM_LEDS; ++i) {
      //leds[i] = HeatColor(led_values[i]);
      leds[i] = ColorFromPalette(*active_palette, led_values[i], 128);
    }
    FastLED.show();
}


void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.printf("Starting up!\n");
  init_dmp();

  BlynkEdgent.begin();
  sim_timer.setInterval(20L, sim_timer_event);    

  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness(32);
  // TODO: tune this value
  FastLED.setMaxPowerInVoltsAndMilliamps(3.7,1000); 
//  startup();
}


void app_loop()
{
  edgentTimer.run();
  edgentConsole.run();
  sim_timer.run();  
}

void loop() {
  BlynkEdgent.run();
}
