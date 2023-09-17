#include <Arduino.h>
#define FASTLED_ESP8266_NODEMCU_PIN_ORDER
#define FASTLED_ALL_PINS_HARDWARE_SPI
#include <FastLED.h>                     // needed for WS2812B LEDs
#include "config.h"
#include "blade_simulation.h"

#define LED_PIN     4                    // which pin?
#define LED_TYPE    WS2812B              // preset for the WS2812B
#define COLOR_ORDER GRB                  // change if needed

CRGB leds[NUM_LEDS];


void setup() {
  Serial.begin(115200);
  Serial.printf("Starting up!\n");
  delay( 300 );                           
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness(32);
  // TODO: tune this value
  FastLED.setMaxPowerInVoltsAndMilliamps(3.7,1000); 
//  startup();
}

void loop()
{
  auto led_values = simulate_temperature();
  for(auto i = 0U; i < NUM_LEDS; ++i) {
    leds[i] = HeatColor(led_values[i]);
  }
  FastLED.show();
  delay(20);
}