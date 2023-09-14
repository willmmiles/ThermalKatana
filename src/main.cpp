#include <Arduino.h>
#define FASTLED_ESP8266_NODEMCU_PIN_ORDER
#include <FastLED.h>                     // needed for WS2812B LEDs
#include "blackbody.h"

#define LED_PIN     4                    // which pin?
#define NUM_LEDS    110                   // adjust to your amount of leds on the strip
#define BRIGHTNESS  200                  // almost full brightness (0-255)
#define LED_TYPE    WS2812B              // preset for the WS2812B
#define COLOR_ORDER GRB                  // change if needed

//const static auto COLOR = CRGB(255, 69, 0); // orange
const static auto COLOR = CRGB { 155, 188, 255} ; // 16000 K

CRGB leds[NUM_LEDS];

void startup();

void setup() {
  delay( 300 );                           
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
  startup();
}

void loop()
{
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = COLOR;
    leds[random(NUM_LEDS)].fadeToBlackBy(random(40,60));
  }
  FastLED.show();
  delay(20);
}

void startup()
{
  for (int f = 0; f < BRIGHTNESS; f++) {
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = COLOR;
      FastLED.setBrightness(  f );
      leds[random(NUM_LEDS)].fadeToBlackBy(255);
    }
  FastLED.show();
  delay(8);
  }
}
