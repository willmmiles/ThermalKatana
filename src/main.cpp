#include "config.h"

// FastLED configuration
#define FASTLED_ESP8266_NODEMCU_PIN_ORDER
#define FASTLED_ALL_PINS_HARDWARE_SPI
#define LED_PIN     4                    // which pin?
#define LED_TYPE    WS2812B              // preset for the WS2812B
#define COLOR_ORDER GRB                  // change if needed

// Blynk configuration
/* Fill-in information from Blynk Device Info here */
#define BLYNK_TEMPLATE_ID           "TMPL2wlrA9g8Y"
#define BLYNK_TEMPLATE_NAME         "Quickstart Template"
#define BLYNK_AUTH_TOKEN            "fpvsNPJbLDDyqtc61it5aTROP0iwIrca"

/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial


#include <Arduino.h>
#include <FastLED.h>                     // needed for WS2812B LEDs
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include "blade_simulation.h"


// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "";
char pass[] = "";

BlynkTimer timer;

CRGB leds[NUM_LEDS];



// This function is called every time the Virtual Pin 0 state changes
BLYNK_WRITE(V0)
{
  // Set incoming value from pin V0 to a variable
  int value = param.asInt();

  // Update state
  Blynk.virtualWrite(V1, value);
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
void myTimerEvent()
{
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  Blynk.virtualWrite(V2, millis() / 1000);
}





void setup() {
  Serial.begin(115200);
  Serial.printf("Starting up!\n");
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  timer.setInterval(1000L, myTimerEvent);    

  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness(32);
  // TODO: tune this value
  FastLED.setMaxPowerInVoltsAndMilliamps(3.7,1000); 
//  startup();
}

void loop()
{
  Blynk.run();
  timer.run();

  static auto last_sim_time = 0;

  auto time_now = millis();
  if ((time_now - last_sim_time) > 20) {
    last_sim_time = time_now;
    auto led_values = simulate_temperature();
    for(auto i = 0U; i < NUM_LEDS; ++i) {
      leds[i] = HeatColor(led_values[i]);
    }
    FastLED.show();
  }
}