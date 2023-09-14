#include <Arduino.h>
#define FASTLED_ESP8266_NODEMCU_PIN_ORDER
#define FASTLED_ALL_PINS_HARDWARE_SPI
#include <FastLED.h>                     // needed for WS2812B LEDs
#include "eigen.h"

#define LED_PIN     4                    // which pin?
#define NUM_LEDS    110                   // adjust to your amount of leds on the strip
#define BRIGHTNESS  200                  // almost full brightness (0-255)
#define LED_TYPE    WS2812B              // preset for the WS2812B
#define COLOR_ORDER GRB                  // change if needed


constexpr auto AMBIENT = 20;
constexpr auto KI = 0.1;  // controller parameters
constexpr auto KD = 0.1;
constexpr auto dT = 0.1;  // "delta-T"
constexpr auto TARGET = 8000;  // target temperature


//const static auto COLOR = CRGB(255, 69, 0); // orange
const static auto COLOR = CRGB { 155, 188, 255} ; // 16000 K
CRGB leds[NUM_LEDS];

typedef Eigen::Array<int,NUM_LEDS,1> temperature_array_t;
temperature_array_t temperature = temperature_array_t::Constant(AMBIENT);
Eigen::Array<float,NUM_LEDS,1> integral = Eigen::Array<float,NUM_LEDS,1>::Constant(0.);

template<typename T>
static void print_array(T& array) {
  Serial.print("[");
  for (int i = 0; i < array.size(); ++i) Serial.printf(" %d", array[i]);
  Serial.println(" ]");
}


temperature_array_t smooth_temperatures(const temperature_array_t &input) {
  temperature_array_t output = temperature_array_t::Zero();;
  const Eigen::Vector<int, 5> smoothing_kernel = { 13,	1573,	6828,	1573,	13 };
  const auto half_kernel_size = (smoothing_kernel.size()/2);
  const auto inner_size = input.size() - smoothing_kernel.size();

  output.segment(half_kernel_size, inner_size)     
    =  (((input.segment(0, inner_size) * smoothing_kernel[0]) +
       (input.segment(1, inner_size) * smoothing_kernel[1]) +
       (input.segment(2, inner_size) * smoothing_kernel[2]) +
       (input.segment(3, inner_size) * smoothing_kernel[3]) +
       (input.segment(4, inner_size) * smoothing_kernel[4])) / 10000) - input.segment(half_kernel_size, inner_size);

  //output.segment(0,half_kernel_size) = 0;
  //output.segment(output.size() - (half_kernel_size+1), half_kernel_size )= 0;

  return output;
}

temperature_array_t cool(const temperature_array_t &input, int ambient) {
  // TODO: do we need to return the whole array?
  temperature_array_t output = temperature_array_t::Zero();;
  for (int i = 0; i < NUM_LEDS; i++) {
    auto j = random(NUM_LEDS);
    output[j] -= (random(40,60) * (input[j]/ambient)) / 100;  // lose between 40 and 60% of the value randomly
    output[j] = max(output[j], ambient - input[j]);  // don't cool below ambient
  }
  return output;
}


temperature_array_t control(const temperature_array_t &input, float ki, float kd, int target, Eigen::Array<float,NUM_LEDS,1>& integral) {
    auto error = Eigen::Array<float,NUM_LEDS,1> { (target - input).cast<float>() };
    integral += error * dT;
    return ((kd * error) + (ki * integral)).cast<int>();
}


// Temperature simulation
void simulate_temperature() {
  // Our temperature simulation consists of three parts:
  // First, "smear" the temperatures in to neighbours (eg. apply a low-pass filter).
  auto temperature_change = smooth_temperatures(temperature);
  //Serial.print("Smoothing: ");
  //print_array(temperature_change);
  // Second, "cool" the temperatures using a stochastic model.  This produces a "flickery flame" effect.
  temperature_change += cool(temperature, AMBIENT);
  // Third, apply a PID controller to each cell in the model - this simulates the "real" controlled heat effect.
  temperature_change += control(temperature, KI, KD, TARGET, integral);

  // Finally apply the results
  temperature += temperature_change;
  temperature = temperature.cwiseMax(0).cwiseMin(10000);

  for (auto i = 0; i < NUM_LEDS; ++i) {
    leds[i] = HeatColor(std::min(temperature[i] / 40, 255));
  }
}



void setup() {
  Serial.begin(115200);
  Serial.printf("Starting up!\n");
  delay( 300 );                           
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection( TypicalLEDStrip );
  FastLED.setBrightness(32);
  // TODO: tune this value
  FastLED.setMaxPowerInVoltsAndMilliamps(5,1000); 
//  startup();
}

void loop()
{
  simulate_temperature();
  FastLED.show();
  //print_array(temperature);
  delay(20);
}