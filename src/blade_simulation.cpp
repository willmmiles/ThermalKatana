// Simulation of a "plasma blade"
// Each node holds a temperature value; on each tick, temperatures spread outward, cool, and then a controller drives them towards the target.
#include <Arduino.h>
#include "blade_simulation.h"

constexpr auto AMBIENT = 295;
auto KI = 0.0;  // controller parameters
auto KD = 0.1;
constexpr auto dT = 0.1;  // "delta-T"
constexpr auto CONTROL_MAX = 500; // degrees per tick
auto TARGET = 5000;  // target temperature

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
  //const Eigen::Vector<int, 5> smoothing_kernel = { 13,	1573,	6828,	1573,	13 };
  //const Eigen::Vector<int, 3> smoothing_kernel = { 1500,	7000,	1500 }	;
  const Eigen::Vector<int, 3> smoothing_kernel = { 900,	8200,	900 }	;
  //const Eigen::Vector<int, 3> smoothing_kernel = { 2500,	5000,	2500 }	;
  const auto magnitude = smoothing_kernel.sum();
  const auto half_kernel_size = (smoothing_kernel.size()/2);
  const auto inner_size = input.size() - smoothing_kernel.size();

  for(auto i = 0; i < smoothing_kernel.size(); ++i) {
    output.segment(half_kernel_size, inner_size) += input.segment(i, inner_size) * smoothing_kernel[i];
  }
  output.segment(half_kernel_size, inner_size) /= magnitude;
  output.segment(half_kernel_size, inner_size) -= input.segment(half_kernel_size, inner_size);

  output.segment(0,half_kernel_size) = 0;
  output.segment(output.size() - (half_kernel_size+1), half_kernel_size )= 0;

  return output;
}

temperature_array_t cool(const temperature_array_t &input, int ambient) {
  // TODO: do we need to return the whole array?
  temperature_array_t output = temperature_array_t::Zero();;
  constexpr int pixel_count[] = { 0, 0, 1, 1, 1, 1, 1, 1, 2, 2 };
  auto n_pixels = random(6);
  for (int i = 0; i < pixel_count[n_pixels]; i++) {
    auto j = random(NUM_LEDS);
    output[j] -= (random(-20,50) * (input[j] - ambient)) / 100;  // lose between 40 and 60% of the value randomly
    output[j] = max(output[j], ambient - input[j]);  // don't cool below ambient
  }
  return output;
}


temperature_array_t control(const temperature_array_t &input, float ki, float kd, int target, Eigen::Array<float,NUM_LEDS,1>& integral) {
    auto error = Eigen::Array<float,NUM_LEDS,1> { (target - input).cast<float>() };
    integral += error * dT;
    return ((kd * error) + (ki * integral)).cast<int>().cwiseMax(-CONTROL_MAX).cwiseMin(CONTROL_MAX);
}


// Temperature simulation
led_value_t simulate_temperature() {
  // Our temperature simulation consists of three parts:
  // First, "smear" the temperatures in to neighbours (eg. apply a low-pass filter).
  auto temperature_change = smooth_temperatures(temperature);
  //Serial.print("Smoothing: ");
  //print_array(temperature_change);
  // Second, "cool" the temperatures using a stochastic model.  This produces a "flickery flame" effect.
  // temperature_change += cool(temperature, AMBIENT);
  // Third, apply a PID controller to each cell in the model - this simulates the "real" controlled heat effect.
  temperature_change += control(temperature, KI, KD, TARGET, integral);

  // Finally apply the results
  temperature += temperature_change;
  temperature = temperature.cwiseMax(0).cwiseMin(10000);

  //print_array(temperature);

  return temperature.cast<uint16_t>();
}

void set_target_temperature(int degrees_k) { TARGET = degrees_k; }
void set_kd(float kd) { KD = kd; };
void set_ki(float ki) { KI = ki; };
  