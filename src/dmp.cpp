#include "config.h"

// Need to be first; some macros in Arduino.h screw this up
#include <eigen.h>
#include <Eigen/Geometry>

#include <Arduino.h>
#include <EEPROM.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "dmp.h"

constexpr auto EEPROM_CONFIG_START = 0x100;
constexpr auto EEPROM_MAGIC = (int32_t) 0xDDAADD00;
constexpr auto ESTIMATED_G = 9.801;  // m/s^2
constexpr auto ESTIMATED_G_COUNTS = 8350.;
constexpr auto STARTUP_DELAY = 200U; // samples
constexpr auto CALIBRATE_SAMPLES = 50U; // samples


// Global objects
static MPU6050 mpu;
// Calibration state
// We keep this in memory as we can't get it back from the DPU if it fails to initialize.
int16_t active_gyro_offset[3];
int16_t active_accel_offset[3];

// G analysis
static auto g_vector = Eigen::Vector<float, 3> {};
static auto sample_count = 0U;
// Motion calculation static variables
static auto last_orientation_abs = Eigen::Quaternion<float> {};
static auto last_position = Eigen::Vector3f { 0, 0, 0 };
static auto last_delta_pos = Eigen::Vector3f { 0, 0 ,0 };


void dmp_set_offset(dmp_axis axis, int16_t value) {
  switch(axis) {
    case dmp_axis::gyro_x: active_gyro_offset[0] = value; mpu.setXGyroOffset(value); break;
    case dmp_axis::gyro_y: active_gyro_offset[1] = value; mpu.setYGyroOffset(value); break;
    case dmp_axis::gyro_z: active_gyro_offset[2] = value; mpu.setZGyroOffset(value); break;
    case dmp_axis::accel_x: active_accel_offset[0] = value; mpu.setXAccelOffset(value); break;
    case dmp_axis::accel_y: active_accel_offset[1] = value; mpu.setYAccelOffset(value); break;
    case dmp_axis::accel_z: active_accel_offset[2] = value; mpu.setZAccelOffset(value); break;       
  }

  // Re-run the calibration since we've adjusted the offsets
  sample_count = std::min(sample_count, STARTUP_DELAY);
  g_vector.setZero();
}


int16_t dmp_get_offset(dmp_axis axis) {
  switch(axis) {
    case dmp_axis::gyro_x: return active_gyro_offset[0];
    case dmp_axis::gyro_y: return active_gyro_offset[1];
    case dmp_axis::gyro_z: return active_gyro_offset[2];
    case dmp_axis::accel_x: return active_accel_offset[0];
    case dmp_axis::accel_y: return active_accel_offset[1];
    case dmp_axis::accel_z: return active_accel_offset[2];
  }
  return 0; // ?
}

// utility
static void set_default(int16_t& target, int16_t value) {
  if (target == INT16_MIN) target = value;
}

void init_dmp(int16_t gyro_offset[3], int16_t accel_offset[3]) {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    auto devStatus = mpu.dmpInitialize();

    // load gyro offsets, if unused
    set_default(gyro_offset[0], mpu.getGyroXSelfTestFactoryTrim());
    set_default(gyro_offset[1], mpu.getGyroYSelfTestFactoryTrim());
    set_default(gyro_offset[2], mpu.getGyroZSelfTestFactoryTrim());
    set_default(accel_offset[0], mpu.getAccelXSelfTestFactoryTrim());
    set_default(accel_offset[1], mpu.getAccelYSelfTestFactoryTrim());
    set_default(accel_offset[2], mpu.getAccelZSelfTestFactoryTrim());

    memcpy(active_gyro_offset, gyro_offset, sizeof(active_gyro_offset));
    memcpy(active_accel_offset, accel_offset, sizeof(accel_offset));

    mpu.setXGyroOffset(gyro_offset[0]);
    mpu.setYGyroOffset(gyro_offset[1]);
    mpu.setZGyroOffset(gyro_offset[2]);
    mpu.setXAccelOffset(accel_offset[0]);
    mpu.setYAccelOffset(accel_offset[1]);
    mpu.setZAccelOffset(accel_offset[2]);

    Serial.printf("Factory trim Gyro: %d, %d ,%d\n",mpu.getGyroXSelfTestFactoryTrim(),mpu.getGyroYSelfTestFactoryTrim(),mpu.getGyroZSelfTestFactoryTrim());
    Serial.printf("Factory trim Accel: %d, %d ,%d\n",mpu.getAccelXSelfTestFactoryTrim(),mpu.getAccelYSelfTestFactoryTrim(),mpu.getAccelZSelfTestFactoryTrim());

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        mpu.PrintActiveOffsets();
        mpu.setRate(4); // WM: Onboard filters seem tuned for this rate
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        sample_count = 0;
        g_vector.setZero();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}



Eigen::Vector3f read_dmp()
{
  Eigen::Vector3f result = {0., 0., 0.};

  uint8_t fifoBuffer[64]; // FIFO storage buffer
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
      // parse packet in to raw readings, taken from MPU6050 examples
      Quaternion q;
      VectorInt16 aa;         // [x, y, z]            accel sensor measurements
      mpu.dmpGetQuaternion(&q, fifoBuffer);        
      mpu.dmpGetAccel(&aa, fifoBuffer);

      // Uplift to a more fully-featured math library
      auto orientation_abs = Eigen::Quaternion<float> { q.w, q.x, q.y, q.z };
      auto accel_rel = Eigen::Map<Eigen::Vector<int16_t, 3>>(&aa.x).cast<float>();
      auto accel_abs = Eigen::Vector3f { orientation_abs * accel_rel };

      // uncomment for calibration
      // TODO: send this back to Blynk?
      // Serial.printf("[%f, %f, %f]\n",accel_rel[0], accel_rel[1], accel_rel[2]);

      if (sample_count > (STARTUP_DELAY+CALIBRATE_SAMPLES)) {
        auto delta_angle = orientation_abs.angularDistance(last_orientation_abs);
        
        // Subtract gravity vector
        accel_abs -= g_vector; 

        auto tip_position = orientation_abs * Eigen::Vector3f { ESTIMATED_G_COUNTS / ESTIMATED_G, 0, 0 }; // 1 "m"
        Eigen::Vector3f delta_pos = (tip_position - last_position) * 50; // m / s
        //Eigen::Vector3f tip_acc = (delta_pos - last_delta_pos) * 50; // m / s^s
        //Eigen::Vector3f end_accel = tip_acc + accel_abs;
        last_delta_pos = delta_pos;
        last_position = tip_position;

        //Serial.printf("[%f, %f, %f] ",accel_abs[0], accel_abs[1], accel_abs[2]);
        //Serial.printf("[%f, %f, %f]\n",delta_pos[0], delta_pos[1], delta_pos[2]);
        //Serial.printf("[%f, %f, %f]\n",tip_acc[0], tip_acc[1], tip_acc[2]);

        // Apply a filter to the handle acceleration
        // This is effectively a high pass filter
        static auto accel_return = 0.;
        accel_return = (0.2 * accel_return) + (0.8 * accel_abs.norm());

        result = Eigen::Vector3f { accel_return, delta_pos.norm(), delta_angle };
      } else {
        // DEBUG!        
        Serial.printf("%d [%f, %f, %f, %f] -- %f, %f, %f -- %f, %f, %f\n",
          sample_count,
          orientation_abs.w(), orientation_abs.x(), orientation_abs.y(), orientation_abs.z(),
          accel_rel[0], accel_rel[1], accel_rel[2],
          accel_abs[0], accel_abs[1], accel_abs[2]
          );
        if (sample_count >= STARTUP_DELAY) {
          g_vector += accel_abs;
          if (sample_count == (STARTUP_DELAY + CALIBRATE_SAMPLES)) {
            g_vector /= CALIBRATE_SAMPLES;  // all done!
            Serial.printf("G vector: %f, %f, %f\n",g_vector[0],g_vector[1], g_vector[2]);          
          }
        }
        ++sample_count;
      }

      last_orientation_abs = orientation_abs;
      
  };

  return result;
}

