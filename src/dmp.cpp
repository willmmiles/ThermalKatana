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


struct dmp_config {
  int32_t magic;  // are we good?
  int16_t gyro_offset[3]; // 94, -20, -20 for katana;  53,  -18, 30 for test board
  int16_t accel_offset[3];  // test board: -1250, -6433, 1345
};



// Global objects
static MPU6050 mpu;
static auto active_config = dmp_config {};
static auto g_vector = Eigen::Vector<float, 3> {};

static void calibrate_g_vector() {
  // Calibrate our gravity vector
    auto sum = Eigen::Vector<float,3> {0,0,0};
    unsigned n_samples = 0;
    while (n_samples < 50) {
      uint8_t fifoBuffer[64]; // FIFO storage buffer
      if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
          // parse packet in to raw readings, taken from MPU6050 examples
          Quaternion q;
          VectorInt16 aa;         // [x, y, z]            accel sensor measurements
          mpu.dmpGetQuaternion(&q, fifoBuffer);        
          mpu.dmpGetAccel(&aa, fifoBuffer);
          auto eq = Eigen::Quaternion<float> { q.w, q.x, q.y, q.z };
          auto accel = Eigen::Vector<float, 3> { (float) aa.x, (float) aa.y, (float) aa.z };
          sum += eq.inverse() * accel;
          ++n_samples;
      }
    }
    g_vector = sum / n_samples;
    Serial.printf("G vector: %f, %f, %f\n",g_vector[0],g_vector[1], g_vector[2]);
}


void dmp_set_offset(dmp_axis axis, int16_t value) {
  switch(axis) {
    case dmp_axis::gyro_x: active_config.gyro_offset[0] = value; mpu.setXGyroOffset(value); break;
    case dmp_axis::gyro_y: active_config.gyro_offset[1] = value; mpu.setYGyroOffset(value); break;
    case dmp_axis::gyro_z: active_config.gyro_offset[2] = value; mpu.setZGyroOffset(value); break;
    case dmp_axis::accel_x: active_config.accel_offset[0] = value; mpu.setXAccelOffset(value); break;
    case dmp_axis::accel_y: active_config.accel_offset[1] = value; mpu.setYAccelOffset(value); break;
    case dmp_axis::accel_z: active_config.accel_offset[2] = value; mpu.setZAccelOffset(value); break;       
  }

  // Re-run the calibration since we've adjusted the offsets
  calibrate_g_vector();
}

void dmp_save_offset() {
  EEPROM.put(EEPROM_CONFIG_START, active_config);
  EEPROM.commit();
}


void init_dmp() {
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

    // supply your own gyro offsets here, scaled for min sensitivity
    EEPROM.get(EEPROM_CONFIG_START, active_config);
    if (!(active_config.magic == EEPROM_MAGIC)) {     
      memset(&active_config, 0, sizeof(active_config));
      active_config.magic = EEPROM_MAGIC;
      // Default to the "factory trim" values
      active_config.accel_offset[0] = mpu.getAccelXSelfTestFactoryTrim();
      active_config.accel_offset[1] = mpu.getAccelYSelfTestFactoryTrim();
      active_config.accel_offset[2] = mpu.getAccelZSelfTestFactoryTrim();
    }
    mpu.setXGyroOffset(active_config.gyro_offset[0]);
    mpu.setYGyroOffset(active_config.gyro_offset[1]);
    mpu.setZGyroOffset(active_config.gyro_offset[2]);
    mpu.setXAccelOffset(active_config.accel_offset[0]);
    mpu.setYAccelOffset(active_config.accel_offset[1]);
    mpu.setZAccelOffset(active_config.accel_offset[2]);

    //Serial.printf("Factory trim: %d, %d ,%d\n",mpu.getAccelXSelfTestFactoryTrim(),mpu.getAccelYSelfTestFactoryTrim(),mpu.getAccelZSelfTestFactoryTrim());

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        mpu.PrintActiveOffsets();
        mpu.setRate(4); // WM: Onboard filters seem tuned for this rate
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        
        calibrate_g_vector();
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



Eigen::Vector2f read_dmp()
{
  static auto first_time = false; // todo this in setup
  static auto last_angle = Eigen::Quaternion<float> {};
  Eigen::Vector2f result = {0., 0.};

  uint8_t fifoBuffer[64]; // FIFO storage buffer
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
      // parse packet in to raw readings, taken from MPU6050 examples
      Quaternion q;
      VectorInt16 aa;         // [x, y, z]            accel sensor measurements
      mpu.dmpGetQuaternion(&q, fifoBuffer);        
      mpu.dmpGetAccel(&aa, fifoBuffer);

      // Uplift to a more fully-featured math library
      auto eq = Eigen::Quaternion<float> { q.w, q.x, q.y, q.z };
      auto accel = Eigen::Vector3f { Eigen::Map<Eigen::Vector<int16_t, 3>>(&aa.x).cast<float>() };

      if (!first_time) {
        auto delta_angle = eq.angularDistance(last_angle);
        
        Serial.printf("%f, %f, %f -- ",accel[0], accel[1], accel[2]);
        // Subtract gravity vector
        accel -= (eq.inverse() * g_vector);
        Serial.printf("%f, %f, %f\n",accel[0], accel[1], accel[2]);

        auto handle_accel = accel.norm();

        // good, good!
        Serial.printf("%f, %f\n",handle_accel, delta_angle);
        
        // dump raw accel data for calibration
        //Serial.printf("%d, %d, %d, %d, %d, %d\n", aa.x, aa.y, aa.z, aaRaw.x, aaRaw.y, aaRaw.z);
        result = Eigen::Vector2f { handle_accel, delta_angle };
      } else {
        first_time = false;
      }
      last_angle = eq;
  };

  return result;
}

