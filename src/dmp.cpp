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
constexpr auto EEPROM_MAGIC = 0xDDAADD00;
constexpr auto ESTIMATED_G = 9.801;  // m/s^2
constexpr auto ESTIMATED_G_COUNTS = 8350.;


// Global object
MPU6050 mpu;

struct dmp_config {
  int32_t magic;  // are we good?
  int16_t gyro_offset[3]; // 94, -20, -20 for katana;  53,  -18, 30 for test board
  int16_t accel_offset[3];  // test board: -1250, -6433, 1345
};

auto active_config = dmp_config {};

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

    Serial.printf("Factory trim: %d, %d ,%d\n",mpu.getAccelXSelfTestFactoryTrim(),mpu.getAccelYSelfTestFactoryTrim(),mpu.getAccelZSelfTestFactoryTrim());

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        //mpu.PrintActiveOffsets();
        mpu.setRate(4);
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
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


void dmp_setOffset(dmp_axis axis, int16_t value) {
  switch(axis) {
    case dmp_axis::gyro_x: active_config.gyro_offset[0] = value; mpu.setXGyroOffset(value); break;
    case dmp_axis::gyro_y: active_config.gyro_offset[1] = value; mpu.setYGyroOffset(value); break;
    case dmp_axis::gyro_z: active_config.gyro_offset[2] = value; mpu.setZGyroOffset(value); break;
    case dmp_axis::accel_x: active_config.accel_offset[0] = value; mpu.setXAccelOffset(value); break;
    case dmp_axis::accel_y: active_config.accel_offset[1] = value; mpu.setYAccelOffset(value); break;
    case dmp_axis::accel_z: active_config.accel_offset[2] = value; mpu.setZAccelOffset(value); break;       
  }
  EEPROM.put(EEPROM_CONFIG_START, active_config);
  EEPROM.commit();
}


void read_dmp()
{
  static auto first_time = false; // todo this in setup
  static auto last_position = Eigen::Vector3f { };
  static auto last_end_v = Eigen::Vector3f { 0, 0 ,0 };

  uint8_t fifoBuffer[64]; // FIFO storage buffer
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
      // parse packet in to raw readings, taken from MPU6050 examples
      Quaternion q;
      VectorInt16 aa;         // [x, y, z]            accel sensor measurements
      VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
      VectorFloat gravity;    // [x, y, z]            gravity vector
      mpu.dmpGetQuaternion(&q, fifoBuffer);        
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);    
      VectorInt16 aaRaw;
      mpu.getAcceleration(&aaRaw.x, &aaRaw.y, &aaRaw.z);

      // Uplift to a more fully-featured math library
      auto eq = Eigen::Quaternion<float> { q.w, q.x, q.y, q.z };
      // Compute new end position
      auto tip_position = eq * Eigen::Vector3f { 0, (ESTIMATED_G_COUNTS / ESTIMATED_G), 0 }; // 1 "m" in acc counts (G~=8500)

      // TODO: on the first few samples, measure local G direction.  Our initial direction is always zero regardless of the orientation.
      // We do have a risk of gyro drift, but we hope it's small enough to ignore over the lifetime of our system

      if (!first_time) {
        Eigen::Vector3f delta_pos = (tip_position - last_position) * 50; // "counts" per second
        Eigen::Vector3f tip_acc = (last_end_v - delta_pos) ; // "counts" per second^2

        auto end_accel = (tip_acc - Eigen::Map<Eigen::Vector<int16_t,3>>(&aa.x).cast<float>()).norm();
        auto handle_accel = aa.getMagnitude();

        // good, good!
        Serial.printf("%f, %f\n",handle_accel, end_accel);
        
        // dump raw accel data for calibration
        //Serial.printf("%d, %d, %d, %d, %d, %d\n", aa.x, aa.y, aa.z, aaRaw.x, aaRaw.y, aaRaw.z);

      } else {
        first_time = false;
      }
      last_position = tip_position;
  };
    // TODO: I think we want the *difference* between this and prev gyro angles, eg. rotational velocity
    // We can then "charge" the brightness with the rotational velocity used.

}

