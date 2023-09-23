#include "config.h"

// Need to be first; some macros in Arduino.h screw this up
#include <eigen.h>
#include <Eigen/Geometry>

#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"


// Global object
MPU6050 mpu;


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
    mpu.setXGyroOffset(94);
    mpu.setYGyroOffset(-20);
    mpu.setZGyroOffset(-20);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);        
        mpu.PrintActiveOffsets();
        mpu.setRate(20);
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

static auto first_time = false; // todo this in setup
static auto last_gyro_angle = Eigen::Quaternion<float> { };
static auto last_end_v = 0.;


void read_dmp()
{
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

      // Uplift to a more fully-featured math library
      auto eq = Eigen::Quaternion<float> { q.w, q.x, q.y, q.z };
      if (!first_time) {
        auto angular_distance = eq.angularDistance(last_gyro_angle);
        // convert to linear distance
        auto linear_distance = sin(angular_distance / 2) * (2 * 1); // r=~1m
        auto linear_velocity = linear_distance * 50;  // Hz
        // derive acceleration
        auto end_accel = linear_velocity - last_end_v;
        auto handle_accel = aaReal.getMagnitude();

        Serial.printf("HA: %f   EA: %f\n",handle_accel, end_accel);

      } else {
        first_time = false;
      }
      last_gyro_angle = eq;
  };
    // TODO: I think we want the *difference* between this and prev gyro angles, eg. rotational velocity
    // We can then "charge" the brightness with the rotational velocity used.

}

