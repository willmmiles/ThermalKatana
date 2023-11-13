// DMP-related calls

#include <cstdint>
#include "eigen.h"

#pragma once

enum class dmp_axis {
    gyro_x = 0,
    gyro_y,
    gyro_z,
    accel_x,
    accel_y,
    accel_z,
};

void init_dmp(int16_t gyro_offset[3], int16_t accel_offset[3]); // send INT16_MIN to use factory default

void dmp_set_offset(dmp_axis, int16_t value);
int16_t dmp_get_offset(dmp_axis);

struct dmp_data {
    Eigen::Vector<int16_t, 3> gyros;
    Eigen::Vector<int16_t, 3> accels;
    Eigen::Vector3f results; // accel - G, tip delta pos, delta angle
};

dmp_data read_dmp();
