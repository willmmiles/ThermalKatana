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
void dmp_save_offset();

Eigen::Vector3f read_dmp();
