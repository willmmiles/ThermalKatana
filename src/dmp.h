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

void init_dmp();

void dmp_set_offset(dmp_axis, int16_t value);
void dmp_save_offset();

Eigen::Vector2f read_dmp();
