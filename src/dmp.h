// DMP-related calls

#include <cstdint>

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

void dmp_setOffset(dmp_axis, int16_t value);

void read_dmp();    // todo: return type
