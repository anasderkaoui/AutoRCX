#include "lsm9ds1_driver.h"
#include <unistd.h>
#include <cstring>
#include <cmath>
#include <limits>
#include <iostream>

lsm9ds1_driver::lsm9ds1_driver() {}
lsm9ds1_driver::~lsm9ds1_driver() {}

void lsm9ds1_driver::set_data_callback(std::function<void(data)> cb) {
    m_data_callback = cb;
}

void lsm9ds1_driver::initialize(unsigned int i2c_bus_ag, unsigned int i2c_addr_ag,
                                unsigned int i2c_addr_mag)
{
    initialize_i2c(i2c_bus_ag, i2c_addr_ag, i2c_addr_mag);

    // Confirm device IDs
    uint8_t whoami_ag = read_ag_register(reg_ag::WHO_AM_I);
    uint8_t whoami_m = read_m_register(reg_m::WHO_AM_I);
    if (whoami_ag != 0x68) {
        throw std::runtime_error("LSM9DS1 AG WHO_AM_I mismatch: " + std::to_string(whoami_ag));
    }
    if (whoami_m != 0x3D) {
        throw std::runtime_error("LSM9DS1 MAG WHO_AM_I mismatch: " + std::to_string(whoami_m));
    }

    // Gyro config: 245 dps, ODR 119 Hz
    write_ag_register(reg_ag::CTRL_REG1_G, 0b01100000); // 119 Hz, 245 dps

    // Accel config: ±2g, ODR 119 Hz
    write_ag_register(reg_ag::CTRL_REG6_XL, 0b01100000); // 119 Hz, ±2g

    // Magnetometer config: Ultra-high-performance, 80Hz
    write_m_register(reg_m::CTRL_REG1_M, 0b11110000); // Ultra-high-performance, 80Hz
    write_m_register(reg_m::CTRL_REG2_M, 0b00000000); // ±4 gauss
    write_m_register(reg_m::CTRL_REG3_M, 0b00000000); // Continuous-conversion mode
    write_m_register(reg_m::CTRL_REG4_M, 0b00001100); // Z ultra-high-perf
}

void lsm9ds1_driver::deinitialize() {
    deinitialize_i2c();
}

lsm9ds1_driver::data lsm9ds1_driver::read_data() {
    data d;
    char accel_buf[6];
    char gyro_buf[6];
    char temp_buf[2];

    // ACCEL
    read_ag_registers(reg_ag::OUTX_L_XL, 6, accel_buf);
    int16_t ax = accel_buf[0] | (accel_buf[1] << 8);
    int16_t ay = accel_buf[2] | (accel_buf[3] << 8);
    int16_t az = accel_buf[4] | (accel_buf[5] << 8);
    d.accel_x = ax * 0.00061; // ±2g, 0.061 mg/LSB
    d.accel_y = ay * 0.00061;
    d.accel_z = az * 0.00061;
    // d.accel_x = ax; // Raw values
    // d.accel_y = ay;
    // d.accel_z = az;
    // GYRO
    read_ag_registers(reg_ag::OUTX_L_G, 6, gyro_buf);
    int16_t gx = gyro_buf[0] | (gyro_buf[1] << 8);
    int16_t gy = gyro_buf[2] | (gyro_buf[3] << 8);
    int16_t gz = gyro_buf[4] | (gyro_buf[5] << 8);
    d.gyro_x = gx * 0.0000875; // 245 dps, 8.75 mdps/LSB
    d.gyro_y = gy * 0.0000875;
    d.gyro_z = gz * 0.0000875;

    // d.gyro_x = gx; // Raw values
    // d.gyro_y = gy;
    // d.gyro_z = gz;


    // TEMP
    read_ag_registers(reg_ag::OUT_TEMP_L, 2, temp_buf);
    int16_t temp = temp_buf[0] | (temp_buf[1] << 8);
    d.temp = 25.0 + (float)temp / 16.0;

    // MAG
    char mag_buf[6];
    read_m_registers(reg_m::OUTX_L_M, 6, mag_buf);
    int16_t mx = mag_buf[0] | (mag_buf[1] << 8);
    int16_t my = mag_buf[2] | (mag_buf[3] << 8);
    int16_t mz = mag_buf[4] | (mag_buf[5] << 8);
    // d.magneto_x = mx * 0.00014; // ±4 gauss, 0.14 mgauss/LSB
    // d.magneto_y = my * 0.00014;
    // d.magneto_z = mz * 0.00014;
    d.magneto_x = mx * 0.014;
    d.magneto_y = my * 0.014;
    d.magneto_z = mz * 0.014;

    if (m_data_callback) m_data_callback(d);
    return d;
}
