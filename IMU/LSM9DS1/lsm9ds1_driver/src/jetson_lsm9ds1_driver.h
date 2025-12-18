#ifndef JETSON_LSM9DS1_DRIVER_H
#define JETSON_LSM9DS1_DRIVER_H

#include "lsm9ds1_driver.h"
#include <cstdint>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdexcept>

class jetson_lsm9ds1_driver : public lsm9ds1_driver
{
public:
    jetson_lsm9ds1_driver();
    ~jetson_lsm9ds1_driver() override;

protected:
    void initialize_i2c(unsigned int i2c_bus, unsigned int i2c_addr_ag,
                        unsigned int i2c_addr_mag) override;
    void deinitialize_i2c() override;

    void write_ag_register(reg_ag addr, uint8_t val) override;
    uint8_t read_ag_register(reg_ag addr) override;
    void read_ag_registers(reg_ag addr, unsigned int n_bytes, char* buf) override;

    void write_m_register(reg_m addr, uint8_t val) override;
    uint8_t read_m_register(reg_m addr) override;
    void read_m_registers(reg_m addr, unsigned int n_bytes, char* buf) override;

private:
    int m_fd_ag;
    int m_fd_mag;
    int open_i2c(unsigned int bus, unsigned int addr);
    void close_i2c(int fd);
    void write_register(int fd, uint8_t reg, uint8_t val);
    void read_registers(int fd, uint8_t reg, unsigned int n_bytes, char* buf);
};

#endif // JETSON_LSM9DS1_DRIVER_H
