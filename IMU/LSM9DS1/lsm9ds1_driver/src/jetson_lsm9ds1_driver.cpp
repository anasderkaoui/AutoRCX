#include "jetson_lsm9ds1_driver.h"
#include <cstdio>

jetson_lsm9ds1_driver::jetson_lsm9ds1_driver() : m_fd_ag(-1), m_fd_mag(-1) {}
jetson_lsm9ds1_driver::~jetson_lsm9ds1_driver() { deinitialize_i2c(); }

int jetson_lsm9ds1_driver::open_i2c(unsigned int bus, unsigned int addr) {
    char path[16];
    snprintf(path, sizeof(path), "/dev/i2c-%u", bus);
    int fd = open(path, O_RDWR);
    if (fd < 0) throw std::runtime_error("Failed to open I2C device");
    if (ioctl(fd, I2C_SLAVE, addr) < 0) {
        close(fd);
        throw std::runtime_error("Failed to set I2C address");
    }
    return fd;
}
void jetson_lsm9ds1_driver::close_i2c(int fd) {
    if (fd >= 0) close(fd);
}
void jetson_lsm9ds1_driver::initialize_i2c(unsigned int bus, unsigned int addr_ag, unsigned int addr_mag) {
    m_fd_ag = open_i2c(bus, addr_ag);
    m_fd_mag = open_i2c(bus, addr_mag);
}
void jetson_lsm9ds1_driver::deinitialize_i2c() {
    if (m_fd_ag >= 0) close_i2c(m_fd_ag);
    if (m_fd_mag >= 0) close_i2c(m_fd_mag);
    m_fd_ag = m_fd_mag = -1;
}

void jetson_lsm9ds1_driver::write_register(int fd, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    if (write(fd, buf, 2) != 2) throw std::runtime_error("I2C write failed");
}
void jetson_lsm9ds1_driver::read_registers(int fd, uint8_t reg, unsigned int n_bytes, char* buf) {
    if (write(fd, &reg, 1) != 1) throw std::runtime_error("I2C reg select failed");
    if (read(fd, buf, n_bytes) != (ssize_t)n_bytes) throw std::runtime_error("I2C read failed");
}

// AG
void jetson_lsm9ds1_driver::write_ag_register(reg_ag addr, uint8_t val) {
    write_register(m_fd_ag, static_cast<uint8_t>(addr), val);
}
uint8_t jetson_lsm9ds1_driver::read_ag_register(reg_ag addr) {
    char b;
    read_registers(m_fd_ag, static_cast<uint8_t>(addr), 1, &b);
    return (uint8_t)b;
}
void jetson_lsm9ds1_driver::read_ag_registers(reg_ag addr, unsigned int n_bytes, char* buf) {
    read_registers(m_fd_ag, static_cast<uint8_t>(addr), n_bytes, buf);
}

// MAG
void jetson_lsm9ds1_driver::write_m_register(reg_m addr, uint8_t val) {
    write_register(m_fd_mag, static_cast<uint8_t>(addr), val);
}
uint8_t jetson_lsm9ds1_driver::read_m_register(reg_m addr) {
    char b;
    read_registers(m_fd_mag, static_cast<uint8_t>(addr), 1, &b);
    return (uint8_t)b;
}
void jetson_lsm9ds1_driver::read_m_registers(reg_m addr, unsigned int n_bytes, char* buf) {
    read_registers(m_fd_mag, static_cast<uint8_t>(addr), n_bytes, buf);
}
