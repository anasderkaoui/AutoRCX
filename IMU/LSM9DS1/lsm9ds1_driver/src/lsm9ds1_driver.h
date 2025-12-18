#ifndef LSM9DS1_DRIVER_H
#define LSM9DS1_DRIVER_H

#include <functional>
#include <cstdint>

class lsm9ds1_driver
{
public:
    struct data {
        float accel_x, accel_y, accel_z;
        float gyro_x, gyro_y, gyro_z;
        float magneto_x, magneto_y, magneto_z;
        float temp;
    };

    lsm9ds1_driver();
    virtual ~lsm9ds1_driver() = 0;

    void set_data_callback(std::function<void(data)> cb);

    void initialize(unsigned int i2c_bus_ag, unsigned int i2c_addr_ag,
                    unsigned int i2c_addr_mag);
    void deinitialize();

    data read_data();

protected:
    // AG = Gyro/Accel; M = Mag
    enum class reg_ag : uint8_t {
        WHO_AM_I = 0x0F,
        CTRL_REG1_G = 0x10,
        CTRL_REG6_XL = 0x20,
        OUT_TEMP_L = 0x15,
        OUTX_L_G = 0x18,
        OUTX_L_XL = 0x28,
    };
    enum class reg_m : uint8_t {
        WHO_AM_I = 0x0F,
        CTRL_REG1_M = 0x20,
        CTRL_REG2_M = 0x21,
        CTRL_REG3_M = 0x22,
        CTRL_REG4_M = 0x23,
        OUTX_L_M = 0x28,
    };

    // PURE VIRTUALS for platform-specific I2C
    virtual void initialize_i2c(unsigned int i2c_bus_ag,
                                unsigned int i2c_addr_ag,
                                unsigned int i2c_addr_mag) = 0;
    virtual void deinitialize_i2c() = 0;

    virtual void write_ag_register(reg_ag addr, uint8_t value) = 0;
    virtual uint8_t read_ag_register(reg_ag addr) = 0;
    virtual void read_ag_registers(reg_ag addr, unsigned int n_bytes, char* buf) = 0;

    virtual void write_m_register(reg_m addr, uint8_t value) = 0;
    virtual uint8_t read_m_register(reg_m addr) = 0;
    virtual void read_m_registers(reg_m addr, unsigned int n_bytes, char* buf) = 0;

private:
    std::function<void(data)> m_data_callback;
};

#endif // LSM9DS1_DRIVER_H
