#include "jetson_lsm9ds1_driver.h"
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <thread>
#include <chrono>

// You may need to tweak these based on your sensor settings!
constexpr float ACCEL_SENSITIVITY = 0.000061; // for ±2g: 0.061 mg/LSB = 0.000061 g/LSB
constexpr float GYRO_SENSITIVITY = 0.00875;   // for ±245 dps: 8.75 mdps/LSB = 0.00875 dps/LSB
constexpr float MAG_SENSITIVITY = 0.14;       // for ±4 gauss: 0.14 mgauss/LSB = 0.014 gauss/LSB

// Utility to open UART
int open_uart(const char* device, int baudrate = B115200) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        perror("open_uart: Unable to open UART device");
        exit(EXIT_FAILURE);
    }

    struct termios tty {};
    if (tcgetattr(fd, &tty) != 0) {
        perror("open_uart: tcgetattr failed");
        exit(EXIT_FAILURE);
    }

    cfsetospeed(&tty, baudrate);
    cfsetispeed(&tty, baudrate);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;    // 8-bit chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo, no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);       // ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD);     // no parity
    tty.c_cflag &= ~CSTOPB;                // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;               // no flow control

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("open_uart: tcsetattr failed");
        exit(EXIT_FAILURE);
    }

    return fd;
}

int main() {
    // === 1. Initialize IMU (update bus and addresses as needed) ===
    jetson_lsm9ds1_driver imu;
    imu.initialize_i2c(1, 0x6b, 0x1e);

    // === 2. Open UART ===
    int uart_fd = open_uart("/dev/ttyTHS0", B115200);

    // === 3. Main Loop ===
    while (true) {
        int16_t accel_raw[3] = {0};
        int16_t gyro_raw[3] = {0};
        int16_t mag_raw[3] = {0};

        imu.read_accel_raw(accel_raw);
        imu.read_gyro_raw(gyro_raw);
        imu.read_mag_raw(mag_raw);

        // Convert raw to SI units (based on your driver doc/settings)
        float ax = accel_raw[0] * ACCEL_SENSITIVITY * 9.8; // m/s^2
        float ay = accel_raw[1] * ACCEL_SENSITIVITY * 9.8;
        float az = accel_raw[2] * ACCEL_SENSITIVITY * 9.8;

        float gx = gyro_raw[0] * GYRO_SENSITIVITY * 3.1415926f / 180.0f; // rad/s
        float gy = gyro_raw[1] * GYRO_SENSITIVITY * 3.1415926f / 180.0f;
        float gz = gyro_raw[2] * GYRO_SENSITIVITY * 3.1415926f / 180.0f;

        float mx = mag_raw[0] * MAG_SENSITIVITY * 0.1; // microtesla
        float my = mag_raw[1] * MAG_SENSITIVITY * 0.1;
        float mz = mag_raw[2] * MAG_SENSITIVITY * 0.1;

        // Output in MotionCal's expected format:
        char line[128];
        snprintf(line, sizeof(line),
            "Raw:%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
            int(ax * 8192.0 / 9.8),
            int(ay * 8192.0 / 9.8),
            int(az * 8192.0 / 9.8),
            int(gx * 57.2958 * 16.0),
            int(gy * 57.2958 * 16.0),
            int(gz * 57.2958 * 16.0),
            int(mx * 10.0),
            int(my * 10.0),
            int(mz * 10.0)
        );

        // Send over UART
        write(uart_fd, line, strlen(line));
        // Debug
        std::cout << line;

        std::this_thread::sleep_for(std::chrono::milliseconds(50)); // ~20 Hz
    }

    imu.deinitialize_i2c();
    close(uart_fd);
    return 0;
}
