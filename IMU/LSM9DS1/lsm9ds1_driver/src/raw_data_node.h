#ifndef RAW_DATA_NODE_H
#define RAW_DATA_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include "jetson_lsm9ds1_driver.h"

class RawDataNode : public rclcpp::Node {
public:
    RawDataNode(unsigned int bus, unsigned int addr_ag, unsigned int addr_mag);

    ~RawDataNode();

private:
    void publish_imu();

    // Publishers for raw data
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr accel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr gyro_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr mag_pub_;

    // IMU driver
    jetson_lsm9ds1_driver imu_;

    // Timer for polling
    rclcpp::TimerBase::SharedPtr timer_;

    // Last sample (buffer)
    lsm9ds1_driver::data last_data_;
    std::mutex data_mutex_;
};

#endif // RAW_DATA_NODE_H
