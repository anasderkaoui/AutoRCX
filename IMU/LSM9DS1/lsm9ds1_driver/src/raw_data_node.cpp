#include "raw_data_node.h"

RawDataNode::RawDataNode(unsigned int bus, unsigned int addr_ag, unsigned int addr_mag)
    : Node("raw_data_node")
{
    // Create publishers
    accel_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("/raw_imu_data/accel", 10);
    gyro_pub_  = this->create_publisher<geometry_msgs::msg::Vector3>("/raw_imu_data/gyro",  10);
    mag_pub_   = this->create_publisher<geometry_msgs::msg::Vector3>("/raw_imu_data/mag",   10);

    // Initialize IMU
    try {
        imu_.initialize(bus, addr_ag, addr_mag);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(this->get_logger(), "Failed to initialize IMU: %s", e.what());
        throw;
    }

    // Set callback for incoming IMU data
    imu_.set_data_callback([this](lsm9ds1_driver::data d){
        std::lock_guard<std::mutex> lock(data_mutex_);
        last_data_ = d;
    });

    // Poll IMU at 100Hz
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&RawDataNode::publish_imu, this));
}

RawDataNode::~RawDataNode() {
    imu_.deinitialize();
}

void RawDataNode::publish_imu()
{
    imu_.read_data();

    lsm9ds1_driver::data d;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        d = last_data_;
    }

    // Offset (bias): [-0.02419209  0.00225898 -0.01183639]
    // Scale (gain): [0.1019021  0.10232313 0.10242377]
    // To apply: corrected = (raw - bias) / gain
    // Publish accelerometer
    geometry_msgs::msg::Vector3 accel;
    accel.x = ((d.accel_x / 10) + 0.02419209) / 0.1019021;
    accel.y = ((d.accel_y / 10) - 0.00225898) / 0.10232313;
    accel.z = ((d.accel_z / 10) + 0.01183639) / 0.10242377;
    
    // accel.x = d.accel_x ;
    // accel.y = d.accel_y ;
    // accel.z = d.accel_z ;
    
    
    accel_pub_->publish(accel);

    // Publish gyroscope


    // bias = np.array([-0.40867749962210653, 1.0843700016736983, 0.16917250027135014])

    geometry_msgs::msg::Vector3 gyro;
    gyro.x = d.gyro_x * 100 + 0.40867749962210653;
    gyro.y = d.gyro_y * 100 - 1.0843700016736983;
    gyro.z = d.gyro_z * 100 - 0.16917250027135014;
    
    
    // gyro.x = d.gyro_x;
    // gyro.y = d.gyro_y;
    // gyro.z = d.gyro_z;
    gyro_pub_->publish(gyro);

    // Publish magnetometer
    geometry_msgs::msg::Vector3 mag_raw;
    // mag_raw.x = d.magneto_x * 10;
    // mag_raw.y = d.magneto_y * 10;
    // mag_raw.z = d.magneto_z * 10;
    
// // Calibration: offsets and soft iron compensation matrix
// const float offset_x = -5.68;
// const float offset_y = -13.35;
// const float offset_z = -110.60;

// const float soft_iron[3][3] = {
//   { 0.989, 0.033, 0.008 },
//   { 0.033, 0.963, 0.025 },
//   { 0.008, 0.025, 1.052 }
// };


    // Hard iron correction
    float mx = mag_raw.x - 9.48f;
    float my = mag_raw.y + 13.34f;
    float mz = mag_raw.z + 72.75f;

    // // Soft iron correction
    geometry_msgs::msg::Vector3 mag;
    mag.x = 1.013f * mx + 0.039f * my + 0.022f * mz;
    mag.y = 0.039f * mx + 0.956f * my + 0.063f * mz;
    mag.z = 0.022f * mx + 0.063f * my + 1.039f * mz;

    mag.x = d.magneto_x;
    mag.y = d.magneto_y;
    mag.z = d.magneto_z;

    mag_pub_->publish(mag);

}


#include "raw_data_node.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Adjust as needed (bus, AG addr, MAG addr)
    auto node = std::make_shared<RawDataNode>(1, 0x6B, 0x1E);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
