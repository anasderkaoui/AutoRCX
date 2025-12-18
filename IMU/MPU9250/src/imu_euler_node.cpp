#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

double normalize_angle_360(double angle_deg) {
    // Convert angle in degrees to 0–360°
    while (angle_deg < 0) angle_deg += 360.0;
    while (angle_deg >= 360.0) angle_deg -= 360.0;
    return angle_deg;
}

class ImuEulerNode : public rclcpp::Node {
public:
    ImuEulerNode() : Node("imu_euler_node") {
        subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10,
            std::bind(&ImuEulerNode::imu_callback, this, std::placeholders::_1));
    }
private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w
        );
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        // RCLCPP_INFO(this->get_logger(),
        //             "Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°",
        //             roll * 180.0 / M_PI,
        //             pitch * 180.0 / M_PI,
        //             yaw * 180.0 / M_PI);
        double roll_deg = roll * 180.0 / M_PI;
        double pitch_deg = pitch * 180.0 / M_PI;
        double yaw_deg = yaw * 180.0 / M_PI;

        // Normalize yaw to 0–360
        yaw_deg = normalize_angle_360(yaw_deg);
        pitch_deg = normalize_angle_360(pitch_deg);
        roll_deg = normalize_angle_360(roll_deg);
        // (Optional) also normalize roll/pitch if you want 0–360

        RCLCPP_INFO(this->get_logger(),
            "Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°",
            roll_deg, pitch_deg, yaw_deg);

    }
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuEulerNode>());
    rclcpp::shutdown();
    return 0;
}
