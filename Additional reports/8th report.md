## Odometry simulation:

```cpp
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <cmath>

class FakeOdomAckermann {
public:
    FakeOdomAckermann() {
        // Initialize ROS node
        cmd_vel_sub = nh.subscribe("/cmd_vel", 10, &FakeOdomAckermann::cmdVelCallback, this);
        odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 10);

        x = 0.0;
        y = 0.0;
        theta = 0.0;

        last_time = ros::Time::now();
    }

    void spin() {
        ros::Rate rate(10); // 10 Hz
        while (ros::ok()) {
            ros::Time current_time = ros::Time::now();
            double dt = (current_time - last_time).toSec();
            last_time = current_time;

            // Apply Ackermann kinematic equations
            double delta_x = linear_velocity * std::cos(theta) * dt;
            double delta_y = linear_velocity * std::sin(theta) * dt;
            double delta_theta = 0.0;

            if (steering_angle != 0.0) {
                double turning_radius = linear_velocity / std::tan(steering_angle);
                delta_theta = linear_velocity * dt / turning_radius;
            }

            x += delta_x;
            y += delta_y;
            theta += delta_theta;

            // Publish odometry message
            publishOdometry(current_time);

            // Publish TF transform
            publishTransform(current_time);

            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber cmd_vel_sub;
    ros::Publisher odom_pub;
    tf2_ros::TransformBroadcaster tf_broadcaster;

    double x, y, theta; // Robot's pose
    double linear_velocity = 0.0;
    double steering_angle = 0.0;

    ros::Time last_time;

    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        linear_velocity = msg->linear.x;  // Velocity of the car
        steering_angle = msg->angular.z; // Steering angle in radians
    }

    void publishOdometry(const ros::Time& current_time) {
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        // Pose
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation.z = std::sin(theta / 2.0);
        odom.pose.pose.orientation.w = std::cos(theta / 2.0);

        // Velocity
        odom.twist.twist.linear.x = linear_velocity;
        odom.twist.twist.angular.z = linear_velocity * std::tan(steering_angle);

        odom_pub.publish(odom);
    }

    void publishTransform(const ros::Time& current_time) {
        geometry_msgs::TransformStamped transform;
        transform.header.stamp = current_time;
        transform.header.frame_id = "odom";
        transform.child_frame_id = "base_link";

        transform.transform.translation.x = x;
        transform.transform.translation.y = y;
        transform.transform.translation.z = 0.0;
        transform.transform.rotation.z = std::sin(theta / 2.0);
        transform.transform.rotation.w = std::cos(theta / 2.0);

        tf_broadcaster.sendTransform(transform);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "fake_odom_ackermann");
    FakeOdomAckermann fake_odom;
    fake_odom.spin();
    return 0;
}
```
