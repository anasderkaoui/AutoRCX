#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "cmd_vel_publisher");

    // Create a node handle
    ros::NodeHandle nh;

    // Create a publisher object
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Set the loop rate
    ros::Rate loop_rate(10);  // 10 Hz

    while (ros::ok()) {
        // Create a Twist message
        geometry_msgs::Twist cmd_vel;

        // Set linear and angular velocities
        cmd_vel.linear.x = 0.5;  // Adjust these values as needed
        cmd_vel.linear.y = 0.0;
        cmd_vel.linear.z = 0.0;
        cmd_vel.angular.x = 0.0;
        cmd_vel.angular.y = 0.0;
        cmd_vel.angular.z = 0.1;  // Adjust these values as needed

        // Publish the Twist message
        ROS_INFO("Publishing cmd_vel: linear_x = %f, angular_z = %f", cmd_vel.linear.x, cmd_vel.angular.z);
        cmd_vel_pub.publish(cmd_vel);

        // Sleep to maintain the loop rate
        loop_rate.sleep();
    }

    return 0;
}
