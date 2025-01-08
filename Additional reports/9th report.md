Node (to be tested) to synchronize real robot motion to joint states:

```cpp
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <cmath>

// Robot dimensions (adjust to match your robot)
const double WHEELBASE = 0.254; // Distance between front and rear axles (meters)
const double TRACK_WIDTH = 0.167; // Distance between left and right wheels (meters)
const double WHEEL_RADIUS = 0.05; // Radius of the wheels (meters)

class AckermanSimulationNode {
public:
    AckermanSimulationNode() {
        // Initialize subscribers and publishers
        cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 10, &AckermanSimulationNode::cmdVelCallback, this);
        joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 10);

        // Initialize joint state message
        joint_state_msg_.name = {"front_left_wheel_joint", "front_right_wheel_joint", "rear_left_wheel_joint", "rear_right_wheel_joint", "front_left_steering_joint", "front_right_steering_joint"};
        joint_state_msg_.position.resize(6, 0.0);
        joint_state_msg_.velocity.resize(6, 0.0);
    }

    void spin() {
        ros::Rate rate(50); // 50 Hz
        while (ros::ok()) {
            ros::spinOnce();
            updateJointStates();
            joint_state_pub_.publish(joint_state_msg_);
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber cmd_vel_sub_;
    ros::Publisher joint_state_pub_;
    sensor_msgs::JointState joint_state_msg_;

    double linear_velocity_ = 0.0;
    double angular_velocity_ = 0.0;

    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        linear_velocity_ = msg->linear.x;
        angular_velocity_ = msg->angular.z;
    }

    void updateJointStates() {
        // Compute steering angles for Ackerman steering
        double delta_left = 0.0;
        double delta_right = 0.0;

        if (angular_velocity_ != 0.0) {
            double turning_radius = linear_velocity_ / angular_velocity_;
            delta_left = std::atan(WHEELBASE / (turning_radius - (TRACK_WIDTH / 2.0)));
            delta_right = std::atan(WHEELBASE / (turning_radius + (TRACK_WIDTH / 2.0)));
        }

        // Update wheel joint positions (steering)
        joint_state_msg_.position[4] = delta_left;  // front_left_steering_joint
        joint_state_msg_.position[5] = delta_right; // front_right_steering_joint

        // Compute wheel velocities
        double rear_wheel_velocity = linear_velocity_ / WHEEL_RADIUS;

        joint_state_msg_.velocity[0] = rear_wheel_velocity; // front_left_wheel_joint
        joint_state_msg_.velocity[1] = rear_wheel_velocity; // front_right_wheel_joint
        joint_state_msg_.velocity[2] = rear_wheel_velocity; // rear_left_wheel_joint
        joint_state_msg_.velocity[3] = rear_wheel_velocity; // rear_right_wheel_joint
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ackerman_simulation_node");
    AckermanSimulationNode node;
    node.spin();
    return 0;
}
```


