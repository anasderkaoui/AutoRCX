import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
import numpy as np
import tf_transformations
import lsm9ds1
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

# === Your calibration values from the script ===
# MAG_X_MIN = -0.46522
# MAG_X_MAX = 0.30296
# MAG_Y_MIN = -0.11802000000000001
# MAG_Y_MAX = 0.7109200000000001
# HEADING_OFFSET = 163.1743014252076  # degrees
OFFSET_X = -12.8400
OFFSET_Y = 30.3396
OFFSET_Z = -43.0400
SCALE_X = 40.0046
SCALE_Y = 40.6531
SCALE_Z = 37.2018

def tilt_compensated_yaw(ax, ay, az, mx, my, mz):
    # Normalize accelerometer
    norm_acc = np.linalg.norm([ax, ay, az])
    if norm_acc == 0:
        return 0.0
    axn, ayn, azn = ax/norm_acc, ay/norm_acc, az/norm_acc

    # Pitch and roll from accel
    roll = np.arctan2(ayn, azn)
    pitch = np.arctan2(-axn, np.sqrt(ayn*ayn + azn*azn))

    # Tilt compensation of mag
    mx2 = mx * np.cos(pitch) + mz * np.sin(pitch)
    my2 = mx * np.sin(roll) * np.sin(pitch) + my * np.cos(roll) - mz * np.sin(roll) * np.cos(pitch)
    heading = np.arctan2(-my2, mx2)  # right-hand, so -my2

    # Convert to degrees and apply heading offset
    # heading_deg = np.degrees(heading) + HEADING_OFFSET
    heading_deg = np.degrees(heading)
    heading_deg = (heading_deg + 360.0) % 360.0
    return np.radians(heading_deg)  # in radians for quaternion

class LSM9DS1ImuNode(Node):
    def __init__(self):
        super().__init__('lsm9ds1_imu_node')
        self.pub_imu = self.create_publisher(Imu, 'imu/data_lsm', 10)
        self.pub_mag = self.create_publisher(MagneticField, 'imu/mag_lsm', 10)
        self.timer = self.create_timer(0.02, self.publish_imu)  # 50 Hz

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Setup IMU
        self.imu = lsm9ds1.make_i2c(1)

    def publish_imu(self):
        # Read raw data
        ax_raw, ay_raw, az_raw = self.imu.read_acceleration()
        gx_raw, gy_raw, gz_raw = self.imu.read_gyroscope()
        mx_gauss, my_gauss, mz_gauss = self.imu.mag_values()

        # Scale (you can skip scaling if you just want orientation, but it's best practice)
        ACC_SENSOR_SCALE = lsm9ds1.lsm9ds1.ACC_SENSOR_SCALE * 9.80665
        DPS_SENSOR_SCALE = lsm9ds1.lsm9ds1.DPS_SENSOR_SCALE * (np.pi/180)

        ax = float(ax_raw * ACC_SENSOR_SCALE)
        ay = float(ay_raw * ACC_SENSOR_SCALE)
        az = float(az_raw * ACC_SENSOR_SCALE)
        gx = float(gx_raw * DPS_SENSOR_SCALE)
        gy = float(gy_raw * DPS_SENSOR_SCALE)
        gz = float(gz_raw * DPS_SENSOR_SCALE)

        # --- Convert gauss to microtesla ---
        mx_ut = mx_gauss * 100.0
        my_ut = my_gauss * 100.0
        mz_ut = mz_gauss * 100.0

        # --- Calibration (μT) ---
        mx_corr = (mx_ut - OFFSET_X) / SCALE_X
        my_corr = (my_ut - OFFSET_Y) / SCALE_Y
        mz_corr = (mz_ut - OFFSET_Z) / SCALE_Z

        # --- Roll and Pitch from accelerometer (in radians) ---
        roll = np.arctan2(ay, az)
        pitch = np.arctan2(-ax, np.sqrt(ay**2 + az**2))

        # --- Yaw calculation: use corrected (Tesla) values ---
        # yaw_rad = tilt_compensated_yaw(ax, ay, az, mx_corr, my_corr, mz_corr)
        yaw_rad = math.atan2(my_corr, mx_corr)
        # yaw_deg = np.rad2deg(yaw_rad)
        yaw_deg = (np.degrees(yaw_rad) + 360) % 360

        # Raw, uncalibrated yaw for comparison (using Gauss, not recommended)
        yaw_uncal = math.atan2(my_gauss, mx_gauss)
        yaw_uncal_deg = (math.degrees(yaw_uncal) + 360) % 360  

        # --- Orientation as quaternion ---
        quat = tf_transformations.quaternion_from_euler(roll, pitch, yaw_rad)

        # --- Fill and publish IMU message ---
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        msg.linear_acceleration.x = float(ax)
        msg.linear_acceleration.y = float(ay)
        msg.linear_acceleration.z = float(az)
        msg.angular_velocity.x = float(gx)
        msg.angular_velocity.y = float(gy)
        msg.angular_velocity.z = float(gz)
        msg.orientation.x = quat[0]
        msg.orientation.y = quat[1]
        msg.orientation.z = quat[2]
        msg.orientation.w = quat[3]
        self.pub_imu.publish(msg)

        # --- Publish corrected magnetometer values (Tesla) ---
        mag_msg = MagneticField()
        mag_msg.header = msg.header
        # mag_msg.magnetic_field.x = float(mx_corr / 1e6) # μT → T
        # mag_msg.magnetic_field.y = float(my_corr / 1e6)
        # mag_msg.magnetic_field.z = float(mz_corr / 1e6)
        # μT
        mag_msg.magnetic_field.x = float(mx_corr) # μT
        mag_msg.magnetic_field.y = float(my_corr)
        mag_msg.magnetic_field.z = float(mz_corr)
        self.pub_mag.publish(mag_msg)

        self.get_logger().info(
            f"Raw yaw: {yaw_uncal_deg:.1f}°, Yaw: {yaw_deg:.1f}°, Roll: {np.rad2deg(roll):.1f}°, Pitch: {np.rad2deg(pitch):.1f}°"
        )

        # --- TF publishing ---
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "base_link"      # <--- Set your parent frame here!
        t.child_frame_id = "imu_link"        # <--- Set your child frame here!
        t.transform.translation.x = 0.0      # Change if your IMU is offset!
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = LSM9DS1ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
