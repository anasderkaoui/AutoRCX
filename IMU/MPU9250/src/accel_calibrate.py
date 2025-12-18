#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np

GRAVITY = 9.81  # m/s^2

class AccelCalibNode(Node):
    def __init__(self):
        super().__init__('accel_calib_node')
        self.subscription = self.create_subscription(Imu, '/imu/data_raw', self.imu_callback, 10)
        self.samples = []
        self.current_face = 0
        self.faces = [
            ("+X up",  np.array([+GRAVITY, 0, 0])),
            ("-X up",  np.array([-GRAVITY, 0, 0])),
            ("+Y up",  np.array([0, +GRAVITY, 0])),
            ("-Y up",  np.array([0, -GRAVITY, 0])),
            ("+Z up",  np.array([0, 0, +GRAVITY])),
            ("-Z up",  np.array([0, 0, -GRAVITY])),
        ]
        self.samples_per_face = 500  # Or higher for better accuracy
        self.face_samples = []
        print("\n=== Accelerometer 6-position Calibration ===")
        print("For each prompt, position the IMU so that the axis points UP as indicated, then press ENTER.")
        self.next_face()

    def next_face(self):
        if self.current_face >= len(self.faces):
            self.compute_calibration()
            rclpy.shutdown()
            return
        name, _ = self.faces[self.current_face]
        input(f"\nPlace the IMU with {name}, then press ENTER...")
        self.face_samples = []
        print(f"Collecting {self.samples_per_face} samples...")

    def imu_callback(self, msg):
        if self.current_face >= len(self.faces):
            return
        acc = np.array([msg.linear_acceleration.x,
                        msg.linear_acceleration.y,
                        msg.linear_acceleration.z])
        self.face_samples.append(acc)
        if len(self.face_samples) >= self.samples_per_face:
            avg = np.mean(self.face_samples, axis=0)
            self.samples.append(avg)
            self.current_face += 1
            self.next_face()

    # def compute_calibration(self):
    #     print("\nCalibration complete!\n")
    #     samples = np.array(self.samples)
    #     ref = np.array([v for _, v in self.faces])
    #     # Solve for scale and bias: samples = scale * ref + bias
    #     # Estimate scale: (measured_plus - measured_minus) / (2*g)
    #     scale = (samples[::2] - samples[1::2]) / (2 * GRAVITY)
    #     scale = np.mean(np.abs(scale), axis=0)
    #     # Estimate bias: mean of (measured_plus + measured_minus) / 2
    #     bias = (samples[::2] + samples[1::2]) / 2
    #     bias = np.mean(bias, axis=0)

    #     print(f"Offsets (bias): x={bias[0]:.5f}, y={bias[1]:.5f}, z={bias[2]:.5f}")
    #     print(f"Scales: x={scale[0]:.5f}, y={scale[1]:.5f}, z={scale[2]:.5f}")
    #     print("\nApply in your code as:")
    #     print("  calibrated = (raw - bias) / scale\n")

    def compute_calibration(self):
        print("\nCalibration complete!\n")
        samples = np.array(self.samples)
        scale = np.zeros(3)
        bias = np.zeros(3)
        for i in range(3):
            plus = samples[2 * i]
            minus = samples[2 * i + 1]
            scale[i] = (plus[i] - minus[i]) / (2 * GRAVITY)
            bias[i] = (plus[i] + minus[i]) / 2
        print(f"Offsets (bias): x={bias[0]:.5f}, y={bias[1]:.5f}, z={bias[2]:.5f}")
        print(f"Scales: x={scale[0]:.5f}, y={scale[1]:.5f}, z={scale[2]:.5f}")
        print("\nApply in your code as:")
        print("  calibrated = (raw - bias) / scale\n")


def main(args=None):
    rclpy.init(args=args)
    node = AccelCalibNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
