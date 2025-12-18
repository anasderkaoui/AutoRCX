
# CALIBRATION PROCESS NEEDS TO BE RUN WHENEVER the IMU has been moved, or wires have been changed or components have been added near IMU, for BEST YAW ESTIMATION !!
# To recalibrate, set the MAG_OFFSETS to 0.0 and MAG_SCALES to 1.0 in the mpu9250sensor.cpp file !!
# To generate the mag.txt file, launch the mpu file and run in another terminal: `ros2 topic echo /imu/mag > mag.txt` while doing an 8-figure sweep in all axes!

#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# def parse_ros2_magnetic_data(file_path):
#     data = []
#     with open(file_path, 'r') as f:
#         x = y = z = None
#         for line in f:
#             if 'magnetic_field:' in line:
#                 x = y = z = None
#             elif 'x:' in line and x is None:
#                 x = float(line.split(':')[1])
#             elif 'y:' in line and y is None:
#                 y = float(line.split(':')[1])
#             elif 'z:' in line and z is None:
#                 z = float(line.split(':')[1])
#                 data.append([x, y, z])
#     return np.array(data)

def parse_ros2_magnetic_data(file_path):
    data = []
    inside_field = False
    x = y = z = None
    with open(file_path, 'r') as f:
        for line in f:
            line = line.strip()
            if line == 'magnetic_field:':
                inside_field = True
                x = y = z = None
            elif inside_field and line.startswith('x:'):
                x = float(line.split(':')[1])
            elif inside_field and line.startswith('y:'):
                y = float(line.split(':')[1])
            elif inside_field and line.startswith('z:'):
                z = float(line.split(':')[1])
                # Only append when x, y, z are all found in same block
                data.append([x, y, z])
                inside_field = False
    return np.array(data)

from scipy.optimize import leastsq
def fit_ellipsoid(data):
    x, y, z = data[:,0], data[:,1], data[:,2]
    offset_init = [np.mean(x), np.mean(y), np.mean(z)]
    scale_init = [np.std(x), np.std(y), np.std(z)]
    def residuals(params, x, y, z):
        ox, oy, oz, sx, sy, sz = params
        xn = (x - ox) / sx
        yn = (y - oy) / sy
        zn = (z - oz) / sz
        r = np.sqrt(xn**2 + yn**2 + zn**2)
        return r - 1
    params_init = offset_init + scale_init
    params_opt, _ = leastsq(residuals, params_init, args=(x, y, z))
    return params_opt

def main():
    data = parse_ros2_magnetic_data("mag.txt")

    # Sanity check
    print(data[:5])
    print("Number of points:", len(data))

    if len(data) < 1000:
        print("Not enough data collected!")
        return
    ox, oy, oz, sx, sy, sz = fit_ellipsoid(data)
    print("\n==== Magnetometer Calibration Results ====")
    print(f"Offset X: {ox:+.4f}   Y: {oy:+.4f}   Z: {oz:+.4f}")
    print(f"Scale  X: {sx:.4f}   Y: {sy:.4f}   Z: {sz:.4f}")
    print("Apply to code as:")
    print("  corrected_x = (raw_x - Offset_X) / Scale_X")
    print("  corrected_y = (raw_y - Offset_Y) / Scale_Y")
    print("  corrected_z = (raw_z - Offset_Z) / Scale_Z")

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(data[:,0], data[:,1], data[:,2], s=1, alpha=0.2)
    ax.set_xlabel('X (μT)')
    ax.set_ylabel('Y (μT)')
    ax.set_zlabel('Z (μT)')
    plt.savefig("magnetometer_scatter.png", dpi=200)
    print("Saved as magnetometer_scatter.png")
    plt.show()

if __name__ == "__main__":
    main()





# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import MagneticField
# import numpy as np
# import time

# class MagCalibrationNode(Node):
#     def __init__(self):
#         super().__init__('mag_calibration_node')
#         self.subscription = self.create_subscription(
#             MagneticField,
#             '/imu/mag',      # Change if your topic is different!
#             self.mag_callback,
#             10)
#         self.mags = []
#         self.start_time = time.time()
#         self.max_samples = 2000  # Set higher for better accuracy

#     def mag_callback(self, msg):
#         mx = msg.magnetic_field.x * 1e6  # Convert from Tesla to μT
#         my = msg.magnetic_field.y * 1e6
#         mz = msg.magnetic_field.z * 1e6
#         self.mags.append([mx, my, mz])

#         # Print every 100 samples
#         if len(self.mags) % 100 == 0:
#             self.get_logger().info(f'Samples collected: {len(self.mags)}')

#         if len(self.mags) >= self.max_samples:
#             self.get_logger().info('Max samples reached, computing calibration...')
#             self.calibrate()
#             rclpy.shutdown()

#     def calibrate(self):
#         mags_np = np.array(self.mags)
#         min_vals = mags_np.min(axis=0)
#         max_vals = mags_np.max(axis=0)
#         offsets = (max_vals + min_vals) / 2
#         scales = (max_vals - min_vals) / 2

#         print("\n==== Magnetometer Calibration Results ====")
#         print(f"Offset X: {offsets[0]:.4f}   Y: {offsets[1]:.4f}   Z: {offsets[2]:.4f}")
#         print(f"Scale  X: {scales[0]:.4f}   Y: {scales[1]:.4f}   Z: {scales[2]:.4f}\n")
#         print("Apply to code as:")
#         print("  corrected_x = (raw_x - Offset_X) / Scale_X")
#         print("  corrected_y = (raw_y - Offset_Y) / Scale_Y")
#         print("  corrected_z = (raw_z - Offset_Z) / Scale_Z")

# def main(args=None):
#     rclpy.init(args=args)
#     node = MagCalibrationNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         print('Interrupted')
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()


# # ==== Magnetometer Calibration Results ====
# # Offset X: -1463459.5894   Y: -1394056.5585   Z: 47811.9082
# # Scale  X: 890646.3509   Y: 1241560.6385   Z: 472674.3599