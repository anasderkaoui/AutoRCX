# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration

# import os


# def generate_launch_description():
#     ld = LaunchDescription()
#     share_dir = get_package_share_directory('mpu9250driver')
#     parameter_file = LaunchConfiguration('params_file')

#     params_declare = DeclareLaunchArgument('params_file',
#                                            default_value=os.path.join(
#                                                share_dir, 'params', 'mpu9250.yaml'),
#                                            description='Path to the ROS2 parameters file to use.')

#     mpu9250driver_node = Node(
#         package='mpu9250driver',
#         executable='mpu9250driver',
#         name='mpu9250driver_node',
#         output="screen",
#         emulate_tty=True,
#         parameters=[parameter_file]
#     )

#     ld.add_action(params_declare)
#     ld.add_action(mpu9250driver_node)
#     return ld


from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    ld = LaunchDescription()
    share_dir = get_package_share_directory('mpu9250driver')
    parameter_file = LaunchConfiguration('params_file')

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(share_dir, 'params', 'mpu9250.yaml'),
        description='Path to the ROS2 parameters file to use.'
    )

    mpu9250driver_node = Node(
        package='mpu9250driver',
        executable='mpu9250driver',
        name='mpu9250driver_node',
        output="screen",
        emulate_tty=True,
        parameters=[parameter_file]
    )

    # Madgwick filter node (fuses /imu/data_raw + /imu/mag → /imu/data_filtered)
    madgwick_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='madgwick_filter',
        parameters=[{
            'frequency': 100.0,
            # 'zeta': 0.1, # gyro gain I think
            'gain': 0.9,          # <-- try higher for faster initial stabilization
            'use_mag': True,       # (if available, keep True)
            'mag_bias_x': 0.0,  
            'mag_bias_y': 0.0,
            'mag_bias_z': 0.0,
            'world_frame': "nwu",  # north-west-up, like on the IMU
            'publish_tf': True
        }],
        remappings=[ # They don't work !
            ('imu', '/imu/data_raw'),
            ('mag', '/imu/mag'),
            ('imu_out', '/imu/data_filtered')
        ],
        output='screen'
    )

    # Euler node (reads from the Madgwick output!)
    # imu_euler_node = Node(
    #     package='mpu9250driver',
    #     executable='imu_euler_node',
    #     name='imu_euler_node',
    #     output='screen',
    #     # Only if your node supports remapping input topic!
    #     # remappings=[('/imu/data', '/imu/data_filtered')],
    # )

    ld.add_action(params_declare)
    ld.add_action(mpu9250driver_node)
    ld.add_action(madgwick_node)
    # ld.add_action(imu_euler_node)

    return ld
