from setuptools import find_packages, setup

package_name = 'lsm9ds1_imu_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','lsm9ds1','numpy', 'tf_transformations'],
    zip_safe=True,
    maintainer='protova2',
    maintainer_email='protova2@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'lsm9ds1_imu_node = lsm9ds1_imu_node.lsm9ds1_imu_node:main'
        ],
    },
)
