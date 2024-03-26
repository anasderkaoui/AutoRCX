# Setup the BNO055 IMU

- In order to use this IMU, make sure to connect it to the raspberry Pi4 just like the previous IMU. This type of connection is for I2C communication.
- In order to this IMU for another computer using the I2C to USB converter, make sure to connect RX->SDA, TX->SCL, Vin->Vcc, Gnd->Gnd and **connect the 3V pin to the PS1 pin**:
  <p align="center">
      <img src="https://github.com/MecaBotiX/m3cooper_ros_2/assets/115218309/fbe41cb9-2cef-43d6-9675-0626d6e7d087">
- [This is the ros2 driver](https://github.com/flynneva/bno055.git) for it.
- Make sure you modify [this line](https://github.com/flynneva/bno055/blob/45e1ff16936101711260c9fda63fbad99376ce3b/launch/bno055.launch.py#L38) in the launch file to "bno055_params_i2c.yaml".
- Also change the "i2c_bus" in [this line](https://github.com/flynneva/bno055/blob/45e1ff16936101711260c9fda63fbad99376ce3b/bno055/params/bno055_params_i2c.yaml#L34) of the param file to the corresponding one (usually 0 or 1).
- Install required dependencies: `rosdep install --from-paths src -i`
- Finally run: `ros2 launch bno055 bno055.launch.py`
