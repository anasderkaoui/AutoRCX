# Setup the BNO055 IMU

## Hardware part:

- To use this IMU with a raspberry pi4, make sure to connect it just like [the previous IMU](https://github.com/anasderkaoui/AutoRCX/tree/main/SLAM). This type of connection is for **I2C communication**.
- In order to use this IMU with a computer using **USB (UART communication)**, you will need the I2C to USB converter like [the one in the following image](https://www.amazon.fr/DSD-TECH-Adaptateur-FT232RL-Compatible/dp/B07BBPX8B8?source=ps-sl-shoppingads-lpcontext&ref_=fplfs&psc=1&smid=A1SUSVKN2N7NX6):
  <p align="center">
      <img src="https://github.com/anasderkaoui/AutoRCX/assets/115218309/c2cf3ae7-0289-4dc6-82d9-1b4b7736cd48">
- Make sure to connect RX ➜ SDA, TX ➜ SCL, Vin ➜ Vcc, Gnd ➜ Gnd and **connect the 3V pin to the PS1 pin, both present on the IMU itself**:
  <p align="center">
      <img src="https://github.com/MecaBotiX/m3cooper_ros_2/assets/115218309/fbe41cb9-2cef-43d6-9675-0626d6e7d087">

## Software part:

- [This is the ros2 driver](https://github.com/flynneva/bno055.git) for it.
- Make sure you modify [this line](https://github.com/flynneva/bno055/blob/45e1ff16936101711260c9fda63fbad99376ce3b/launch/bno055.launch.py#L38) in the launch file to "bno055_params_i2c.yaml" if you are using the IMU in I2C mode.
- Also change the "i2c_bus" in [this line](https://github.com/flynneva/bno055/blob/45e1ff16936101711260c9fda63fbad99376ce3b/bno055/params/bno055_params_i2c.yaml#L34) of the param file to the corresponding one (usually 0 or 1).
- Install required dependencies: `rosdep install --from-paths src -i`
- Finally run: `ros2 launch bno055 bno055.launch.py`
