# Setup the BNO055 IMU

## Hardware part:

- To use this IMU with a raspberry pi4, make sure to connect it just like [the previous IMU](https://github.com/anasderkaoui/AutoRCX/tree/main/SLAM). This type of connection is for **I2C communication**.
- In order to use this IMU with a computer using **USB (UART communication)**, you will need the I2C to USB converter like [the one in the following image](https://www.amazon.fr/DSD-TECH-SH-U09C2-Adaptateur-Programmation/dp/B07TXVRQ7V/ref=sxin_16_sbv_search_btf?adgrpid=1362295842745133&content-id=amzn1.sym.219f4781-dc21-4302-8e9f-a81a032eb50a%3Aamzn1.sym.219f4781-dc21-4302-8e9f-a81a032eb50a&cv_ct_cx=usb+ttl&dib=eyJ2IjoiMSJ9.LbUssafjgUd5X0Orfpi1iQ.FC507ci5W5Ll5fv1h7Rh44FFbiLzPBKRibP0xb7Ix2Q&dib_tag=se&hvadid=85143748760118&hvbmt=be&hvdev=c&hvlocphy=126145&hvnetw=o&hvqmt=e&hvtargid=kwd-85144014752699%3Aloc-66&hydadcr=4667_1852455&keywords=usb+ttl&msclkid=39b7b8e1eef018249943dfe1061401a7&pd_rd_i=B07TXVRQ7V&pd_rd_r=9d61665e-c8ab-4524-b57c-44289332e6e0&pd_rd_w=FiVKJ&pd_rd_wg=76Awx&pf_rd_p=219f4781-dc21-4302-8e9f-a81a032eb50a&pf_rd_r=2FG3RWG5KVEFB059EQ94&qid=1728112878&sbo=RZvfv%2F%2FHxDF%2BO5021pAnSA%3D%3D&sr=1-1-07652b71-81e3-41f8-9097-e46726928fb7) ([backup link](https://www.iot-store.com.au/collections/iot-networking-comms/products/industrial-usb-to-ttl-converter-ft232rl)):
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
