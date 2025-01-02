In this report we will address some small issues that arise from time to time and can be solved quickly:

- First is binding USB port with a static name on Linux to a specific device. We're going to give example of a LiDAR:
  - Step 1: Open a terminal. List the serial devices on your system. You may figure out the serial number of your USB device by disconnecting and connecting it to your system.
    `ls /dev/ttyUSB*`

  - Step 2: Check the bus number and port number of your USB device using: `dmesg | grep ttyUSB`<br>
    It will show something like:<br>
    `output:[ 5.290556] usb 1-1.3.1: ch341-uart converter now attached to ttyUSB0`
    In my case, the USB device path is 1-1.3.1. Jot the path down, and we will use this path later. If it shows nothing, you may try:<br>
    `sudo apt install udev -y`<br>
    Then by running<br>
    
    `udevadm info -a -n /dev/ttyUSB0`
    
    It will show something like<br>
    
     ```
     looking at device '/devices/platform/scb/fd500000.pcie/pci0000:00/0000:00:00.0/0000:01:00.0/usb1/1-1/1-1.3/1-1.3.1/1-1.3.1:1.0/ttyUSB0/tty/ttyUSB0':
      KERNEL=="ttyUSB0"
      SUBSYSTEM=="tty"
      DRIVER==""
     ```
    
    We will get an identical result. The USB device path of ttyUSB0 is 1-1.3.1.
    
  - Step 3: Edit the USB serial rules on your Linux system.<br>
    Now in my case (Using Jetson Nano) the rules start with 99 instead of 10 like in the example below:
    
    `sudo nano /etc/udev/rules.d/10-usb-serial.rules`

    To be sure, run `ls` when in `/etc/udev/rules.d/` and then look fo ra file where it says "usb-serial" in it. If you did not find one then create it:
    
    `sudo nano /etc/udev/rules.d/99-usb-serial.rules`

    Add this line at the end of the 10-usb-serial.rules. Change your-device-name to a static name you want to assign.
    
    `SUBSYSTEM=="tty", KERNELS=="1-1.3.1", SYMLINK+="your-device-name"`

    In my case, I wrote:
    
    `SUBSYSTEM=="tty", KERNELS=="1-1.3.1", SYMLINK+="USB-LIDAR"` <br>
    
    Press Ctrl + X, then confirm to save the file.
    
  - Step 4: Reload the udev rules to take effect. You may also reboot your system to update the udev rules.
    ```
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    ```

  - Step 5: Verify the changes.
    `ls -l /dev/ttyUSB*`<br>
    The result will look like
    ```
    crw-rw----+ 1 root dialout 188, 6 Jan 11 18:27 /dev/ttyUSB0
    lrwxrwxrwx 1 root root 7 Dec 17 14:17 /dev/USB-LIDAR-> ttyUSB0
    ```
    Congrats, you have already bound a static name to a USB port.

Now we will see how to run `sudo` commands automatically for only two specific commands when you open a new terminal window or tab without having to retype the password each time.
