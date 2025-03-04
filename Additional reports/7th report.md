In this report we will address some small issues that arise from time to time and can be solved quickly:

## First is binding USB port with a static name on Linux to a specific device. We're going to give the example of a LiDAR:

To bind a device to a specific port in Ubuntu 18.04 on your Jetson Nano, you can create a persistent udev rule that assigns a consistent device name based on its unique attributes (e.g., serial number, vendor ID, product ID). Here's how to do it:

  - <ins>Step 1</ins>: Open a terminal. List the serial devices on your system. You may figure out the serial number of your USB device by disconnecting and connecting it to your system.<br>
    `ls /dev/ttyUSB*`

  - <ins>Step 2</ins>: Check the bus number and port number of your USB device using: `dmesg | grep ttyUSB`<br>
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
    
    The USB device path of ttyUSB0 is **1-1.3.1**.

    Run the following command to list connected USB devices:
    udevadm info --query=all --name=/dev/ttyUSB0
    Replace /dev/ttyUSB0 with the actual device name if different.
    Look for unique attributes such as "ID_SERIAL", "ID_VENDOR", and "ID_PRODUCT".
    
  - <ins>Step 3</ins>: Edit the USB serial rules on your Linux system.<br>
    Now in my case (Using Jetson Nano) the rules start with 99 instead of 10 like in this example: `sudo nano /etc/udev/rules.d/10-usb-serial.rules`<br>

    To be sure, run `ls` when in `/etc/udev/rules.d/` and then look for a file where it says "usb-serial" in it. If you found one then modify it, and if not then create it:
    
    `sudo nano /etc/udev/rules.d/99-usb-serial.rules`

    Add this line at the end of the 99-usb-serial.rules. Change your-device-name to a static name you want to assign.
    
    `SUBSYSTEM=="tty", KERNELS=="1-1.3.1", SYMLINK+="your-device-name"`
    `SUBSYSTEM=="tty", ATTRS{idVendor}=="1234", ATTRS{idProduct}=="5678", SYMLINK+="my_device"`

    In my case, I wrote:
    
    `SUBSYSTEM=="tty", KERNELS=="1-1.3.1", SYMLINK+="USB-LIDAR"` <br>
    
    Press Ctrl + X, then confirm to save the file.
    
  - <ins>Step 4</ins>: Reload the udev rules to take effect. You may also reboot your system to update the udev rules.
    ```
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    ```

  - <ins>Step 5</ins>: Verify the changes. Unplug and reconnect your device.
    `ls -l /dev/ttyUSB*`<br>
    The result will look like
    ```
    crw-rw----+ 1 root dialout 188, 6 Jan 11 18:27 /dev/ttyUSB0
    lrwxrwxrwx 1 root root 7 Dec 17 14:17 /dev/USB-LIDAR-> ttyUSB0
    ```
    It should now appear as `/dev/my_device` (or whatever name you assigned in the SYMLINK).

## Now we will see how to run `sudo` commands automatically for only two specific commands when you open a new terminal window or tab without having to retype the password each time.<br>
> [!IMPORTANT]
> Having `sudo` commands in your `~/.bashrc` is not recommended, as it can cause unnecessary prompts and potential security issues. However, if you still want to avoid typing your password each time, you can configure **passwordless sudo** for the specific commands.

Here’s how:

1. **Edit the sudoers file:**
   Open the sudoers configuration safely using `visudo`:
   ```bash
   sudo visudo
   ```

2. **Add a rule for your user:**
   At the bottom of the file, add a line that allows your user to execute the specific commands without a password. Replace `your_username` with your actual username (you can know it by typing **`whoami`** in the terminal) and `/path/to/command` with the full path of the commands (you can check it by typing **`which chmod`** for e.g.) in your `.bashrc`.

   For example:<br>
   If you have multiple commands, separate them with commas:<br>
   ```bash
   your_username ALL=(ALL) NOPASSWD: /bin/chmod 777 /dev/USB-LIDAR, /bin/chmod 777 /dev/ttyUSB*
   ```
> [!Warning]
> It’s better to limit the permissions to only those commands and arguments you need (e.g., `/bin/chmod 777 /dev/USB-LIDAR` instead of just `/bin/chmod`).

3. **Update your `.bashrc`:**
   Modify your `.bashrc` to use `sudo` without a password:
   ```bash
   gedit ~/.bashrc
   sudo chmod 777 /dev/USB-LIDAR   # add your commands at the end of the file
   ```
   Now you won't be prompted to type in your password anymore.

### Another example but with bluetooth this time around. We will set it to turn on automatically at each startup.<br>
> [!Note]
> This is the safer way to do it, using a service instead of "bashrc" and "visudo" !

1. Create a systemd service file:<br>
`sudo nano /etc/systemd/system/unblock-bluetooth.service`

3. Add the following content:<br>
```bash
[Unit]
Description=Unblock Bluetooth at Startup
After=network.target

[Service]
Type=oneshot
ExecStart=/usr/sbin/rfkill unblock bluetooth
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
```

3. Enable and start the service:
```bash
sudo systemctl daemon-reload
sudo systemctl enable unblock-bluetooth.service
sudo systemctl start unblock-bluetooth.service
```
And there you have it, the bluetooth will be enabled at each startup of the card.
