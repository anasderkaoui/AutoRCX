In this session we are going to learn how to boot the Jetson Nano from an M.2 M key SSD. As you know, the eMMC of the Jetson Nano only has 15Gb of storage available, which is very tiny for developping. In order to have more space on the Jetson Nano, I will be equipping it with a 500Gb SSD. This guide describes how to set up your system to boot from an M.2 SSD.<br>

This guide is intended for experienced users only.

First step : Insert the SSD in the M.2 slot on the Jetson Nano and screw it in place.<br>

Second step : Now comes the crucial part !

**WARNING : FOLLOW EACH INSTRUCTION AS SHOWN OR THE JETSON NANO WILL BE BRICKED AND WILL REQUIRE [REFLASHING](https://github.com/anasderkaoui/AutoRCX/blob/main/Additional%20reports/1st%20report.md) !!** <br>

- Locate and identify storage device :<br>
  The first thing you need to do is to identify the storage device you are intending to use.

![image](https://github.com/anasderkaoui/AutoRCX/assets/115218309/b924526d-8872-4430-a444-77afc24b8f00)

Search "disk" in the search bar and you can find your device as shown in the Picture. In the example above: /dev/sdb1

**NOTE : You will have to format your SSD in ext4 format, make sure it does not hold important data !** <br>

When working with NVME SSD your device name should look like this: /dev/nvme0n1p1<br>
The ending "p1" stands for partition one. In the following steps **you must remove the partition information from the device path**. See the following example:<br>
**<YOUR_STORAGE_DEVICE> = /dev/nvme0n1**

- Set up RootFS on SSD :<br>
  We will now move the RootFS, also known as the Root Filesystem ("/"), to the SSD. Follow the steps bellow :<br>
  1. Format the storage device : `sudo parted <YOUR_STORAGE_DEVICE> mklabel gpt`
  2. Create the RootFS partition : `sudo parted <YOUR_STORAGE_DEVICE> mkpart APP 0GB <YOUR_ROOTFS_SIZE>`
  3. Create filesystem : `sudo mkfs.ext4 <YOUR_STORAGE_DEVICE>`
  4. Copy the existing RootFS to the storage device :<br>
     `sudo mount <YOUR_STORAGE_DEVICE> /mnt`<br>
     `sudo rsync -axHAWX --numeric-ids --info=progress2 --`<br>
     `exclude={"/dev/","/proc/","/sys/","/tmp/","/run/","/mnt/","/media/*","/lost+found"} / /mnt/`<br>
     
- Switch boot device to SSD
You need to change your root target in exlinux.conf to the SSD. This is necessary so that the Operating system knows where to find the system files.<br>
Open exlinux.conf to change the root path: `sudo nano /boot/exlinux/exlinux.conf`<br>
Modify the following line : APPEND ${cbootargs} quiet root=**<YOUR_STORAGE_DEVICE>** rw rootwait rootfstype=ext4 console=ttyTCU0,115200n8 console=tty0 fbcon=map:0 net.ifnames=0

- After a reboot your system will start from the SSD !<br>
  
- You can validate the SSD boot by typing : `df /`
![image](https://github.com/anasderkaoui/AutoRCX/assets/115218309/200a7fda-b2ed-43d3-a9d4-2480c9b44f43)<br>
You should see : **<YOUR_STORAGE_DEVICE>** Mounted on /


About CUDA (not needed in this tutorial, but good to start Jetson Inference):
https://jfrog.com/connect/post/installing-cuda-on-nvidia-jetson-nano/ <br>
https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html#post-installation-actions
