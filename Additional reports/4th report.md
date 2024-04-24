In this session we are going to learn how to boot the Jetson Nano from an M.2 M key SSD. As you know, the eMMC of the Jetson Nano only has 15Gb of storage available, which is very tiny for developping. In order to have more space on the Jetson Nano, I will be equipping it with a 500Gb SSD. This guide describes how to set up your system to boot from an M.2 SSD.<br>

There are two methods the first one is the more complex and the second one is the easier and more straightforward. **Both methods have been tested and work correctly !**<br>

## 1st Method:

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
Modify the following line : `APPEND ${cbootargs} quiet root=<YOUR_STORAGE_DEVICE> rw rootwait rootfstype=ext4 console=ttyTCU0,115200n8 console=tty0 fbcon=map:0 net.ifnames=0`

- After a reboot your system will start from the SSD !<br>
  
- You can validate the SSD boot by typing : `df /`
![image](https://github.com/anasderkaoui/AutoRCX/assets/115218309/200a7fda-b2ed-43d3-a9d4-2480c9b44f43)<br>
You should see : **<YOUR_STORAGE_DEVICE>** Mounted on /

## 2nd Method (Easiest):

- Ready up your storage device (weather it be a USB, SD card, SSD, Hard Drive...) <br>
- Format it using the "Format" option in the "Disks" application on the Jetson Nano <br>
![image](https://github.com/anasderkaoui/AutoRCX/assets/115218309/f07d0288-139d-4609-bfe1-fee7217debca)
![image2](https://github.com/anasderkaoui/AutoRCX/assets/115218309/bf8dae60-09b8-45dc-9602-05985d6ca901)
- Add a partition to the drive by clicking on the "Plus" sign, and then choose Ext4 format
- After that you should see the device's partition name <br> In this case it is called : **/dev/sda1** <br>
![image4](https://github.com/anasderkaoui/AutoRCX/assets/115218309/47309926-aafd-4f6e-a1d2-771a7444e3d4)
- Locate the partition newly created on the left bar (usually at the end) and open it in order to mount that partition (if not done automatically)
![image3](https://github.com/anasderkaoui/AutoRCX/assets/115218309/258d0d67-6b98-4c54-8b75-f22c76c640bf)
- Open a terminal and clone this repository to your main directory (or any other directory of your choice): `git clone https://github.com/JetsonHacksNano/bootFromUSB.git`
- Go to that repository (you should be at "bootFromUSB") and run : `./copyRootToUSB.sh -p /dev/"your storage device's partition name"` (for example `./copyRootToUSB.sh -p /dev/sda1`). This will take a while.
- Now go to your storage device, open the "boot" folder, then "extlinux", open a terminal and run : `sudo cp extlinux.conf extlinux.conf.original`
- Now run : `sudo gedit extlinux.conf`
- A window should open in order to edit the file. Copy the main paragraph and change the LABEL's name to any other name besides "primary" <br>
![image](https://github.com/anasderkaoui/AutoRCX/assets/115218309/c26f8a6a-ff04-4b59-9c92-a8a5a087ba84)
- In the line that starts with "APPEND", modify the "/dev/mmcblk0p1" to be the one according to your storage device (for example "/dev/sda1" or "/dev/mmcblk1p1")
- (You can skip this step if you want it is not very important) You can be more specific and replace this part of the same line "root=/dev/mmcblk0p1" with "root=PARTUUID=<device's PARTUUID>". In order to get the device's PARTUUID, run this command : `sudo blkid` and then search for the PARTUUID of your device's partition.
- Now we are going to do the same thing with the storage of the Jetson Nano. Open a terminal and run : `cd ~/boot/extlinux/extlinux.conf && sudo cp extlinux.conf extlinux.conf.original && sudo gedit extlinux.conf`


About CUDA (not needed in this tutorial, but good to start Jetson Inference):
https://jfrog.com/connect/post/installing-cuda-on-nvidia-jetson-nano/ <br>
https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html#post-installation-actions
