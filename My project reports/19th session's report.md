## 11/04/2023

<br />

# Report of the nineteenth session

<br />

Today marked a significant milestone in our project, as we took a giant leap forward. My attention was primarily devoted to the powerful Nvidia Jetson Nano card, which proved to be instrumental in propelling us towards success.<br />

In the previous session, I had encountered a roadblock in 3D printing the component responsible for carrying the laser sensor. However, in this session, I approached the task with renewed determination and precision. With only one attempt, I managed to obtain the correct measurements and create a carrier that securely held the sensor in place. The carrier is now functioning seamlessly and provides the expected level of stability, ensuring the sensor's accuracy and reliability.<br />

The highlight of this session was undoubtedly my first experience with the Nvidia Jetson Nano card. This impressive piece of technology is a game-changer, equipped with Ubuntu OS, and it has 4GB of internal storage. To further augment our project's capacity, our supervisor generously provided us with an additional 64GB SD card, which will help store our next projects.<br />

![-jetson-waveshare-35844](https://user-images.githubusercontent.com/115218309/232207341-64b2d813-9357-45a4-a94c-97a9792ac67f.jpg)

Our chosen operating system, ROS, required some crucial preliminary steps before installation. I began by formatting and mounting the SD card's disk partition in Linux, utilizing ext4 to make optimal use of the additional storage capacity. This task was crucial in ensuring smooth functioning and efficient management of the vast amounts of data we anticipated handling. You can follow the commands below to **mount and format your SD card**:<br />

`lsblk` :locate a partition you wish to format. The SD card appears as "sdb1..." or in our case "mmcblk1..."<br />
`sudo mkfs -t ext4 /dev/sdb1` :format disk partition with ext4 file system<br />
`lsblk -f` :verify the file system change<br />
`sudo mkdir -p [yourmountpoint]` :create a mount point<br />
`sudo mount -t auto /dev/sdb1 [yourmountpoint]` :mount the partition<br />
`lsblk -f` :verify if the partition is mounted<br />

After the process of mounting and formating the SD card is complete, now it is time to **move user data from /home to /media/storage/home**.<br />
Follow the instructions given bellow:<br />

Temporarily mount the new partition:<br />
`sudo mkdir /mnt/tmp`<br />
`sudo mount /dev/sdb1 /mnt/tmp`<br />

Copy HOME to the new location:<br />
`sudo rsync -avx /home/ /mnt/tmp`<br />

Mount the new partition as HOME:<br />
`sudo mount /dev/sdb1 /home`<br />
`sudo umount /home` :unmount the new home first<br />
`sudo rm -rf /home/*` :deletes the old home<br />

Make HOME permanent:<br />
We need to know the UUID of the new partition for the fstab entry seen from:<br />
`sudo blkid`<br />
Note or copy/paste the correct UUID (of your sd card partition) to edit the fstab with:<br />
`sudo nano /etc/fstab` :you can use any other editor (vim for example), and add the following line at the end:
`UUID=noted number from above    /home    ext4    defaults   0  2`<br />

Finally, reboot. After a reboot, /home resides on the new drive having plenty of space.<br />

After rebooting the card, we are ready to **install ROS** on it. We chose to install ROS Melodic which is compatible with the version of Ubuntu that our card is running (20.04). In order to do so, you could follow the instructions at the end of the page or simply copy and execute the bash file bellow in your terminal:<br />
```ruby
#!/bin/bash

# See http://wiki.ros.org/melodic/Installation/Ubuntu for details
set -e;
sudo apt update;
sudo apt install curl;
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list';

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -;

sudo apt update;

sudo apt install ros-melodic-desktop-full;

echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc;
source ~/.bashrc;

source /opt/ros/melodic/setup.bash;

sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential;

sudo apt install python-rosdep;

sudo rosdep init;
rosdep update;

printenv | grep ROS;
# should print something like :
# ROS_ETC_DIR=/opt/ros/melodic/etc/ros
# ROS_ROOT=/opt/ros/melodic/share/ros
# ROS_MASTER_URI=http://localhost:11311
# ROS_VERSION=1
# ROS_PYTHON_VERSION=2
# ROS_PACKAGE_PATH=/opt/ros/melodic/share
# ROSLISP_PACKAGE_DIRECTORIES=
# ROS_DISTRO=melodic
```
Sources:<br />
- [Format SD card](https://phoenixnap.com/kb/linux-format-disk)
- [Move home to SD card](https://askubuntu.com/questions/21321/move-home-folder-to-second-drive)
- [Installing ROS](http://wiki.ros.org/melodic/Installation/Ubuntu)
- [Configuring ROS](http://wiki.ros.org/fr/ROS/Tutorials/InstallingandConfiguringROSEnvironment)
