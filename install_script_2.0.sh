#!/bin/bash

cat << "EOF"

===============================================================================================
         ____     ____   
        /  _/    / __ \
        / /_____/ / / /
    ___/ /_____/ /_/ /
    \___/     /_____/

===============================================================================================

EOF

SCRIPT_DIR=$(pwd)
echo $SCRIPT_DIR

echo "1.0======= Initialize workspace ======="
cd ~
mkdir -p workspace/library workspace/build/ mir_1008_catkin_ws/src
echo "1.0======= Initialize workspace Finish======="

echo "2.0======= Initialize catkin space ======="
cd ~/mir_1008_catkin_ws
catkin_make
echo "source ~/mir_1008_catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/mir_1008_catkin_ws/devel/setup.bash
echo "2.0======= Initialize catkin space Finish======="

echo "3.0======= Copy file ======="
cd ~/workspace/library
cp $SCRIPT_DIR/library/libfreenect2.zip $SCRIPT_DIR/library/librealsense.zip $SCRIPT_DIR/library/Pangolin.zip .
cd ~/workspace
cp -r $SCRIPT_DIR/exec/demo .
cp $SCRIPT_DIR/exec/ORB_SLAM2.zip .
cd ~/mir_1008_catkin_ws/src/
cp $SCRIPT_DIR/ros_stack/iqr_4d_ros_stack.zip .
echo "3.0======= Copy file Finish======="

echo "4.0======= Library Setup ======="

echo "4.1======= Kinect v2 ======="
sudo apt -y install build-essential cmake pkg-config libturbojpeg libjpeg-turbo8-dev libusb-1.0-0-dev libglfw3-dev libva-dev libjpeg-dev

cd ~/workspace/library
# git clone https://github.com/OpenKinect/libfreenect2.git
unzip libfreenect2.zip

cd ~/workspace/build
mkdir libfreenect2
cd libfreenect2
cmake ~/workspace/library/libfreenect2
make && sudo make install

sudo cp ~/workspace/library/libfreenect2/platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger
# reload udev rules without reboot
echo "4.1======= Kinect v2 Finish======="

echo "4.2======= realsense2 ======="
#Intel Realsense D435
sudo apt-key adv --keyserver keys.gnupg.net --recv-key C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C8B3A55A6F3EFCDE
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u
sudo apt -y install git librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg

cd ~/workspace/library/
#git clone https://github.com/IntelRealSense/librealsense.git
unzip librealsense.zip

cd librealsense
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger

cd ~/workspace/build
mkdir librealsense
cd librealsense
cmake ~/workspace/library/librealsense -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=true -DBUILD_WITH_TM2=false
sudo make uninstall && make clean && make && sudo make install
echo "4.2======= realsense2  Finish======="
echo "4.0======= Library Setup Finish======="

echo "5.0======= ROS setup ======="
sudo apt -y install ros-kinetic-joy ros-kinetic-teleop-twist-joy ros-kinetic-navigation
cd ~/mir_1008_catkin_ws/src
unzip iqr_4d_ros_stack.zip
sudo cp ~/mir_1008_catkin_ws/src/iqr_4d_robot/iqr_4d_bringup/config/50-iqr-4d-robot.rules /etc/udev/rules.d
sudo udevadm control --reload-rules && udevadm trigger

# echo "======= Kinect v2 ======="
# git clone https://github.com/code-iai/iai_kinect2.git
echo "5.0======= ROS setup Finish======="


echo "======= MiR robot ======="
sudo apt -y install ros-kinetic-costmap-queue ros-kinetic-dwb-critics
# cd ~/mir_1008_catkin_ws/src
# git clone https://github.com/mintar/mir_robot.git
# cd mir_robot
# git checkout -b mir_msgs-2.0.2 origin/mir_msgs-2.0.2

echo "======= Install other packages ======="
sudo apt -y install ros-kinetic-sbpl-lattice-planner ros-kinetic-nav-core-adapter ros-kinetic-dwb-plugins ros-kinetic-rospy-message-converter ros-kinetic-controller-manager python-websocket ros-kinetic-hector-slam

echo "======= Build ros package ======="
cd ~/mir_1008_catkin_ws
source ~/mir_1008_catkin_ws/devel/setup.bash
catkin_make





echo "======= ntpdate ======="
sudo apt-get update
sudo apt-get install ntpdate

echo "======= rgbd launch ======="
#cd ~/mir_1008_catkin_ws/src
#git clone https://github.com/ros-drivers/rgbd_launch.git
#cd ..
#catkin_make

echo "====== All Finish. ======="
