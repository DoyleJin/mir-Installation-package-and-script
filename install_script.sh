#!/bin/bash

cat << "EOF"

===============================================================================================
    ____     ____              __  _            __     ____        __          __  _          
   /  _/    / __ \__  ______  / /_(_)__  ____  / /_   / __ \____  / /_  ____  / /_(_)_________
   / /_____/ / / / / / / __ \/ __/ / _ \/ __ \/ __/  / /_/ / __ \/ __ \/ __ \/ __/ / ___/ ___/
 _/ /_____/ /_/ / /_/ / /_/ / /_/ /  __/ / / / /_   / _, _/ /_/ / /_/ / /_/ / /_/ / /__(__  ) 
/___/     \___\_\__,_/\____/\__/_/\___/_/ /_/\__/  /_/ |_|\____/_.___/\____/\__/_/\___/____/  
                                                                                              
===============================================================================================

EOF

SCRIPT_DIR=$(pwd)
echo $SCRIPT_DIR

echo "======= Setup apt env ======="
sudo apt -y install apt-transport-https

echo "======= Install ROS ======="
sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ $DISTRIB_CODENAME main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt update
sudo apt -y install ros-kinetic-desktop-full
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
echo "export ROS_HOSTNAME=$(hostname).local" >> ~/.bashrc
export ROS_HOSTNAME=$(hostname).local
source /opt/ros/kinetic/setup.bash

echo "======= Initialize workspace ======="
cd ~
mkdir -p workspace/library workspace/build/ catkin_ws/src

echo "======= Initialize catkin space ======="
cd ~/catkin_ws
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/catkin_ws/devel/setup.bash

echo "======= Copy file ======="
cd ~/workspace/library
cp $SCRIPT_DIR/library/libfreenect2.zip $SCRIPT_DIR/library/librealsense.zip $SCRIPT_DIR/library/Pangolin.zip .
cd ~/workspace
cp -r $SCRIPT_DIR/exec/demo .
cp $SCRIPT_DIR/exec/ORB_SLAM2.zip .
cd ~/catkin_ws/src/
cp $SCRIPT_DIR/ros_stack/iqr_4d_ros_stack.zip .

echo "======= Library Setup ======="
echo "======= Kinect v2 ======="
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

#Intel Realsense D435
sudo apt -y install git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev

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

echo "======= ROS setup ======="
sudo apt -y install ros-kinetic-joy ros-kinetic-teleop-twist-joy ros-kinetic-navigation
cd ~/catkin_ws/src
unzip iqr_4d_ros_stack.zip
sudo cp iqr_4d_robot/iqr_4d_bringup/config/50-iqr-4d-robot.rules /etc/udev/rules.d
sudo udevadm control --reload-rules && udevadm trigger

echo "======= Kinect v2 ======="
cd ~/catkin_ws/src
# git clone https://github.com/code-iai/iai_kinect2.git

echo "======= UR5 modern driver ======="
sudo apt -y install ros-kinetic-industrial-msgs ros-kinetic-universal-robot

# cd ~/catkin_ws/src
# git clone https://github.com/ros-industrial/ur_modern_driver.git
# cd ~/catkin_ws/src/ur_modern_driver
# git -b kinetic-devel origin/kinetic-devel

echo "======= Realsense ======="
# cd ~/catkin_ws/src
# git clone https://github.com/intel-ros/realsense.git

echo "======= MiR robot ======="
sudo apt -y install ros-kinetic-costmap-queue ros-kinetic-dwb-critics
# cd ~/catkin_ws/src
# git clone https://github.com/mintar/mir_robot.git
# cd mir_robot
# git checkout -b mir_msgs-2.0.2 origin/mir_msgs-2.0.2

echo "======= Install other packages ======="
sudo apt -y install ros-kinetic-sbpl-lattice-planner ros-kinetic-nav-core-adapter ros-kinetic-dwb-plugins ros-kinetic-rospy-message-converter ros-kinetic-controller-manager python-websocket ros-kinetic-hector-slam

echo "======= Build ros package ======="
cd ~/catkin_ws
source ~/catkin_ws/devel/setup.bash
catkin_make

echo "======= ORB SLAM ======="
cd ~/workspace/library
# git clone https://github.com/stevenlovegrove/Pangolin.git
unzip Pangolin.zip
cd ~/workspace/build
mkdir Pangolin
cd Pangolin
cmake ~/workspace/library/Pangolin/
make && sudo make install

cd ~/workspace
# git clone https://github.com/raulmur/ORB_SLAM2.git
unzip ORB_SLAM2.zip
cd ORB_SLAM2
rm -rf Thirdparty/g2o/build Thirdparty/DBoW2/build Examples/ROS/ORB_SLAM2/build build
chmod a+x build.sh
./build.sh

echo "export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:~/workspace/ORB_SLAM2/Examples/ROS" >> ~/.bashrc
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:~/workspace/ORB_SLAM2/Examples/ROS
chmod a+x build_ros.sh
./build_ros.sh

echo "====== All Finish. ======="
