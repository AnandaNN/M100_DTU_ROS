# Installing and running (Ubuntu 18.04)

## Install ROS Melodic
[http://wiki.ros.org/melodic/Installation/Ubuntu]

`sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`

`sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654`

`sudo apt install ros-melodic-desktop-full`

`echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc`

`source ~/.bashrc`

`sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential python-catkin-tools`

`sudo rosdep init`

`rosdep update`


## Install dependencies

`sudo apt install ros-melodic-nav-msgs`

`sudo apt install ros-melodic-image-transport`

`sudo apt install ros-melodic-urg-node`

`sudo apt install ros-melodic-nmea-msgs`

`sudo apt install ros-melodic-message-to-tf`

`sudo apt install ros-melodic-rosserial`

`sudo apt install ros-melodic-rosserial-arduino`

`sudo apt install v4l-utils`

## Install OpenCV

Tested and run with opencv 4.2.0 (should work with opencv > 3.3)

[https://linuxize.com/post/how-to-install-opencv-on-ubuntu-18-04/]

`sudo apt install build-essential cmake git pkg-config libgtk-3-dev libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libxvidcore-dev libx264-dev libjpeg-dev libpng-dev libtiff-dev gfortran openexr libatlas-base-dev python3-dev python3-numpy libtbb2 libtbb-dev libdc1394-22-dev`

`mkdir opencv_build && cd opencv_build`

`git clone -b 4.2.0 https://github.com/opencv/opencv.git`

`git clone -b 4.2.0 https://github.com/opencv/opencv_contrib.git`

`cd opencv`

`mkdir build`

`cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D INSTALL_C_EXAMPLES=ON -D INSTALL_PYTHON_EXAMPLES=ON -D OPENCV_GENERATE_PKGCONFIG=ON -D OPENCV_ENABLE_NONFREE:BOOL=ON -D OPENCV_EXTRA_MODULES_PATH=~/Libraries/opencv_build/opencv_contrib/modules -D BUILD_EXAMPLES=ON ..`

`make -j4`

`sudo make install`

`pip install opencv-contrib-python`

Test it with: `python -c "import cv2; print cv2.__version__"`

## Instal DJI SDK

Clone repository branch 3.9

`git clone -b 3.9 https://github.com/dji-sdk/Onboard-SDK.git`

Compile and install 

`cd Onboard-SDK`

`mkdir build && cd build`

`cmake ..`

`make`

`sudo make install`

`sudo ldconfig`

`sudo usermod -a -G dialout $USER`

Log in and out and DJI SDK is ready to go

## Setting up the ROS ws

`mkdir -p ~/ROS/DJI_OSDK_ws/src`

`catkin_init_workspace`

`cd ..`

`catkin init`

`catkin build`

`echo "source ~/ROS/DJI_OSDK_ws/devel/setup.bash" >> ~/.bashrc`

`source ~/.bashrc`

## Install Guidance

`cd ~/ROS/DJI_OSDK_ws/src`

`git clone https://github.com/AnandaNN/Guidance-SDK-ROS.git`

`sudo sh -c 'echo "SUBSYSTEM==\"usb\", ATTR{idVendor}==\"fff0\", ATTR{idProduct}==\"d009\", MODE=\"0666\"" > /etc/udev/rules.d/51-guidance.rules'`

`catkin build` Needs to be run a couple of times before it compiles (Something with the message generation)


## Fixing usb cam

`cd ~/ROS/DJI_OSDK_ws/src`

`git clone https://github.com/ros-drivers/usb_cam.git`

Open `usb_cam/src/usb_cam.cpp` and add:
`av_log_set_level(AV_LOG_ERROR);`
to line 376

This fixes the following warning beeing spammed to the console:
`[swscaler @ 0x564625b66b60] deprecated pixel format used, make sure you did set range correctly`

## Teensy UDEV rules
Add the file: `49-teensy.rules` to the udev folder : `/etc/udev/rules.d/` and add the following content to it:

[https://www.pjrc.com/teensy/49-teensy.rules]


## Python

`pip install pandas`

`pip install scikit-image`


## M100 DTU ROS Package

`cd ~/ROS/DJI_OSDK_ws/src`

`git clone https://github.com/AnandaNN/M100_DTU_ROS.git`
