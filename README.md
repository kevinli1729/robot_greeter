# Introduction
This ROS package contains an application that scans an entryway for people, 
recognizes them, and greets them using humanoid robot Dreamer. It uses 
OpenCV to detect and recognize faces, and ControlIt! to move the robot.

# Installation

This package requires ROS, OpenCV, and ControlIt!.

## ROS
[Here](http://wiki.ros.org/ROS/Installation) are the official instructions.

## OpenCV
Download OpenCV 2.4.11 for Linux [here](http://opencv.org/downloads.html).

Follow the official installation guide for Linux [here](http://docs.opencv.org/doc/tutorials/introduction/linux_install/linux_install.html#linux-installation).
Below is a summary of the commands to execute:

    $ cd ~/Downloads
    $ unzip opencv-2.4.11.zip
    $ cd opencv-2.4.11/
    $ mkdir release
    $ cd release
    $ cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local ..
    $ make
    $ sudo make install

The OpenCV files are installed to /usr/local/bin/opencv*, /usr/local/include/openscv2, /usr/local/lib/, and /usr/local/share/OpenCV.

## ControlIt!
[Here](http://robotcontrolit.com/installation) are the official instructions.

## Robot Greeter
After you have setup a ROS workspace during the installation of ControlIt!, add this package to the workspace:

    $ roscd
    $ cd ../src
    $ git clone git@github.com:liangfok/robot_greeter.git

You should now have the robot_greeter package within your workspace:

    $ roscd robot_greeter

To compile the application:

    $ roscd 
    $ cd ..
    $ catkin_make
