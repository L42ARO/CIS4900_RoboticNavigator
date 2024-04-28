#!/bin/bash
xhost +

docker run -it \
--net=host \
--env="DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-v /home/jetson/temp:/root/yahboomcar_ros2_ws/temp \
-v /home/jetson/rosboard:/root/rosboard \
-v /home/jetson/maps:/root/maps \
-v /dev/bus/usb/001/001:/dev/bus/usb/001/001 \
-v /dev/bus/usb/001/002:/dev/bus/usb/001/002 \
-v /dev/bus/usb/001/003:/dev/bus/usb/001/003 \
-v /dev/bus/usb/001/004:/dev/bus/usb/001/004 \
-v /dev/bus/usb/001/005:/dev/bus/usb/001/005 \
-v /dev/bus/usb/001/006:/dev/bus/usb/001/006 \
-v /dev/bus/usb/001/007:/dev/bus/usb/001/007 \
-v /dev/bus/usb/001/008:/dev/bus/usb/001/008 \
-v /dev/bus/usb/001/009:/dev/bus/usb/001/009 \
-v /dev/bus/usb/001/010:/dev/bus/usb/001/010 \
-v /dev/bus/usb/001/011:/dev/bus/usb/001/011 \
-v /dev/bus/usb/001/012:/dev/bus/usb/001/012 \
-v /dev/bus/usb/001/013:/dev/bus/usb/001/013 \
-v /home/jetson/usf_robotics:/root/usf_robotics \
-v /home/jetson/python-packages:/usr/local/lib/python3.8/dist-packages \
--device=/dev/myserial \
--device=/dev/rplidar \
--device=/dev/input \
--device=/dev/astradepth \
--device=/dev/astrauvc \
--device=/dev/video0 \
-p 9090:9090 \
-p 8888:8888 \
yahboomtechnology/ros-foxy:4.0.0 /bin/bash
