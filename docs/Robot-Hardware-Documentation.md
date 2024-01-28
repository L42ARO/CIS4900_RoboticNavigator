> Note this page will constantly be modified as more and more information on the robotic platform is known

## Robotic Kit
[Rosmaster X1](https://a.co/d/7gkyNuO)

## Computer Specs
### JETSON Nano
>![image](https://github.com/L42ARO/CIS4900_RoboticNavigator/assets/89555610/a313f198-184e-4b6a-adb1-7db4978fcf24)
### OS
Ubuntu + [ROS Melodic](https://wiki.ros.org/melodic)
## Motors

[520 Metal Motor with encoder](https://a.co/d/bezZJWe)

Differential Drive
4 Wheels ***(Not mechanum wheels)***

## Sensors
- **Astra Pro Depth Camera**
- **SLAM A1M8 Lidar**
- **9 Axis Altitude Sensor**
- **Encoder From motor**

### Camera
[Astra Pro Depth Camera](https://www.orbbec.com/products/structured-light-camera/astra-series/)
Github repo on how to use it: [here](https://github.com/YahboomTechnology/Astra-Pro-Depth-Camera)

- Depth range 0.6m - 8m
- Depth resolution up to 640X480@30fps
- RGB resolution up to 1920X1080@30fps (Pro Plus)

>- [ ] Confirm which one we're using `Astra Pro`, `Astra Pro Plus` or `Astra S`

### Lidar
[A1M8 Lidar](https://www.amazon.com/Slamtec-RPLIDAR-Scanning-Avoidance-Navigation/dp/B07TJW5SXF)

- Distance Range 0.15 - 6m,White objects
- Angular Range 0-360 Degree
- Distance Resolution<0.5mm
- Angular Resolution≤1Degree
- sampling frequency 8000 times/s (8000Hz)

> - [ ] **NOTE** Investigate onto why Amazon has the robot kit also listed with `ydlidarx3 lidar`?