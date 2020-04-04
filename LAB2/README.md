## Package information

Package name: lab2_hw

IMU default port: "/dev/ttyUSB0"

GPS default port: "/dev/ttyUSB1"

You can modify them in

lab2_hw/launch/driver_imu_gps.launch

or

lab2_hw/scripts/imu_node.py and lab2_hw/scripts/gps_node.py

## How to launch IMU and GPS driver:

roslaunch lab2_hw driver_imu_gps.launch


