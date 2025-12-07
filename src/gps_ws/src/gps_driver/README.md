# GPS Driver for ROS2

ROS2 GPS driver for reading NMEA data from serial port.

## Build

```bash
cd ~/gps_ws
colcon build
source install/setup.bash
```

## Usage

```bash
ros2 launch gps_driver gps_driver.launch.py
```

Or with custom parameters:

```bash
ros2 launch gps_driver gps_driver.launch.py port:=/dev/ttyUSB0 baud:=9600
```

## Topics

- `/fix` (sensor_msgs/NavSatFix): GPS position data
- `/vel` (geometry_msgs/TwistStamped): GPS velocity data
