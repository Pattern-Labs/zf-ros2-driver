# frgen21_msgs

## About the project

This is the custom ROS 2 message and service definition which is used by the [FRGen21 ROS 2 driver](https://git.emea.zf-world.com/frgen21/frgen21_ros2_driver).

## Custom messages
* [frgen21_msgs/msg/RadarScanExtended](msg/RadarScanExtended.msg) (radar point targets in spherical coordinates)
* [frgen21_msgs/msg/RadarInfo](msg/RadarInfo.msg) (additional sensor info like firmware version, range mode, etc.)
* [frgen21_msgs/msg/RadarFrame](msg/RadarFrame.msg) (bytes of reassembled radar frame from UDP packages)

## Custom services
* [frgen21_msgs/srv/InputSignals](srv/InputSignals.srv) (service for FRGen21 input signals)


## Contact

Yannik Motzet - [yannik.motzet@zf.com](mailto:yannik.motzet@zf.com)