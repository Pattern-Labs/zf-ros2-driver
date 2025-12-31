# Changelog

## v1.2.2 - 2024-11-05

### Changed
 - switch to frgen21_driver v1.2.2 (fix compiler warning with Boost 1.73)
 - rename launch files for better understanding
 - outsource frgen21_msgs to standalone repo

## v1.2.1 - 2024-09-04

### Changed
 - switch to frgen21_driver v1.2.1 (fix compiler warning regarding unaligned pointer value)


## 1.2.0 - 2024-08-29

### Changed
 - [adapt changes from frgen21_driver v1.1.0](https://git.emea.zf-world.com/frgen21/frgen21_ros2_driver/-/commit/1d3735127780fa649dc2c7751accf79de739818b)
 - cleanup code

### Added
 - [integrate FrameStatus::SW_VERSION_CHANGED of frgen21_driver v1.1.0](https://git.emea.zf-world.com/frgen21/frgen21_ros2_driver/-/commit/1d3735127780fa649dc2c7751accf79de739818b)
 - [integrate multicast support of frgen21_driver v1.2.0](https://git.emea.zf-world.com/frgen21/frgen21_ros2_driver/-/commit/0532b60b69e8a26e21fa93f3bb7b13c0af6cf571)


## v1.1.0 - 2024-06-05

### Changed
 - [outsource driver code as shared library](https://git.emea.zf-world.com/yannik.motzet/frgen21_ros2_driver/-/merge_requests/12)
 - [improve replay_udp_from_pcap.py ](https://git.emea.zf-world.com/yannik.motzet/frgen21_ros2_driver/-/merge_requests/11)


## v1.0.0 - 2024-03-12

### Added

- [add RadarFrame message to output reassembled UDP packages](https://git.emea.zf-world.com/yannik.motzet/frgen21_ros2_driver/-/issues/7)
- [fill range_mode, chirp_mode, channel in RadarInfo message](https://git.emea.zf-world.com/yannik.motzet/frgen21_ros2_driver/-/merge_requests/5)
- [implement Service for Input Signals](https://git.emea.zf-world.com/yannik.motzet/frgen21_ros2_driver/-/issues/3)
- [add CRC check in FrameReassembler](https://git.emea.zf-world.com/yannik.motzet/frgen21_ros2_driver/-/issues/10)
   
### Changed

- [rename RadarInfos message to RadarInfo](https://git.emea.zf-world.com/yannik.motzet/frgen21_ros2_driver/-/merge_requests/5)
- [move msg and srv definitions to seperate project](https://git.emea.zf-world.com/yannik.motzet/frgen21_ros2_driver/-/merge_requests/8)
 
### Fixed


## v0.0.2 - 2023-12-14

### Added

- [add FRGen21 mesh](https://git.emea.zf-world.com/yannik.motzet/frgen21_ros2_driver/-/issues/4)
- [output RadarScanExtended message](https://git.emea.zf-world.com/yannik.motzet/frgen21_ros2_driver/-/issues/1)
- [output RadarInfos message](https://git.emea.zf-world.com/yannik.motzet/frgen21_ros2_driver/-/issues/2)


## v0.0.1 - 2023-12-07
 
### Added

- init project using frgen21_driver code to publish PointCloud2
