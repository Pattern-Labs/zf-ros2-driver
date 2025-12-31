# Changelog

## v1.2.2 - 2024-11-06

### Fixed
- fix compiler warning regarding Bind placeholders which occurs in Ubuntu 24.04. with Boost 1.73.0 ([solution](https://stackoverflow.com/a/65813178))

## v1.2.1 - 2024-09-04

### Fixed
- fix compiler warning in isCRC32valid() call: taking address of packed member of 'ZF_FRGen21_PTL_Interface_ZX_R01_00::Struct_ZX_Itf4Eth_Payload_Package_t’ may result in an unaligned pointer value


## v1.2.0 - 2024-08-29

### Added
- UDPReceiver: add multicast support and asynchronous receive


## v1.1.0 - 2024-07-05

### Added
- implement FrameStatus::SW_VERSION_CHANGED to detect sw version change in radar frame

### Changed
- fix compiler warning *ignoring attribute 'aligned' because it conflicts with attribute 'packed'*
- rename PointTargetMapping, PhysicalValueDerivation, FrameInterpreter from R2.50 to R02.00
- remove R02.00 PointTargetList struct, reuse from R01.00
- cleanup code


## v1.0.0 - 2024-05-24

### Added

- outsource driver code from frgen21_ros2_driver as shared library
