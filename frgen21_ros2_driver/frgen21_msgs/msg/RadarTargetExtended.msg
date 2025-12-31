# frgen21_msgs/msg/RadarTargetExtended.msg

# This message holds extended data of a radar target

# Range: Range of the target point in m
float32 range

# Azimuth: Azimuth angle in radian of the targt point
float32 azimuth

# Elevation: Elevation angle in radian of the target point
float32 elevation

# Velocity: Target velocity in m/s
float32 velocity

# SNR: Signal noise ratio
float32 snr

float32 power

# Radar cross section of the target
float32 rcs

# Detection confidence of the target
float32 detectionconfidence

# Mean square error
float32 mean_square_error_range
float32 mean_square_error_velocity
float32 mean_square_error_azimuth
float32 mean_square_error_elevation
float32 mean_square_error_subarray
float32 mean_square_error_subarray2nd_best

# Standard deviation
float32 std_dev_range
float32 std_dev_velocity
float32 std_dev_azimuth_angle
float32 std_dev_elevation_angle
float32 std_dev_rcs