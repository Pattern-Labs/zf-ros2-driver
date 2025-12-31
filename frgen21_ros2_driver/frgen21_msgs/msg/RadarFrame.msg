# frgen21_msgs/msg/FRGen21Frame.msg

# Time of sensor data acquisition, radar coordinate frame ID
std_msgs/Header header

# radar firmware version
string version

# data containing the Point Target List reasembled from UDP packages
uint8[] data