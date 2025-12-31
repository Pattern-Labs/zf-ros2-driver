#ifndef FRGEN21_CONVERSION_H
#define FRGEN21_CONVERSION_H

#include <cmath>
#include <string>
#include <unordered_map>

#include "R02.00/PointTargetInterfaceMapping.h"
#include "R02.00/R2PhysicalValueDerivation.h"

#include "frgen21_msgs/msg/radar_frame.hpp"
#include "frgen21_msgs/msg/radar_info.hpp"
#include "frgen21_msgs/msg/radar_scan_extended.hpp"
#include "frgen21_msgs/msg/radar_target_extended.hpp"
#include "frgen21_msgs/srv/input_signals.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

class Frgen21Conversion {
public:
  static void get_radar_mode(std::string &range_mode, std::string &chirp_mode,
                             std::string &channel, float bandwidth,
                             float carrier_frequency,
                             float center_frequency_step);
  static sensor_msgs::msg::PointCloud2
  targetsToPointCloud2(std::shared_ptr<char[]> &targets, std::string frame_id);
  static frgen21_msgs::msg::RadarScanExtended
  targetsToRadarScanExtended(std::shared_ptr<char[]> &targets,
                             std::string frame_id);
  static void targetsToROS(std::shared_ptr<char[]> &targets,
                           std::string frame_id,
                           sensor_msgs::msg::PointCloud2 &msg_pc,
                           frgen21_msgs::msg::RadarScanExtended &msg_scan);
  static frgen21_msgs::msg::RadarInfo
  frameToRadarInfo(std::shared_ptr<char[]> &frame, std::string frame_id);
  static frgen21_msgs::msg::RadarFrame
  frameToRadarFrame(std::shared_ptr<char[]> &frame, uint32_t frame_size,
                    std::string frame_id);
};

#endif // FRGEN21_CONVERSIOn_H