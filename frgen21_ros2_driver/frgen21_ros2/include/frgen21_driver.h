#ifndef FRGEN21_DRIVER_H
#define FRGEN21_DRIVER_H

#include "rclcpp/rclcpp.hpp"

#include "frgen21_conversion.h"
#include "frgen21_frame_interpreter.h"
#include "frgen21_frame_reassembler.h"
#include "udp_com.h"

using namespace frgen21;

class FRGen21Driver : public rclcpp::Node {
public:
  FRGen21Driver();
  void runDriver();

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_pc_;
  rclcpp::Publisher<frgen21_msgs::msg::RadarScanExtended>::SharedPtr
      publisher_scan_;
  rclcpp::Publisher<frgen21_msgs::msg::RadarInfo>::SharedPtr publisher_info_;
  rclcpp::Publisher<frgen21_msgs::msg::RadarFrame>::SharedPtr publisher_frame_;

  std::unique_ptr<UdpReceiver> udp_receiver_;
  std::unique_ptr<FrameReassembler> frame_reassembler_;
  std::unique_ptr<FrameInterpreter> frame_interpreter_;

  int input_port_;
  bool publish_scan_;
  bool publish_pc_;
  bool publish_info_;
  bool publish_frame_;
  std::string multicast_ip_;
  std::string host_ip_;
  std::string frame_id_;
  std::string topic_pc_;
  std::string topic_scan_;
  std::string topic_info_;
  std::string topic_frame_;
  void processPackage(const std::vector<char> &udp_data);
};

#endif // FRGEN21_DRIVER_H