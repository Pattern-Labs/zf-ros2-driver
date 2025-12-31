#include "frgen21_driver.h"

FRGen21Driver::FRGen21Driver() : Node("frgen21_driver") {
  this->declare_parameter("input_port", 60001);
  this->declare_parameter("multicast_ip", "");
  this->declare_parameter("host_ip", "0.0.0.0");
  this->declare_parameter("frame_id", "frgen21");
  this->declare_parameter("publish_pc", true);
  this->declare_parameter("publish_scan", true);
  this->declare_parameter("publish_info", true);
  this->declare_parameter("publish_frame", true);
  this->declare_parameter("topic_pc", "/frgen21/pointcloud");
  this->declare_parameter("topic_scan", "/frgen21/scan");
  this->declare_parameter("topic_info", "/frgen21/info");
  this->declare_parameter("topic_frame", "/frgen21/frame");

  this->get_parameter("input_port", input_port_);
  this->get_parameter("multicast_ip", multicast_ip_);
  this->get_parameter("host_ip", host_ip_);
  this->get_parameter("frame_id", frame_id_);
  this->get_parameter("publish_pc", publish_pc_);
  this->get_parameter("publish_scan", publish_scan_);
  this->get_parameter("publish_info", publish_info_);
  this->get_parameter("publish_frame", publish_frame_);
  this->get_parameter("topic_pc", topic_pc_);
  this->get_parameter("topic_scan", topic_scan_);
  this->get_parameter("topic_info", topic_info_);
  this->get_parameter("topic_frame", topic_frame_);

  if (publish_pc_) {
    publisher_pc_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        topic_pc_, rclcpp::SensorDataQoS());
  }
  if (publish_scan_) {
    publisher_scan_ =
        this->create_publisher<frgen21_msgs::msg::RadarScanExtended>(
            topic_scan_, rclcpp::SensorDataQoS());
  }
  if (publish_info_) {
    publisher_info_ = this->create_publisher<frgen21_msgs::msg::RadarInfo>(
        topic_info_, rclcpp::SensorDataQoS());
  }
  if (publish_frame_) {
    publisher_frame_ = this->create_publisher<frgen21_msgs::msg::RadarFrame>(
        topic_frame_, rclcpp::SensorDataQoS());
  }

  // init multicast UDP receiver
  if (multicast_ip_ != "") {
    udp_receiver_ =
        std::make_unique<UdpReceiver>(input_port_, host_ip_, multicast_ip_);
  }
  // init normal UDP receiver
  else {
    udp_receiver_ = std::make_unique<UdpReceiver>(input_port_);
  }

  frame_reassembler_ = FrameReassemblerFactory::createFrameReassembler();
  frame_interpreter_ = FrameInterpreterFactory::createFrameInterpreter();
}

void FRGen21Driver::processPackage(const std::vector<char> &udp_data) {

  FrameStatus frame_status = frame_reassembler_->pushData(udp_data);

  if (frame_status == FrameStatus::FRAME_COMPLETE) {
    std::shared_ptr<char[]> frame = frame_reassembler_->getFrame();
    uint32_t frame_size = frame_reassembler_->getSize();
    std::shared_ptr<char[]> targets = frame_interpreter_->interpret(frame);

    sensor_msgs::msg::PointCloud2 msg_pc;
    frgen21_msgs::msg::RadarScanExtended msg_scan;
    Frgen21Conversion::targetsToROS(targets, frame_id_, msg_pc, msg_scan);
    if (publish_frame_) {
      frgen21_msgs::msg::RadarFrame msg_frame =
          Frgen21Conversion::frameToRadarFrame(frame, frame_size, frame_id_);
      publisher_frame_->publish(msg_frame);
    }
    if (publish_pc_) {
      publisher_pc_->publish(msg_pc);
    }
    if (publish_scan_) {
      publisher_scan_->publish(msg_scan);
    }
    if (publish_info_) {
      frgen21_msgs::msg::RadarInfo msg_info =
          Frgen21Conversion::frameToRadarInfo(frame, frame_id_);
      publisher_info_->publish(msg_info);
    }
  }

  else if (frame_status == FrameStatus::REASSEMBLER_FOUND_VERSION) {
    std::string interface_type = frame_reassembler_->getInterfaceType();
    std::string software_version = frame_reassembler_->getSoftwareVersion();
    RCLCPP_INFO(this->get_logger(), "found sensor with firmware version %s %s",
                interface_type.c_str(), software_version.c_str());

    frame_reassembler_ = FrameReassemblerFactory::createFrameReassembler(
        interface_type, software_version);

    if (!frame_reassembler_) {
      RCLCPP_ERROR(this->get_logger(),
                   "firmware version %s %s is not supported, exiting ...",
                   interface_type.c_str(), software_version.c_str());
      rclcpp::shutdown();
      std::exit(EXIT_SUCCESS);
    }

    frame_interpreter_ = FrameInterpreterFactory::createFrameInterpreter(
        interface_type, software_version);
    frame_reassembler_->pushData(udp_data);
  }

  else if (frame_status == FrameStatus::SW_VERSION_CHANGED) {
    std::string interface_type = frame_reassembler_->getInterfaceType();
    std::string software_version = frame_reassembler_->getSoftwareVersion();
    RCLCPP_INFO(this->get_logger(), "firmware version changed to %s",
                software_version.c_str());

    frame_reassembler_ = FrameReassemblerFactory::createFrameReassembler(
        interface_type, software_version);

    if (!frame_reassembler_) {
      RCLCPP_ERROR(this->get_logger(),
                   "firmware version %s %s is not supported, exiting ...",
                   interface_type.c_str(), software_version.c_str());
      rclcpp::shutdown();
      std::exit(EXIT_SUCCESS);
    }

    frame_interpreter_ = FrameInterpreterFactory::createFrameInterpreter(
        interface_type, software_version);
    frame_reassembler_->pushData(udp_data);
  }

  else if (frame_status == FrameStatus::REASSEMBLER_NOT_INITIALIZED) {
    RCLCPP_INFO_ONCE(this->get_logger(),
                     "waiting for header package to retrieve firmware version");
  } else if (frame_status == FrameStatus::LAST_FRAME_INCOMPLETE) {
    RCLCPP_WARN(this->get_logger(), "frame was dropped: packages incomplete");
  } else if (frame_status == FrameStatus::CRC_ERROR) {
    RCLCPP_WARN(this->get_logger(), "frame was dropped: CRC error");
  } else if (frame_status == FrameStatus::FRAME_OVERFLOW) {
    RCLCPP_WARN(this->get_logger(), "frame was dropped: too much data");
  }
}

void FRGen21Driver::runDriver() {
  // asynchronous reveive
  udp_receiver_->startAsyncReceive(
      std::bind(&FRGen21Driver::processPackage, this, std::placeholders::_1));
}
