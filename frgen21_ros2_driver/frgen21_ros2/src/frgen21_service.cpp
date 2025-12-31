#include "frgen21_input_signal_handler.h"
#include "frgen21_msgs/msg/radar_info.hpp"
#include "frgen21_msgs/srv/input_signals.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <optional>

class InputSignalService : public rclcpp::Node {
public:
  InputSignalService() : Node("frgen21_service") {

    this->declare_parameter("name_service", "/frgen21/input");
    this->declare_parameter("topic_info", "/frgen21/info");
    this->declare_parameter("sensor_ip", "192.168.100.60");
    this->declare_parameter("sensor_input_port", 60002);
    this->declare_parameter("host_output_port", 60002);
    this->get_parameter("name_service", name_service_);
    this->get_parameter("topic_info", topic_info_);
    this->get_parameter("sensor_ip", sensor_ip_);
    this->get_parameter("sensor_input_port", sensor_input_port_);
    this->get_parameter("host_output_port", host_output_port_);

    callback_group_subscriber = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    // callback_group_service =
    // this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    opt_sub = rclcpp::SubscriptionOptions();
    opt_sub.callback_group = callback_group_subscriber;
    // opt_srv = rclcpp::SubscriptionOptions();
    // opt_srv.callback_group = callback_group_service;

    subscription_radarinfo_ =
        this->create_subscription<frgen21_msgs::msg::RadarInfo>(
            topic_info_, rclcpp::SensorDataQoS(),
            std::bind(&InputSignalService::callback_info_for_init, this,
                      std::placeholders::_1),
            opt_sub);

    service_ = this->create_service<frgen21_msgs::srv::InputSignals>(
        name_service_,
        std::bind(&InputSignalService::handle_service_request, this,
                  std::placeholders::_1, std::placeholders::_2));
    last_range_mode_.reset();
  }

private:
  std::unordered_map<std::string, uint8_t> mapping_range_mode{
      {"short", 0}, {"mid", 1}, {"far", 2}, {"SFMCW mid", 3}, {"SFMCW far", 4}};

  void
  callback_info_for_init(const frgen21_msgs::msg::RadarInfo::SharedPtr msg) {

    if (input_signal_handler_ == nullptr ||
        sensor_sw_ != msg->software_version_overall.c_str()) {
      sensor_sw_ = msg->software_version_overall.c_str();
      input_signal_handler_ =
          frgen21::InputSignalHandlerFactory::createInputSignalHandler(
              msg->software_version_overall.c_str(), sensor_ip_,
              sensor_input_port_, host_output_port_);
      RCLCPP_INFO(this->get_logger(),
                  "created InputSignalHandler for firmware version %s",
                  msg->software_version_overall.c_str());
    }

    if (input_signal_handler_ == nullptr) {
      RCLCPP_ERROR(this->get_logger(),
                   "no InputSignalHandler found for firmware version %s",
                   msg->software_version_overall.c_str());
      rclcpp::shutdown();
      std::exit(EXIT_SUCCESS);
    }

    last_range_mode_ = std::make_optional(mapping_range_mode[msg->range_mode]);
  }

  void handle_service_request(
      const std::shared_ptr<frgen21_msgs::srv::InputSignals::Request> request,
      const std::shared_ptr<frgen21_msgs::srv::InputSignals::Response>
          response) {

    response->success = false;
    response->range_mode = -1;
    RCLCPP_INFO(this->get_logger(), "incoming range mode request: %d",
                request->range_mode);

    if (input_signal_handler_ == nullptr) {
      RCLCPP_WARN(
          this->get_logger(),
          "InputSignalHandler not yet initialized, waiting for %s topic",
          topic_info_.c_str());
      return;
    }

    // send UDP with InputSignals to sensor
    input_signal_handler_->set_mode(request->range_mode);

    // check if sensor is changing mode within 3 cycles
    last_range_mode_.reset();
    bool range_mode_matched = false;
    bool range_mode_received = false;
    auto start_time = std::chrono::steady_clock::now();

    while (!range_mode_matched) {
      if (last_range_mode_.has_value()) {
        range_mode_received = true;
        response->range_mode = last_range_mode_.value();
        if (last_range_mode_.value() == request->range_mode) {
          range_mode_matched = true;
          response->success = true;
          break;
        }
      }
      if (std::chrono::steady_clock::now() - start_time >=
          std::chrono::milliseconds(MAX_WAIT_TIME_MS)) {
        break;
      }
      // sleep for a short duration to avoid high CPU usage
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (!range_mode_matched) {
      if (!range_mode_received) {
        RCLCPP_WARN(this->get_logger(),
                    "changing range mode: unresolved - no RadarInfo message "
                    "received, topic %s still available?",
                    topic_info_.c_str());
        return;
      }
      RCLCPP_WARN(this->get_logger(), "changing range mode: fail");
    } else {
      RCLCPP_INFO(this->get_logger(), "changing range mode: success");
    }
  }

  const int MAX_WAIT_TIME_MS = 210;
  std::optional<uint8_t> last_range_mode_;
  std::string name_service_;
  std::string topic_info_;
  std::string sensor_ip_;
  std::string sensor_sw_;
  int sensor_input_port_;
  int host_output_port_;
  std::unique_ptr<frgen21::InputSignalHandler> input_signal_handler_;
  rclcpp::Subscription<frgen21_msgs::msg::RadarInfo>::SharedPtr
      subscription_radarinfo_;
  rclcpp::Service<frgen21_msgs::srv::InputSignals>::SharedPtr service_;
  rclcpp::CallbackGroup::SharedPtr callback_group_subscriber;
  rclcpp::SubscriptionOptions opt_sub;
  // rclcpp::CallbackGroup::SharedPtr callback_group_service;
  // rclcpp::SubscriptionOptions opt_srv;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto input_signal_service = std::make_shared<InputSignalService>();

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(input_signal_service);
  executor->spin();

  rclcpp::shutdown();
  return 0;
}