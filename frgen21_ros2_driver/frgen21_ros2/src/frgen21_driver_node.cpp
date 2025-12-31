#include "frgen21_driver.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto driver = std::make_shared<FRGen21Driver>();
  driver->runDriver();
  rclcpp::spin(driver);
  rclcpp::shutdown();
  return 0;
}