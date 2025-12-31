#include "frgen21_input_signal_handler.h"
#include "R01.00/frgen21_input_signal_handler_R01.00.h"
#include "R02.00/frgen21_input_signal_handler_R02.00.h"
#include "R02.02/frgen21_input_signal_handler_R02.02.h"

namespace frgen21 {
uint16_t InputSignalHandler::CalculateCRC16_P05(
    const uint8_t input_data[], const uint16_t data_offset,
    const uint16_t crc_length, const uint16_t crc_start_value) {
  uint8_t crc_loop_counter;
  uint16_t crc_data_ptr_addr;
  uint16_t crc_cal;

  crc_cal = crc_start_value;
  for (crc_data_ptr_addr = data_offset; crc_data_ptr_addr < crc_length;
       crc_data_ptr_addr++) {
    crc_cal = (crc_cal ^ (input_data[crc_data_ptr_addr] << 8));
    for (crc_loop_counter = 0; crc_loop_counter < 8; crc_loop_counter++) {
      if ((crc_cal & 0x8000) > 0) {
        crc_cal = ((crc_cal << 1) ^ 0x1021);
      } else {
        crc_cal <<= 1;
      }
    }
  }
  return crc_cal;
}

std::unique_ptr<InputSignalHandler>
InputSignalHandlerFactory::createInputSignalHandler(
    std::string software_version, const std::string &remote_ip, int remote_port,
    int local_port) {

  size_t sw_version_dot_pos = software_version.find('.');
  int8_t sw_version_major =
      std::stoul(software_version.substr(1, sw_version_dot_pos - 1));
  int8_t sw_version_minor =
      std::stoul(software_version.substr(sw_version_dot_pos + 1));
  ;

  if (sw_version_major == 1) {
    return std::make_unique<R01_00::InputSignalHandlerR01_00>(
        remote_ip, remote_port, local_port);
  } else if (sw_version_major == 2) {
    if (sw_version_minor <= 1) {
      return std::make_unique<R02_00::InputSignalHandlerR02_00>(
          remote_ip, remote_port, local_port);
    }
    return std::make_unique<R02_02::InputSignalHandlerR02_02>(
        remote_ip, remote_port, local_port);
  } else if (sw_version_major > 2) {
    return std::make_unique<R02_02::InputSignalHandlerR02_02>(
        remote_ip, remote_port, local_port);
  } else
    return nullptr;
}
} // namespace frgen21