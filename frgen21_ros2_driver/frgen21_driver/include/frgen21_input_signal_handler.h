#ifndef FRGEN21_INPUT_SIGNAL_HANDLER_H
#define FRGEN21_INPUT_SIGNAL_HANDLER_H

#include "udp_com.h"
#include <stdint.h>
#include <string>

namespace frgen21 {
class InputSignalHandler {
public:
  virtual void set_mode(uint8_t range_mode) = 0;
  uint16_t CalculateCRC16_P05(const uint8_t input_data[],
                              const uint16_t data_offset,
                              const uint16_t crc_length,
                              const uint16_t crc_start_value);
};

class InputSignalHandlerFactory {
public:
  static std::unique_ptr<InputSignalHandler>
  createInputSignalHandler(std::string software_version = "",
                           const std::string &remote_ip = "192.168.100.60",
                           int remote_port = 60002, int local_port = 60002);
};

} // namespace frgen21

#endif // FRGEN21_INPUT_SIGNAL_HANDLER_H