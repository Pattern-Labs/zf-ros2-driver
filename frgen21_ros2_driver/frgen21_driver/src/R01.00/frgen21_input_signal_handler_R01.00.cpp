#include "R01.00/frgen21_input_signal_handler_R01.00.h"

namespace R01_00 {
InputSignalHandlerR01_00::InputSignalHandlerR01_00(const std::string &remote_ip,
                                                   int remote_port,
                                                   int local_port)
    : remote_ip_(remote_ip), remote_port_(remote_port), local_port_(local_port),
      udp_sender_(remote_ip, remote_port, local_port) {
  alive_counter_ = 1;
}

void InputSignalHandlerR01_00::set_mode(uint8_t range_mode) {
  // create object from struct
  CommsInputSignalsR1_00 input_signals;
  input_signals.VehicleData_AliveCounter = alive_counter_;
  input_signals.VehicleSpeed = 0 + 0x8000;
  input_signals.SensingMode = 1;
  input_signals.PRTMode = 0;

  uint8_t input_data_id[2] = {0x02, 0x00};

  // update mode
  input_signals.RangeMode = range_mode;

  // calculate CRC
  uint16_t crc = CalculateCRC16_P05(reinterpret_cast<uint8_t *>(&input_signals),
                                    2, sizeof(CommsInputSignalsR1_00), 0xFFFF);
  uint16_t crc_final =
      CalculateCRC16_P05(input_data_id, 0, sizeof(input_data_id), crc);

  input_signals.VehicleData_CRC = crc_final;

  // send via UDP
  const char *ptr = reinterpret_cast<const char *>(&input_signals);
  std::vector<char> output_data =
      std::vector<char>(ptr, ptr + sizeof(CommsInputSignalsR1_00));

  udp_sender_.send(output_data);
  alive_counter_++;
}
} // namespace R01_00