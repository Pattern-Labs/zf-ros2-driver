#include "R02.02/frgen21_input_signal_handler_R02.02.h"

namespace R02_02 {
InputSignalHandlerR02_02::InputSignalHandlerR02_02(const std::string &remote_ip,
                                                   int remote_port,
                                                   int local_port)
    : remote_ip_(remote_ip), remote_port_(remote_port), local_port_(local_port),
      udp_sender_(remote_ip, remote_port, local_port) {
  alive_counter_ = 1;
}

void InputSignalHandlerR02_02::set_mode(uint8_t range_mode) {
  // create object from struct
  CommsInputSignalsR02_02 input_signals;
  input_signals.VehicleData_AliveCounter = alive_counter_;
  input_signals.VehicleSpeed = 0 + 0x8000;
  input_signals.SensingMode = 1;
  input_signals.PRTMode = 0;

  uint8_t input_data_id[2] = {0x02, 0x00};

  // update mode
  input_signals.RangeMode = range_mode;

  // calculate CRC
  uint16_t crc = CalculateCRC16_P05(
      reinterpret_cast<uint8_t *>(&input_signals), 2,
      sizeof(input_signals) - sizeof(input_signals.AntiReplayCounter) -
          sizeof(input_signals.CMAC),
      0xFFFF);
  uint16_t crc_final =
      CalculateCRC16_P05(input_data_id, 0, sizeof(input_data_id), crc);

  input_signals.VehicleData_CRC = crc_final;

  // send via UDP
  const char *ptr = reinterpret_cast<const char *>(&input_signals);
  std::vector<char> output_data =
      std::vector<char>(ptr, ptr + sizeof(CommsInputSignalsR02_02));

  udp_sender_.send(output_data);
  alive_counter_++;
}
} // namespace R02_02