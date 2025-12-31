#include "R02.00/RxSignalHandler_Cfg_Cmp_R02.00.h"
#include "frgen21_input_signal_handler.h"
#include "udp_com.h"

namespace R02_00 {
using namespace frgen21;

class InputSignalHandlerR02_00 : public InputSignalHandler {
public:
  InputSignalHandlerR02_00(const std::string &remote_ip, int remote_port,
                           int local_port);
  void set_mode(uint8_t range_mode) override;

private:
  const std::string remote_ip_;
  int remote_port_;
  int local_port_;
  UDPSender udp_sender_;
  uint8_t alive_counter_;
};

} // namespace R02_00