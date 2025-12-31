#include "frgen21_frame_interpreter.h"
#include "R01.00/frgen21_frame_interpreter_ZX_R01.00.h"
#include "R02.00/frgen21_frame_interpreter_ZX_R02.00.h"

namespace frgen21 {

std::unique_ptr<FrameInterpreter>
FrameInterpreterFactory::createFrameInterpreter(std::string interface_type,
                                                std::string software_version) {
  if (interface_type == "ZX") {
    size_t sw_version_dot_pos = software_version.find('.');
    int8_t sw_version_major =
        std::stoul(software_version.substr(1, sw_version_dot_pos - 1));
    int8_t sw_version_minor =
        std::stoul(software_version.substr(sw_version_dot_pos + 1));
    ;

    if (sw_version_major == 1) {
      return std::make_unique<R01_00::FrameInterpreterR01_00>();
    } else if (sw_version_major >= 2) {
      return std::make_unique<R02_00::FrameInterpreterR02_00>();
    } else
      return nullptr;
  }

  if (interface_type == "XA") {
    return nullptr;
  }

  if (interface_type == "") {
    if (software_version == "") {
      return nullptr;
    }
  }

  // std::cout << interface_type << " " << software_version << " not supported"
  // << std::endl;
  return nullptr;
}

} // namespace frgen21
