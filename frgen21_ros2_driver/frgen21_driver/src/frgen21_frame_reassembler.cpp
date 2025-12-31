#include "frgen21_frame_reassembler.h"
#include "R01.00/frgen21_frame_reassembler_ZX_R01.00.h"
#include "R02.50/frgen21_frame_reassembler_ZX_R02.50.h"

using namespace frgen21;

FrameReassembler::FrameReassembler(const std::string &interface_type,
                                   const std::string &software_version)
    : interface_type_(interface_type), software_version_(software_version) {}

std::string FrameReassembler::getInterfaceType() { return interface_type_; }

std::string FrameReassembler::getSoftwareVersion() { return software_version_; }

bool FrameReassembler::isCRC32valid(const char *data, std::size_t data_length,
                                    const uint32_t *crc) {
  boost::crc_32_type crc_calculated;
  crc_calculated.process_bytes(data, data_length);
  return crc_calculated.checksum() == bswap_32(*crc);
}

FrameReassemblerDefault::FrameReassemblerDefault(
    const std::string &interface_type, const std::string &software_version)
    : FrameReassembler(interface_type, software_version) {}

FrameStatus FrameReassemblerDefault::pushData(const std::vector<char> &data) {
  int data_length = data.size();

  // ZX
  if (data_length < 1500) {
    interface_type_ = "ZX";
    if (data_length ==
        sizeof(
            ZF_FRGen21_PTL_Interface_ZX_R01_00::Struct_ZX_Itf4Eth_Header_t)) {
      const ZF_FRGen21_PTL_Interface_ZX_R01_00::Struct_ZX_Itf4Eth_Header_t
          *tmp_package;
      tmp_package =
          reinterpret_cast<const ZF_FRGen21_PTL_Interface_ZX_R01_00::
                               Struct_ZX_Itf4Eth_Header_t *>(data.data());
      std::string software_version(
          reinterpret_cast<const char *>(const_cast<uint8_t *>(
              tmp_package->sContextData.SoftwareVersion_Overall)));
      software_version_ = software_version;
    }
  }
  // XA
  else {
    interface_type_ = "XA";
  }

  if (!interface_type_.empty() && !software_version_.empty()) {
    return FrameStatus::REASSEMBLER_FOUND_VERSION;
  }

  return FrameStatus::REASSEMBLER_NOT_INITIALIZED;
}

std::shared_ptr<char[]> FrameReassemblerDefault::getFrame() { return nullptr; }

std::size_t FrameReassemblerDefault::getSize() { return 0; }

std::unique_ptr<FrameReassembler>
FrameReassemblerFactory::createFrameReassembler(std::string interface_type,
                                                std::string software_version) {
  if (interface_type == "ZX") {
    size_t sw_version_dot_pos = software_version.find('.');
    int8_t sw_version_major =
        std::stoul(software_version.substr(1, sw_version_dot_pos - 1));
    int8_t sw_version_minor =
        std::stoul(software_version.substr(sw_version_dot_pos + 1));

    if (sw_version_major == 1) {
      return std::make_unique<R01_00::FrameReassemblerR01_00>(software_version);
    } else if (sw_version_major == 2) {
      if (sw_version_minor >= 50) {
        return std::make_unique<R02_50::FrameReassemblerR02_50>(
            software_version);
      }
      return std::make_unique<R01_00::FrameReassemblerR01_00>(software_version);
    } else if (sw_version_major >= 3) {
      return std::make_unique<R02_50::FrameReassemblerR02_50>(software_version);
    } else
      return std::make_unique<FrameReassemblerDefault>(interface_type,
                                                       software_version);
  }

  if (interface_type == "XA") {
    return std::make_unique<FrameReassemblerDefault>(interface_type,
                                                     software_version);
  }

  if (interface_type == "") {
    if (software_version == "") {
      return std::make_unique<FrameReassemblerDefault>(interface_type,
                                                       software_version);
    }
  }

  return nullptr;
}
