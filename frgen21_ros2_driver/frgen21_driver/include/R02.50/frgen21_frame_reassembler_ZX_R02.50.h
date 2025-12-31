#ifndef FRGEN21_FRAME_REASSEMBLER_ZX_R02_50_H
#define FRGEN21_FRAME_REASSEMBLER_ZX_R02_50_H

#include <memory>
#include <optional>

#include "R02.50/ZF_R2RadarPointTargetInterface_ZX.h"
#include "frgen21_frame_reassembler.h"

namespace R02_50 {
using radar_frame_t =
    ZF_FRGen21_PTL_Interface_ZX_R02_50::struct_ZX_Itf4Eth_Primary_Data;
using payload_package_t =
    ZF_FRGen21_PTL_Interface_ZX_R02_50::Struct_ZX_Itf4Eth_Payload_Package_t;
using namespace frgen21;

const uint8_t PTL_ARC_SIZE = 8u;
const uint8_t PTL_CMAC_SIZE = 16u;
const uint8_t PTL_CS_BYTES_SIZE = PTL_ARC_SIZE + PTL_CMAC_SIZE;

class FrameReassemblerR02_50 : public FrameReassembler {
public:
  FrameReassemblerR02_50(const std::string &software_version);
  FrameStatus pushData(const std::vector<char> &data);
  std::shared_ptr<char[]> getFrame();
  std::size_t getSize();
  std::shared_ptr<radar_frame_t> getFrameAsStruct();

private:
  void prepareNewFrame();
  template <typename T>
  void processPackage(const std::vector<char> &data, T *buffer_ptr);
  void processPackage(const std::vector<char> &data,
                      payload_package_t *buffer_ptr);
  std::shared_ptr<radar_frame_t> radar_data_container_buffer_;
  std::optional<uint16_t> scan_id_;
  std::optional<uint8_t> packet_id_;
  std::size_t frame_size_;
  FrameStatus frame_status_;
};
} // namespace R02_50

#endif // FRGEN21_FRAME_REASSEMBLER_ZX_R02_50_H