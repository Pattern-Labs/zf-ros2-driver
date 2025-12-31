#include "R01.00/frgen21_frame_reassembler_ZX_R01.00.h"

namespace R01_00 {
FrameReassemblerR01_00::FrameReassemblerR01_00(
    const std::string &software_version)
    : FrameReassembler("ZX", software_version),
      radar_data_container_buffer_(std::make_shared<radar_frame_t>()),
      frame_size_(0), frame_status_(FrameStatus::NONE) {
  prepareNewFrame();
}

void FrameReassemblerR01_00::prepareNewFrame() {
  scan_id_.reset();
  packet_id_.reset();
  frame_size_ = 0;
  frame_status_ = FrameStatus::NONE;
  // radar_data_container_buffer_.reset();
}

template <typename T>
void FrameReassemblerR01_00::processPackage(const std::vector<char> &data,
                                            T *buffer_ptr) {
  int data_length = data.size();
  const T *tmp_package;
  tmp_package = reinterpret_cast<const T *>(data.data());
  auto crc_value = tmp_package->CRC;

  if (!isCRC32valid(data.data(), sizeof(T) - sizeof(T::CRC),
                    &crc_value)) {
    frame_status_ = FrameStatus::CRC_ERROR;
    return;
  }

  if (scan_id_.has_value() && tmp_package->ScanID != scan_id_.value()) {
    prepareNewFrame();
    frame_status_ = FrameStatus::LAST_FRAME_INCOMPLETE;
  }

  std::memcpy(buffer_ptr, data.data(), data_length);
  frame_size_ += data_length;
  auto scan_id = tmp_package->ScanID;
  scan_id_ = std::make_optional(scan_id);
  auto packet_id = tmp_package->PacketID;
  packet_id_ = std::make_optional(packet_id);
}

void FrameReassemblerR01_00::processPackage(const std::vector<char> &data,
                                            payload_package_t *buffer_ptr) {
  int data_length = data.size();
  const payload_package_t *tmp_package;
  tmp_package = reinterpret_cast<const payload_package_t *>(data.data());
  auto crc_value = tmp_package->CRC;

  if (!isCRC32valid(data.data(),
                    sizeof(payload_package_t) - sizeof(payload_package_t::CRC),
                    &crc_value)) {
    frame_status_ = FrameStatus::CRC_ERROR;
    return;
  }

  if (scan_id_.has_value() && tmp_package->ScanID != scan_id_.value()) {
    prepareNewFrame();
    frame_status_ = FrameStatus::LAST_FRAME_INCOMPLETE;
  }

  uint8_t position = tmp_package->PacketID - 1;
  if (position <= ZF_FRGen21_PTL_Interface_ZX_R01_00::NumberOfTargetPackages) {
    std::memcpy(buffer_ptr + position, data.data(), data_length);
    frame_size_ += data_length;
    auto scan_id = tmp_package->ScanID;
    scan_id_ = std::make_optional(scan_id);
    auto packet_id = tmp_package->PacketID;
    packet_id_ = std::make_optional(packet_id);
  } else
    frame_status_ = FrameStatus::FRAME_OVERFLOW;
}

FrameStatus FrameReassemblerR01_00::pushData(const std::vector<char> &data) {
  int data_length = data.size();

  if (frame_status_ == FrameStatus::LAST_FRAME_INCOMPLETE) {
    // new frame already created in processPackage() and package added
    frame_status_ = FrameStatus::NONE;
  } else if (frame_status_ != FrameStatus::NONE) {
    prepareNewFrame();
  }

  switch (data_length) {
  // header: 672
  case sizeof(ZF_FRGen21_PTL_Interface_ZX_R01_00::Struct_ZX_Itf4Eth_Header_t):
    processPackage(data, &radar_data_container_buffer_->sHeader);
    // check if sw version has changed
    if (frame_status_ == FrameStatus::NONE ||
        frame_status_ == FrameStatus::LAST_FRAME_INCOMPLETE) {
      std::string software_version(reinterpret_cast<const char *>(
          const_cast<uint8_t *>(radar_data_container_buffer_->sHeader
                                    .sContextData.SoftwareVersion_Overall)));
      if (software_version_ != software_version) {
        frame_status_ = FrameStatus::SW_VERSION_CHANGED;
        software_version_ = software_version;
      }
    }
    break;

  // payload package: 1438
  case sizeof(payload_package_t):
    processPackage(data, &radar_data_container_buffer_->sTargetPackages[0]);
    break;

  // remaining targets with noise: 1408
  case sizeof(ZF_FRGen21_PTL_Interface_ZX_R01_00::
                  Struct_ZX_Itf4Eth_Payload_RemainingTargetsAndNoise_t):
    processPackage(
        data, &radar_data_container_buffer_->sRemainingTargetsAndNoisePackage);
    break;

  // blockage_coeff: 776
  case sizeof(ZF_FRGen21_PTL_Interface_ZX_R01_00::
                  Struct_ZX_Itf4Eth_Payload_Blockage_Dyn_Coeff_t):
    processPackage(data, &radar_data_container_buffer_->sBlockageDynCoeffs[0]);
    break;

  // blockage_coeff + AntiReplayCounter + CRC: 800
  case sizeof(ZF_FRGen21_PTL_Interface_ZX_R01_00::
                  Struct_ZX_Itf4Eth_Payload_Blockage_Dyn_Coeff_t) +
      PTL_CS_BYTES_SIZE:
    processPackage(data, &radar_data_container_buffer_->sBlockageDynCoeffs[1]);
    break;

  default:
    break;
  }

  // return frame status
  if (frame_status_ == FrameStatus::NONE) {
    if (frame_size_ == sizeof(radar_frame_t)) {
      frame_status_ = FrameStatus::FRAME_COMPLETE;
    } else if (frame_size_ >= sizeof(radar_frame_t)) {
      frame_status_ = FrameStatus::FRAME_OVERFLOW;
    }
  }
  return frame_status_;
}

std::shared_ptr<radar_frame_t> FrameReassemblerR01_00::getFrameAsStruct() {
  if (frame_status_ == FrameStatus::FRAME_COMPLETE) {
    auto tmp_return = radar_data_container_buffer_;
    prepareNewFrame();

    return tmp_return;
  } else
    return nullptr;
}

std::shared_ptr<char[]> FrameReassemblerR01_00::getFrame() {
  if (frame_status_ == FrameStatus::FRAME_COMPLETE) {
    char *tmp_package =
        reinterpret_cast<char *>(radar_data_container_buffer_.get());
    std::size_t size = sizeof(radar_frame_t);

    std::shared_ptr<char[]> char_array(new char[size]);
    std::copy(tmp_package, tmp_package + size, char_array.get());
    prepareNewFrame();

    return char_array;

  } else
    return nullptr;
}

std::size_t FrameReassemblerR01_00::getSize() { return sizeof(radar_frame_t); }

} // namespace R01_00