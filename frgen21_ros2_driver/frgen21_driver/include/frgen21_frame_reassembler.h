#ifndef FRGEN21_FRAME_REASSEMBLER_H
#define FRGEN21_FRAME_REASSEMBLER_H

#include <boost/crc.hpp>
#include <byteswap.h>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace frgen21 {

enum class FrameStatus {
  // Ready to receive a package or received package was added to the current
  // sensor frame
  NONE,
  // Frame succesfuly reassembled and ready to read
  FRAME_COMPLETE,
  // Frame got discarded and a new frame is reassembled since package with new
  // scan ID was received
  LAST_FRAME_INCOMPLETE,
  // Received data overflows size of the frame
  FRAME_OVERFLOW,
  // Received Package has wrong CRC, Frame gets discarded
  CRC_ERROR,
  REASSEMBLER_NOT_INITIALIZED,
  REASSEMBLER_FOUND_VERSION,
  // software version changed, last frame can be incomplete
  SW_VERSION_CHANGED
};

class FrameReassembler {
public:
  FrameReassembler(const std::string &interface_type,
                   const std::string &software_version);
  virtual FrameStatus pushData(const std::vector<char> &data) = 0;
  virtual std::shared_ptr<char[]> getFrame() = 0;
  virtual std::size_t getSize() = 0;
  virtual bool isCRC32valid(const char *data, std::size_t data_length,
                            const uint32_t *crc);
  virtual ~FrameReassembler() = default;
  std::string getInterfaceType();
  std::string getSoftwareVersion();

protected:
  std::string interface_type_;
  std::string software_version_;
};

class FrameReassemblerDefault : public FrameReassembler {
public:
  FrameReassemblerDefault(const std::string &interface_type,
                          const std::string &software_version);
  FrameStatus pushData(const std::vector<char> &data);
  std::shared_ptr<char[]> getFrame();
  std::size_t getSize();
};

class FrameReassemblerFactory {
public:
  static std::unique_ptr<FrameReassembler>
  createFrameReassembler(std::string interface_type = "",
                         std::string software_version = "");
};

} // namespace frgen21

#endif // FRGEN21_FRAME_REASSEMBLER_H