#ifndef FRGEN21_FRAME_INTERPRETER_H
#define FRGEN21_FRAME_INTERPRETER_H

#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace frgen21 {

class FrameInterpreter {
public:
  virtual std::shared_ptr<char[]>
  interpret(std::shared_ptr<char[]> &radar_frame) = 0;
};

class FrameInterpreterFactory {
public:
  static std::unique_ptr<FrameInterpreter>
  createFrameInterpreter(std::string interface_type = "",
                         std::string software_version = "");
};

} // namespace frgen21

#endif // FRGEN21_FRAME_INTERPRETER_H