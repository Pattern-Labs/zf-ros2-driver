#ifndef FRGEN21_FRAME_INTERPRETER_ZX_R02_00_H
#define FRGEN21_FRAME_INTERPRETER_ZX_R02_00_H

#include "R01.00/ZF_R1RadarPointTargetInterface.h"
#include "R01.00/ZF_R1RadarPointTargetInterface_ZX.h"
#include "R02.00/PointTargetInterfaceMapping.h"
#include "R02.00/R2PhysicalValueDerivation.h"
#include "R02.00/ZF_R2RCSRangeCorrection.h"
#include "frgen21_frame_interpreter.h"
#include <cstring>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

namespace R02_00 {
using namespace frgen21;

class FrameInterpreterR02_00 : public FrameInterpreter {
public:
  FrameInterpreterR02_00();
  std::shared_ptr<char[]>
  interpret(std::shared_ptr<char[]> &radar_frame) override;
  ZF_FRGen21_PTL_Interface_R02_00::R2_IRSP_DerivedPointTargetData_t
  interpretAsStruct(std::shared_ptr<char[]> &radar_frame);
  ZF_FRGen21_PTL_Interface_R02_00::R2_IRSP_DerivedPointTargetData_t
  interpretFromStruct(
      std::shared_ptr<
          ZF_FRGen21_PTL_Interface_ZX_R01_00::struct_ZX_Itf4Eth_Primary_Data>
          &radar_frame);

private:
  ZF_FRGen21_PTL_Interface_R02_00::R2PhysicalValueDerivation
      physical_value_derivation_;
};
} // namespace R02_00

#endif // FRGEN21_FRAME_INTERPRETER_ZX_R02_00_H