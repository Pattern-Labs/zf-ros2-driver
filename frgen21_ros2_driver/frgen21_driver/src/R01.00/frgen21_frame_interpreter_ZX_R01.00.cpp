#include "R01.00/frgen21_frame_interpreter_ZX_R01.00.h"

namespace R01_00 {
FrameInterpreterR01_00::FrameInterpreterR01_00() {}

std::shared_ptr<char[]>
FrameInterpreterR01_00::interpret(std::shared_ptr<char[]> &radar_frame) {
  const ZF_FRGen21_PTL_Interface_ZX_R01_00::struct_ZX_Itf4Eth_Primary_Data_t
      *tmp_package;
  tmp_package = reinterpret_cast<
      const ZF_FRGen21_PTL_Interface_ZX_R01_00::struct_ZX_Itf4Eth_Primary_Data_t
          *>(radar_frame.get());

  std::unique_ptr<ZF_FRGen21_PTL_Interface_R01_00::Struct_Itf4Eth_Primary_Data>
      ptrPrimaryData = std::make_unique<
          ZF_FRGen21_PTL_Interface_R01_00::Struct_Itf4Eth_Primary_Data>();
  ZF_FRGen21_PTL_Interface_ZX_R01_00::Struct_ZX_If4Eth_GenericTypes_COMPLEX_t
      sDynCalib;

  PointTargetInterfaceMapping(ptrPrimaryData.get(), &sDynCalib, tmp_package);

  ZF_FRGen21_PTL_Interface_R01_00::R1_IRSP_DerivedPointTargetData_t
      target_values;
  physical_value_derivation_.CalculateDerivedTargetValues(ptrPrimaryData.get(),
                                                          &target_values);

  std::size_t size =
      sizeof(ZF_FRGen21_PTL_Interface_R01_00::R1_IRSP_DerivedPointTargetData_t);
  std::shared_ptr<char[]> char_array(new char[size]);
  // std::copy(tmp_package, tmp_package + size, char_array.get());
  char *tmp_package2 = reinterpret_cast<char *>(&target_values);
  std::copy(tmp_package2, tmp_package2 + size, char_array.get());
  return char_array;
}

ZF_FRGen21_PTL_Interface_R01_00::R1_IRSP_DerivedPointTargetData_t
FrameInterpreterR01_00::interpretAsStruct(
    std::shared_ptr<char[]> &radar_frame) {
  const ZF_FRGen21_PTL_Interface_ZX_R01_00::struct_ZX_Itf4Eth_Primary_Data_t
      *tmp_package;
  tmp_package = reinterpret_cast<
      const ZF_FRGen21_PTL_Interface_ZX_R01_00::struct_ZX_Itf4Eth_Primary_Data_t
          *>(radar_frame.get());

  std::unique_ptr<ZF_FRGen21_PTL_Interface_R01_00::Struct_Itf4Eth_Primary_Data>
      ptrPrimaryData = std::make_unique<
          ZF_FRGen21_PTL_Interface_R01_00::Struct_Itf4Eth_Primary_Data>();
  ZF_FRGen21_PTL_Interface_ZX_R01_00::Struct_ZX_If4Eth_GenericTypes_COMPLEX_t
      sDynCalib;

  PointTargetInterfaceMapping(ptrPrimaryData.get(), &sDynCalib, tmp_package);

  ZF_FRGen21_PTL_Interface_R01_00::R1_IRSP_DerivedPointTargetData_t
      target_values;
  physical_value_derivation_.CalculateDerivedTargetValues(ptrPrimaryData.get(),
                                                          &target_values);
  return target_values;
}

ZF_FRGen21_PTL_Interface_R01_00::R1_IRSP_DerivedPointTargetData_t
FrameInterpreterR01_00::interpretFromStruct(
    std::shared_ptr<
        ZF_FRGen21_PTL_Interface_ZX_R01_00::struct_ZX_Itf4Eth_Primary_Data>
        &radar_frame) {
  std::unique_ptr<ZF_FRGen21_PTL_Interface_R01_00::Struct_Itf4Eth_Primary_Data>
      ptrPrimaryData = std::make_unique<
          ZF_FRGen21_PTL_Interface_R01_00::Struct_Itf4Eth_Primary_Data>();
  ZF_FRGen21_PTL_Interface_ZX_R01_00::Struct_ZX_If4Eth_GenericTypes_COMPLEX_t
      sDynCalib;
  PointTargetInterfaceMapping(ptrPrimaryData.get(), &sDynCalib,
                              radar_frame.get());

  ZF_FRGen21_PTL_Interface_R01_00::R1_IRSP_DerivedPointTargetData_t
      target_values;
  physical_value_derivation_.CalculateDerivedTargetValues(ptrPrimaryData.get(),
                                                          &target_values);
  return target_values;
}

} // namespace R01_00
