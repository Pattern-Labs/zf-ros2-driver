/*********************************************************************
 *   MODULE SPECIFICATION:
 *
 *	   PointTargetInterfaceMapping functions.
 *      Maps the Point Target Interface from
 *       'ZF_R1RadarPointTargetInterface_ZX.h'
 *      to
 *       'ZF_R1RadarPointTargetInterface.h'
 *      which is compatible with the Physical Value Derivation.
 *      Structures are mapped from network order (big endian) to
 *      little endian.
 *
 *      Refer to:
 *
 *
 *  CONFIDENTIAL AND PROPRIETARY:
 *      This copyrighted document is the property of ZF Friedrichshafen AG
 *      and its affiliates, including ZF Group TRW Automotive Inc.
 *      and its subsidiary companies ("ZF").
 *
 *      This document and the information contained herein is disclosed
 *      in confidence and may not be copied, disclosed to others,
 *      or used for any purpose unless authorized by ZF.
 *
 *      This document must be returned to ZF upon request and may not
 *      be stored in any database or retrieval system.
 *
 *      CopyrightDate: (C) ZF Friedrichshafen AG 2022.
 *
 *      Last updated: 2022-10-12.
 *
 ********************************************************************/

#include <algorithm>
#include <stdint.h>
#include <string.h>

#include "R01.00/PointTargetInterfaceMapping.h"

template <typename T> static T SwapByteOrder(T val) {
  const int num_bytes = sizeof(val);
  uint8_t *val_u8 = (uint8_t *)&val;

  for (int index_byte = 0; index_byte < num_bytes / 2U; index_byte++) {
    std::swap(val_u8[index_byte], val_u8[num_bytes - 1 - index_byte]);
  }
  return val;
}

void PointTargetInterfaceMapping(
    ZF_FRGen21_PTL_Interface_R01_00::Struct_Itf4Eth_Primary_Data_t
        *const ptr_PointTargetListOutput,
    ZF_FRGen21_PTL_Interface_ZX_R01_00::Struct_ZX_If4Eth_GenericTypes_COMPLEX_t
        *const sDynCalibCoeffs,
    const ZF_FRGen21_PTL_Interface_ZX_R01_00::struct_ZX_Itf4Eth_Primary_Data_t
        *const ptr_PointTargetListInput) {
  // context data
  ptr_PointTargetListOutput->sHeader.sContextData.InterfaceVersion =
      ptr_PointTargetListInput->sHeader.sContextData.InterfaceVersion;
  memcpy(&ptr_PointTargetListOutput->sHeader.sContextData.SensorSerialNumber[0],
         &ptr_PointTargetListInput->sHeader.sContextData.SensorSerialNumber[0],
         32ULL * sizeof(uint8_t));
  memcpy(
      &ptr_PointTargetListOutput->sHeader.sContextData.SoftwareVersion_Boot[0],
      &ptr_PointTargetListInput->sHeader.sContextData.SoftwareVersion_Boot[0],
      32ULL * sizeof(uint8_t));
  memcpy(
      &ptr_PointTargetListOutput->sHeader.sContextData.SoftwareVersion_RPU[0],
      &ptr_PointTargetListInput->sHeader.sContextData.SoftwareVersion_RPU[0],
      32ULL * sizeof(uint8_t));
  memcpy(
      &ptr_PointTargetListOutput->sHeader.sContextData.SoftwareVersion_APU[0],
      &ptr_PointTargetListInput->sHeader.sContextData.SoftwareVersion_APU[0],
      32ULL * sizeof(uint8_t));
  memcpy(&ptr_PointTargetListOutput->sHeader.sContextData
              .SoftwareVersion_Overall[0],
         &ptr_PointTargetListInput->sHeader.sContextData
              .SoftwareVersion_Overall[0],
         32ULL * sizeof(uint8_t));
  ptr_PointTargetListOutput->sHeader.sContextData.CycleCounter = SwapByteOrder(
      ptr_PointTargetListInput->sHeader.sContextData.CycleCounter);
  ptr_PointTargetListOutput->sHeader.sContextData.TimestampMeasure_Start =
      SwapByteOrder(ptr_PointTargetListInput->sHeader.sContextData
                        .TimestampMeasure_Start);
  ptr_PointTargetListOutput->sHeader.sContextData.TimestampMeasure_End =
      SwapByteOrder(
          ptr_PointTargetListInput->sHeader.sContextData.TimestampMeasure_End);
  ptr_PointTargetListOutput->sHeader.sContextData.NumberOfTargets =
      SwapByteOrder(
          ptr_PointTargetListInput->sHeader.sContextData.NumberOfTargets);
  ptr_PointTargetListOutput->sHeader.sContextData.NumberOfTTMFs = SwapByteOrder(
      ptr_PointTargetListInput->sHeader.sContextData.NumberOfTTMF);
  ptr_PointTargetListOutput->sHeader.sContextData.DataValidity = SwapByteOrder(
      ptr_PointTargetListInput->sHeader.sContextData.DataValidity);
  ptr_PointTargetListOutput->sHeader.sContextData.eGPTP_Status = SwapByteOrder(
      ptr_PointTargetListInput->sHeader.sContextData.eGPTP_Status);
  ptr_PointTargetListOutput->sHeader.sContextData.eFullTargetList =
      SwapByteOrder(
          ptr_PointTargetListInput->sHeader.sContextData.eFullTargetList);
  ptr_PointTargetListOutput->sHeader.sContextData.Reserved =
      SwapByteOrder(ptr_PointTargetListInput->sHeader.sContextData.Reserved);
  ptr_PointTargetListOutput->sHeader.sContextData.fEffectiveRange =
      SwapByteOrder(
          ptr_PointTargetListInput->sHeader.sContextData.fEffectiveRange);
  ptr_PointTargetListOutput->sHeader.sContextData.IndexTemperature =
      SwapByteOrder(
          ptr_PointTargetListInput->sHeader.sContextData.IndexTemperature);
  ptr_PointTargetListOutput->sHeader.sContextData.fCenterFrequencyStep =
      SwapByteOrder(
          ptr_PointTargetListInput->sHeader.sContextData.fCenterFrequencyStep);
  ptr_PointTargetListOutput->sHeader.sContextData.fAdcSamplingInterval =
      SwapByteOrder(
          ptr_PointTargetListInput->sHeader.sContextData.fAdcSamplingInterval);

  // dynamic config parameters
  ptr_PointTargetListOutput->sHeader.sDynamicConfigParams.fEffectiveBandwidth =
      SwapByteOrder(ptr_PointTargetListInput->sHeader.sDynamicConfigParams
                        .fEffectiveBandwidth);
  ptr_PointTargetListOutput->sHeader.sDynamicConfigParams.fPulseRepetitionTime =
      SwapByteOrder(ptr_PointTargetListInput->sHeader.sDynamicConfigParams
                        .fPulseRepetitionTime);
  ptr_PointTargetListOutput->sHeader.sDynamicConfigParams.fCarrierFrequency =
      SwapByteOrder(ptr_PointTargetListInput->sHeader.sDynamicConfigParams
                        .fCarrierFrequency);
  ptr_PointTargetListOutput->sHeader.sDynamicConfigParams
      .fNormalizedSignalPower =
      SwapByteOrder(ptr_PointTargetListInput->sHeader.sDynamicConfigParams
                        .fNormalizedSignalPower);
  ptr_PointTargetListOutput->sHeader.sDynamicConfigParams.fWindowLoss =
      SwapByteOrder(
          ptr_PointTargetListInput->sHeader.sDynamicConfigParams.fWindowLoss);

  // antenna gain
  for (int i_az = 0; i_az < 181; i_az++) {
    ptr_PointTargetListOutput->sHeader.sAntennaElementGain.Azimuth[i_az] =
        SwapByteOrder(ptr_PointTargetListInput->sHeader.sAntennaElementGain
                          .Azimuth[i_az]);
  }
  for (int i_el = 0; i_el < 37; i_el++) {
    ptr_PointTargetListOutput->sHeader.sAntennaElementGain.Elevation[i_el] =
        SwapByteOrder(ptr_PointTargetListInput->sHeader.sAntennaElementGain
                          .Elevation[i_el]);
  }

  // jamming status
  ptr_PointTargetListOutput->sHeader.sBlindDetection.JammingStatus =
      SwapByteOrder(
          ptr_PointTargetListInput->sHeader.sBlindDetection.JammingStatus);

  // convert all packages including
  // Struct_ZX_Itf4Eth_Payload_RemainingTargetsAndNoise_t
  int i_tar_out = 0;
  for (int i_pkg = 0;
       i_pkg < ZF_FRGen21_PTL_Interface_ZX_R01_00::NumberOfTargetPackages + 1;
       i_pkg++) {
    // check which kind of package is processed
    const ZF_FRGen21_PTL_Interface_ZX_R01_00::Struct_ZX_Itf4Eth_TargetType_t
        *sTargetList;
    int numTargets = 0;
    if (i_pkg < ZF_FRGen21_PTL_Interface_ZX_R01_00::NumberOfTargetPackages) {
      // case: standard target package
      numTargets = ZF_FRGen21_PTL_Interface_ZX_R01_00::SizeOfStdTargetPackage;
      sTargetList =
          &ptr_PointTargetListInput->sTargetPackages[i_pkg].sTargets[0];
    } else if (i_pkg ==
               ZF_FRGen21_PTL_Interface_ZX_R01_00::NumberOfTargetPackages) {
      // case: last package with targets and noise
      numTargets = ZF_FRGen21_PTL_Interface_ZX_R01_00::SizeOfRemainingTargets;
      sTargetList = &ptr_PointTargetListInput->sRemainingTargetsAndNoisePackage
                         .sTargets[0];
    }

    // process all targets of the package
    for (int i_tar_in = 0; i_tar_in < numTargets; i_tar_in++) {
      ptr_PointTargetListOutput->sPayload.sTargets[i_tar_out].sFrequency.Range =
          SwapByteOrder(sTargetList[i_tar_in].sFrequency.Range);
      ptr_PointTargetListOutput->sPayload.sTargets[i_tar_out]
          .sFrequency.Velocity =
          SwapByteOrder(sTargetList[i_tar_in].sFrequency.Velocity);
      ptr_PointTargetListOutput->sPayload.sTargets[i_tar_out]
          .sFrequency.Azimuth =
          SwapByteOrder(sTargetList[i_tar_in].sFrequency.Azimuth);
      ptr_PointTargetListOutput->sPayload.sTargets[i_tar_out]
          .sFrequency.Elevation =
          SwapByteOrder(sTargetList[i_tar_in].sFrequency.Elevation);
      // assume 8 bit members for MSE structure
      memcpy(&ptr_PointTargetListOutput->sPayload.sTargets[i_tar_out].sMSE,
             &sTargetList[i_tar_in].sMSE,
             sizeof(ZF_FRGen21_PTL_Interface_R01_00::Struct_Itf4Eth_MSE));
      // assume 8 bit members for AAR structure
      // Note: bitfield substructure does not matter for byte order, as long as
      // the base type is 8 bit
      memcpy(&ptr_PointTargetListOutput->sPayload.sTargets[i_tar_out].sAAR,
             &sTargetList[i_tar_in].sAAR,
             sizeof(ZF_FRGen21_PTL_Interface_R01_00::Struct_Itf4Eth_AAR_Index));
      ptr_PointTargetListOutput->sPayload.sTargets[i_tar_out].SNR =
          SwapByteOrder(sTargetList[i_tar_in].SNR);
      ptr_PointTargetListOutput->sPayload.sTargets[i_tar_out].Power =
          SwapByteOrder(sTargetList[i_tar_in].Power);
      ptr_PointTargetListOutput->sPayload.sTargets[i_tar_out].ModelType =
          SwapByteOrder(sTargetList[i_tar_in].ModelType);
      ptr_PointTargetListOutput->sPayload.sTargets[i_tar_out]
          .DetectionConfidence =
          SwapByteOrder(sTargetList[i_tar_in].DetectionConfidence);
      ptr_PointTargetListOutput->sPayload.sTargets[i_tar_out].PeakID =
          SwapByteOrder(sTargetList[i_tar_in].PeakID);
      // next target
      i_tar_out++;
    }
  }

  // noise data
  ptr_PointTargetListOutput->sPayload.sNoise.Offset =
      SwapByteOrder(ptr_PointTargetListInput->sRemainingTargetsAndNoisePackage
                        .sHRNoise.Offset);
  ptr_PointTargetListOutput->sPayload.sNoise.DynamicRange =
      SwapByteOrder(ptr_PointTargetListInput->sRemainingTargetsAndNoisePackage
                        .sHRNoise.DynamicRange);
  memcpy(&ptr_PointTargetListOutput->sPayload.sNoise.NoiseValues[0],
         &ptr_PointTargetListInput->sRemainingTargetsAndNoisePackage.sHRNoise
              .NoiseValues[0],
         ZF_FRGen21_PTL_Interface_R01_00::SizeOfNoiseLevels * sizeof(uint8_t));

  ////online phase calibration co-efficients
  // online phase calibration co-efficients
  for (int i_pkg = 0;
       i_pkg < ZF_FRGen21_PTL_Interface_ZX_R01_00::SizeOfBlockagePackages;
       i_pkg++) {
    // check which kind of package is processed
    const ZF_FRGen21_PTL_Interface_ZX_R01_00::
        Struct_ZX_If4Eth_GenericTypes_COMPLEX_t *sDynCalibCoeffList;
    sDynCalibCoeffList =
        &ptr_PointTargetListInput->sBlockageDynCoeffs[i_pkg].sDynCalibCoeffs[0];

    for (int i_coff_in = 0;
         i_coff_in <
         ZF_FRGen21_PTL_Interface_ZX_R01_00::SizeOfBlockagePackagesValues;
         i_coff_in++) {
      sDynCalibCoeffs[i_coff_in + i_pkg * ZF_FRGen21_PTL_Interface_ZX_R01_00::
                                              SizeOfBlockagePackagesValues]
          .real = SwapByteOrder(sDynCalibCoeffList[i_coff_in].real);
      sDynCalibCoeffs[i_coff_in + i_pkg * ZF_FRGen21_PTL_Interface_ZX_R01_00::
                                              SizeOfBlockagePackagesValues]
          .imag = SwapByteOrder(sDynCalibCoeffList[i_coff_in].imag);
    }
  }
}