/*********************************************************************
 *   MODULE SPECIFICATION:
 *
 *      ZF Radar Point Target Interface - Primary Point Targets (Frame internal
 *and Point Target List internal to be mapped to customer interface)
 *
 *      Refer to:
 *         url:integrity://skobde-mks.kobde.trw.com:7001/si/viewhistory?project=/DAS/040%5fTest%5fProjects/%3cFRGen21%3e%20Full%20Range%20Radar%20Generation%2021/045%5fArchitecture/010%5fSystem/000%5fSystem%5fArchitecture/project.pj&selection=%22point%5ftarget%5flist%5finterface.pdf%22
 *
 *         url:integrity://skobde-mks.kobde.trw.com:7001/si/viewhistory?project=/DAS/040%5fTest%5fProjects/%3cFRGen21%3e%20Full%20Range%20Radar%20Generation%2021/060%5fSoftware%5fConstruction/060%5fIntegration/BuildViews%5fCore/030%5fIntegration/Algorithm/ZZZ%5fBuild%5fProducts/project.pj&selection=%22EthCompressedInterfaceDefinition.xlsx%22
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
 *      CopyrightDate: (C) ZF Friedrichshafen AG 2018-2022.
 *
 *      Last updated: 2022-10-12.
 *
 ********************************************************************/

#pragma once

#include <stdint.h>

#ifdef __cplusplus
// only need to export C interface if used by C++ source code
extern "C" {
#endif

#if defined(__GNUC__)
/* GCC */
#define TYPEDEF_PACKED_BEGIN(type, tag) typedef type __attribute__((packed)) tag
#define TYPEDEF_PACKED_END
#elif defined(_MSC_VER)
/* MS Visual Studio */
#define TYPEDEF_PACKED_BEGIN(type, tag) __pragma(pack(1)) typedef type tag
#define TYPEDEF_PACKED_END
// Set all structures to single byte packed/aligned to prevent
// mis-interpretation across different compilers.
#pragma pack(push, 1)
#endif

namespace ZF_FRGen21_PTL_Interface_ZX_R01_00 {
constexpr size_t NumberOfTargetPackages = 44ULL;
constexpr size_t SizeOfStdTargetPackage = 65ULL;
constexpr size_t SizeOfRemainingTargets = 40ULL;
constexpr size_t SizeOfNoiseLevels = 512ULL;
constexpr size_t SizeOfTargetList =
    SizeOfStdTargetPackage * NumberOfTargetPackages + SizeOfRemainingTargets;

constexpr size_t SizeOfZXNumRX = 12ULL;
constexpr size_t SizeOfZXNumTX = 16ULL;

constexpr size_t SizeOfBlockagePackages = 2ULL;
constexpr size_t SizeOfBlockagePackagesValues =
    SizeOfZXNumRX * SizeOfZXNumTX / SizeOfBlockagePackages;

/*
 * Enum of the gPTP Status /copied from Vector Stack
 */
TYPEDEF_PACKED_BEGIN(enum, E_ZX_ITF4EETH_GPTP_STATUS){
    E_ZX_ITF4EETH_GPTP_TIMEOUT_MASK = 0x1,
    E_ZX_ITF4EETH_GPTP_SYNC_TO_GW_SHIFT = 0x2,
    E_ZX_ITF4EETH_GPTP_SYNC_TO_GW_MASK = 0x4,
    E_ZX_ITF4EETH_GPTP_GLOBAL_TIME_BASE_MASK = 0x8,
    E_ZX_ITF4EETH_GPTP_TIMELEAP_FUTURE_MASK = 0x10,
    E_ZX_ITF4EETH_GPTP_TIMELEAP_PAST_MASK = 0x20} E_ZX_ITF4EETH_GPTP_STATUS;
TYPEDEF_PACKED_END

/**
 * Enum of Full Peak computed
 */
TYPEDEF_PACKED_BEGIN(enum, E_ZX_ITF4EETH_FULL_PEAK_LIST){
    E_ZX_ITF4EETH_PEAK_LIST_NOT_FINISHED = 0,
    E_ZX_ITF4EETH_PEAK_LIST_OVERFLOW_PROTECTION,
    E_ZX_ITF4EETH_PEAK_LIST_FINISHED,
    E_ZX_ITF4EETH_PEAK_LIST_NON_RADIATING} E_ZX_ITF4EETH_FULL_PEAK_LIST;
TYPEDEF_PACKED_END

/**
 * Enum of Model Type
 */
TYPEDEF_PACKED_BEGIN(enum, E_ZX_ITF4EETH_MODEL_TYPE){
    E_ZX_ITF4EETH_MODEL_TYPE_SINGLE_TARGET = 0U, /**< STMF only */
    E_ZX_ITF4EETH_MODEL_TYPE_SINGLE_TARGET_FALLBACK =
        1U, /**< TTMF tried, but not successful*/
    E_ZX_ITF4EETH_MODEL_TYPE_DOUBLE_TARGET =
        2U,                                /**< TTMF tried, 2 tragets found */
    E_ZX_ITF4EETH_MODEL_TYPE_UNKNOWN = 3U, /**< error value*/
    E_ZX_ITF4EETH_MODEL_TYPE_FLAG_PARTIAL_PEAK =
        8U /**< flag indicating no high - resolution model fitting in range */
} E_ZX_ITF4EETH_MODEL_TYPE;
TYPEDEF_PACKED_END

/**
 * Frequency structure for 4D coordinates and their standard deviation
 * Note: the min/max values of the Elevation member correspond to the
 * valid range of the data type, not the field-of-view
 */
TYPEDEF_PACKED_BEGIN(struct, Struct_ZX_Itf4Eth_Frequency) {
  uint16_t Range;    /**< Range Value           Min: 0 Max: 2*pi */
  uint16_t Velocity; /**< Velocity Value        Min: 0 Max: 2*pi */
  int16_t Azimuth;   /**< Azimuth Angle Value   Min: -5*pi Max: 5*pi*/
  int16_t Elevation; /**< Elevation Angle Value Min: -11*pi Max: 11*pi*/
}
Struct_ZX_Itf4Eth_Frequency_t;
TYPEDEF_PACKED_END

/**
 * MSE - Mean square error
 */
TYPEDEF_PACKED_BEGIN(struct, Struct_ZX_Itf4Eth_MSE) {
  uint8_t Range;           /**< MSE Range Value           Min: -60dB Max: 0dB */
  uint8_t Velocity;        /**< MSE Velocity Value        Min: -60dB Max: 0dB */
  uint8_t Azimuth;         /**< MSE Azimuth Angle Value   Min: -60dB Max: 0dB */
  uint8_t Elevation;       /**< MSE Elevation Angle Value Min: -60dB Max: 0dB */
  uint8_t SubArray;        /**< MSE Sub Array Value       Min: -60dB Max: 0dB */
  uint8_t SubArray2ndBest; /**< MSE Sub Array Value       Min: -60dB Max: 0dB */
}
Struct_ZX_Itf4Eth_MSE_t;
TYPEDEF_PACKED_END

/**
 * Structure containing the flattened index of the 2nd and 3rd best AAR
 * solutions
 */
TYPEDEF_PACKED_BEGIN(struct, Struct_ZX_Itf4Eth_AAR_Index) {
  uint8_t AAR_Subarray_2nd : 4; /**< Index of 2nd best angular ambiguity
                                   resolution */
  uint8_t AAR_Subarray_3rd : 4; /**< Index of 3rd best angular ambiguity
                                   resolution */
}
Struct_ZX_Itf4Eth_AAR_Index_t;
TYPEDEF_PACKED_END

/**
 * Structure of Taget Type
 */
TYPEDEF_PACKED_BEGIN(struct, Struct_ZX_Itf4Eth_TargetType) {
  Struct_ZX_Itf4Eth_Frequency_t sFrequency; /**< Structure of Frequency */
  Struct_ZX_Itf4Eth_MSE_t sMSE; /**< Structure of Mean-Squared fitting Error */
  Struct_ZX_Itf4Eth_AAR_Index_t sAAR; /**< 2nd and 3rd best AAR solution */
  uint8_t SNR;                 /**< Signal-to-Noise Ratio Min: 0dB Max: 60dB */
  uint16_t Power;              /**< Target Power */
  uint8_t ModelType;           /**< Enum of Model Type */
  uint8_t DetectionConfidence; /**< Detection Confidence  */
  uint16_t PeakID; /**< Peak ID of the target, not needed, but helps for
                      debugging purpuses */
}
Struct_ZX_Itf4Eth_TargetType_t;
TYPEDEF_PACKED_END

TYPEDEF_PACKED_BEGIN(struct, Struct_ZX_Itf4Eth_Dynamic_ConfigParams) {
  float fEffectiveBandwidth;  /**< Bandwidth of the radar for the current cycle
                                 -> required to compute range */
  float fPulseRepetitionTime; /**< PRT for the current radar cycle -> required
                                 to compute radial velocity*/
  float
      fCarrierFrequency; /**< The carrier Frequency of the radar for the current
                            radar cycle -> may change in case of jamming */
  float fNormalizedSignalPower; /**< for computation of RCS */
  float fWindowLoss;            /**< for computation of RCS */
}
Struct_ZX_Itf4Eth_Dynamic_ConfigParams_t;
TYPEDEF_PACKED_END

TYPEDEF_PACKED_BEGIN(struct, Struct_ZX_Itf4Eth_AntennaElementGain) {
  uint16_t Azimuth[181];  /**< Antenna Element Gain in Azimuth */
  uint16_t Elevation[37]; /**< Antenna Element Gain in Elevation */
}
Struct_ZX_Itf4Eth_AntennaElementGain_t;
TYPEDEF_PACKED_END

TYPEDEF_PACKED_BEGIN(struct, Struct_ZX_Itf4Eth_ContextData) {
  uint32_t InterfaceVersion;           /**< Version Information */
  uint8_t SensorSerialNumber[32];      /**< Serial Number of the device  */
  uint8_t SoftwareVersion_Boot[32];    /**< Softwareversion Bootloader */
  uint8_t SoftwareVersion_RPU[32];     /**< Softwareversion R5 */
  uint8_t SoftwareVersion_APU[32];     /**< Softwareversion A53 */
  uint8_t SoftwareVersion_Overall[32]; /**< Softwareversion Overall */
  uint32_t CycleCounter;               /**< Counter of the ethernet frame */
  uint64_t TimestampMeasure_Start;     /**< Time stamp of starting the chirp */
  uint64_t TimestampMeasure_End;       /**< Endtime of the chirp */
  uint16_t NumberOfTargets;            /**< Numer of targets detected */
  uint16_t NumberOfTTMF;               /**< Number of TTMF */
  uint8_t
      DataValidity; /**< Validity of the Data: 0: data invalid, 1: data valid*/
  uint8_t eGPTP_Status;    /**< Synchronized Status of the STBM from R5       */
  uint8_t eFullTargetList; /**< Flag for peak list completion
                              (E_ZX_ITF4EETH_FULL_PEAK_LIST as 8 bit) */
  uint8_t Reserved;        /**< Dummy reserved */
  float fEffectiveRange;   /**< Maximum calculated range of the targets */
  uint16_t IndexTemperature;  /**< IndexTemperature */
  float fCenterFrequencyStep; /**< Shift of center frequencies between
                                consecutive chirps
                                -> required for stepped FMCW [Hertz] */
  float fAdcSamplingInterval; /**< ADC sampling inverval
                                -> required to resolve range-Doppler coupling in
                                SFMCW [seconds] */
}
Struct_ZX_Itf4Eth_ContextData_t;
TYPEDEF_PACKED_END

TYPEDEF_PACKED_BEGIN(struct, Struct_ZX_Itf4Eth_BlindDetection) {
  uint16_t JammingStatus; /**<  Status of jamming: 0: none, 1:low, 2: medium, 3:
                             high, 4: error*/
}
Struct_ZX_Itf4Eth_BlindDetection_t;
TYPEDEF_PACKED_END

TYPEDEF_PACKED_BEGIN(struct, Struct_ZX_Itf4Eth_Header_t) {
  uint16_t ScanID;  /**< CycleCounter which overflows after 65535 */
  uint8_t PacketID; /**< PacketID for Header its 0 */
  uint8_t Reserved; /**< Not used at the moment, its to fulfill 4 Bytes Boundary
                       of the ContextData */
  Struct_ZX_Itf4Eth_ContextData_t sContextData;
  Struct_ZX_Itf4Eth_Dynamic_ConfigParams_t sDynamicConfigParams;
  Struct_ZX_Itf4Eth_AntennaElementGain_t sAntennaElementGain;
  Struct_ZX_Itf4Eth_BlindDetection_t sBlindDetection;
  uint32_t CRC; /**< CRC of the Header */
}
Struct_ZX_Itf4Eth_Header_t;
TYPEDEF_PACKED_END

/*
 *  High Resolution Noise
 */
TYPEDEF_PACKED_BEGIN(struct, Struct_ZX_Itf4Eth_HrNoise_t) {
  float Offset;
  float DynamicRange;
  uint8_t NoiseValues[SizeOfNoiseLevels];
}
Struct_ZX_Itf4Eth_HrNoise_t;
TYPEDEF_PACKED_END

/*
 * Payload off the PTL, without Header and CRC
 */
TYPEDEF_PACKED_BEGIN(struct, Struct_ZX_Itf4Eth_Payload_Package_t) {
  uint16_t ScanID;    /**< CycleCounter which overflows after 65535 */
  uint8_t PacketID;   /**< PacketID for Header its 0 */
  uint8_t PointCount; /**< PoinCount for number of valid targets in the
                         following Array */
  Struct_ZX_Itf4Eth_TargetType_t
      sTargets[SizeOfStdTargetPackage]; /**< Table of Targets detected */
  uint32_t CRC;
}
Struct_ZX_Itf4Eth_Payload_Package_t;
TYPEDEF_PACKED_END

/* Bytes before sTargets[] need to be the same as in the
 * Struct_ZX_Itf4Eth_Payload_Package_t cause access is the same in the
 * targetPrep... ==> Not to ask in every loop if target is in "normal" or
 * "remaining package"
 */
TYPEDEF_PACKED_BEGIN(struct, Struct_ZX_If44Eth_Payload_RemainTargetsNoise_t) {
  uint16_t ScanID;    /**< CycleCounter which overflows after 65535 */
  uint8_t PacketID;   /**< PacketID for Header its 0 */
  uint8_t PointCount; /**< PoinCount for number of valid targets in the
                         following Array */
  Struct_ZX_Itf4Eth_TargetType_t
      sTargets[SizeOfRemainingTargets]; /**< Table of Targets detected */
  Struct_ZX_Itf4Eth_HrNoise_t sHRNoise;
  uint32_t CRC;
}
Struct_ZX_Itf4Eth_Payload_RemainingTargetsAndNoise_t;
TYPEDEF_PACKED_END

TYPEDEF_PACKED_BEGIN(struct, Struct_ZX_If4Eth_GenericTypes_COMPLEX_t) {
  float real; /**< attribute real */
  float imag; /**< attribute imag */
}
Struct_ZX_If4Eth_GenericTypes_COMPLEX_t;
TYPEDEF_PACKED_END

TYPEDEF_PACKED_BEGIN(struct, Struct_ZX_Itf4Eth_Payload_Blockage_Dyn_Coeff_t) {
  uint16_t ScanID;  /**< CycleCounter which overflows after 65535 */
  uint8_t PacketID; /**< PacketID for Header it?s 0 */
  uint8_t Reserved; /**< Not used at the moment, it?s to fulfill 4 Bytes
                       Boundary of the ContextData */
  Struct_ZX_If4Eth_GenericTypes_COMPLEX_t
      sDynCalibCoeffs[SizeOfBlockagePackagesValues];
  uint32_t CRC;
}
Struct_ZX_Itf4Eth_Payload_Blockage_Dyn_Coeff_t;
TYPEDEF_PACKED_END

/**
 * Structure of full Target List
 */
TYPEDEF_PACKED_BEGIN(struct, struct_ZX_Itf4Eth_Primary_Data) {
  Struct_ZX_Itf4Eth_Header_t sHeader;
  Struct_ZX_Itf4Eth_Payload_Package_t sTargetPackages[NumberOfTargetPackages];
  Struct_ZX_Itf4Eth_Payload_RemainingTargetsAndNoise_t
      sRemainingTargetsAndNoisePackage;
  Struct_ZX_Itf4Eth_Payload_Blockage_Dyn_Coeff_t
      sBlockageDynCoeffs[SizeOfBlockagePackages];
  uint8_t
      reservedfor_RPU_8bytes[8]; /**< Anti Replay Value  ==> Patched by R5 */
  uint8_t
      reservedfor_RPU_16bytes[16]; /**< CMAC               ==> Patched by R5 */
}
struct_ZX_Itf4Eth_Primary_Data_t;
TYPEDEF_PACKED_END

} // namespace ZF_FRGen21_PTL_Interface_ZX_R01_00

#if defined(_MSC_VER)
#pragma pack(pop)
#endif

#ifdef __cplusplus
}
#endif