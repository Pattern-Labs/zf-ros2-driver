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

#include <stddef.h>
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

namespace ZF_FRGen21_PTL_Interface_R01_00 {

/*************************************************************/
/*      POINT TARGET LIST DEFINITIONS                        */
/*************************************************************/

constexpr size_t SizeOfCompressedTargetList = 2900U;
constexpr size_t SizeOfNoiseLevels = 512U;

/*************************************************************/
/*      POINT TARGET LIST DATA TYPES                         */
/*************************************************************/

/**
 * Enum message type
 */
TYPEDEF_PACKED_BEGIN(enum, E_ITF4EETH_MessageTypeEnum){
    ITF4EETH_MESSAGE_TYPE_COMPRESSED = 0,
    ITF4EETH_MESSAGE_TYPE_BOOTUP_CALIBRATION = 1,
    ITF4EETH_MESSAGE_TYPE_MIMO_CHANNEL_CALIBRATION = 2,
    ITF4EETH_MESSAGE_TYPE_VERIFICATION = 3,
} E_ITF4EETH_MessageTypeEnum;
TYPEDEF_PACKED_END

/**
 * Enum of radar orientation
 */
TYPEDEF_PACKED_BEGIN(enum, E_ITF4EETH_ORIENTATION){
    E_ITF4EETH_HORIZONTAL_UPRIGHT = 0, /**< Orientation: horizontal upright */
    E_ITF4EETH_VERTICAL_TURNED_RIGHT, /**< Orientation: vertical turned right */
    E_ITF4EETH_HORIZONTAL_BUTTOM_UP,  /**< Orientation: horizontal buttom up */
    E_ITF4EETH_VERTICAL_TURNED_LEFT   /**< Orientation: vertical turned left */
} E_ITF4EETH_ORIENTATION;
TYPEDEF_PACKED_END

/**
 * Enum of full peak list computed
 */
TYPEDEF_PACKED_BEGIN(enum, E_ITF4EETH_FULL_PEAK_LIST){
    E_ITF4EETH_PEAK_LIST_NOT_FINISHED = 0,
    E_ITF4EETH_PEAK_LIST_OVERFLOW_PROTECTION, E_ITF4EETH_PEAK_LIST_FINISHED,
    E_ITF4EETH_PEAK_LIST_NON_RADIATING} E_ITF4EETH_FULL_PEAK_LIST;
TYPEDEF_PACKED_END

/**
 * Enum of GPTP status
 */
TYPEDEF_PACKED_BEGIN(enum, E_ITF4EETH_GPTP_STATUS){
    E_XA_ITF4EETH_GPTP_TIMEOUT_MASK = 0x1,
    E_XA_ITF4EETH_GPTP_SYNC_TO_GW_SHIFT = 0x2,
    E_XA_ITF4EETH_GPTP_SYNC_TO_GW_MASK = 0x4,
    E_XA_ITF4EETH_GPTP_GLOBAL_TIME_BASE_MASK = 0x8,
    E_XA_ITF4EETH_GPTP_TIMELEAP_FUTURE_MASK = 0x10,
    E_XA_ITF4EETH_GPTP_TIMELEAP_PAST_MASK = 0x20} E_ITF4EETH_GPTP_STATUS;
TYPEDEF_PACKED_END

/**
 * Enum of model type
 */
TYPEDEF_PACKED_BEGIN(enum, E_ITF4EETH_MODEL_TYPE){
    E_ITF4EETH_MODEL_TYPE_SINGLE_TARGET = 0, /**< Single target model only */
    E_ITF4EETH_MODEL_TYPE_SINGLE_TARGET_FALLBACK =
        1, /**< Two-target model tried, but not successful*/
    E_ITF4EETH_MODEL_TYPE_DOUBLE_TARGET =
        2, /**< Two-target model tried, successful, 2 targets found */
    E_ITF4EETH_MODEL_TYPE_UNKNOWN = 3, /**< error value*/
    E_ITF4EETH_MODEL_TYPE_FLAG_PARTIAL_PEAK =
        8 /**< flag indicating no high-resolution model fitting in range */
} E_ITF4EETH_MODEL_TYPE;
TYPEDEF_PACKED_END

/**
 * Frequency structure for 4D coordinates and their standard deviation
 * Note: the min/max values of the Elevation member correspond to the
 * valid range of the data type, not the field-of-view
 */
TYPEDEF_PACKED_BEGIN(struct, Struct_Itf4Eth_Frequency) {
  uint16_t Range;    /**< Range value            Min: 0 Max: 2*pi */
  uint16_t Velocity; /**< Velocity value         Min: 0 Max: 2*pi */
  int16_t Azimuth;   /**< Azimuth value          Min: -5*pi Max: 5*pi*/
  int16_t Elevation; /**< Elevation value        Min: -11*pi Max: 11*pi*/
}
Struct_Itf4Eth_Frequency_t;
TYPEDEF_PACKED_END

/**
 * MSE - Mean square error
 */
TYPEDEF_PACKED_BEGIN(struct, Struct_Itf4Eth_MSE) {
  uint8_t Range;    /**< MSE range value                Min: -60 dB Max: 0 dB */
  uint8_t Velocity; /**< MSE velocity value             Min: -60 dB Max: 0 dB */
  uint8_t Azimuth;  /**< MSE azimuth angle value        Min: -60 dB Max: 0 dB */
  uint8_t
      Elevation;    /**< MSE elevation angle value      Min: -60 dB Max: 0 dB */
  uint8_t SubArray; /**< MSE best sub array value       Min: -60 dB Max: 0 dB */
  uint8_t SubArray2ndBest; /**< MSE 2nd best sub array value   Min: -60 dB Max:
                              0 dB */
}
Struct_Itf4Eth_MSE_t;
TYPEDEF_PACKED_END

/**
 * Structure containing the IF_Filter_LUT index of the 2nd and 3rd best AAR
 * solutions
 */
TYPEDEF_PACKED_BEGIN(struct, Struct_Itf4Eth_AAR_Index) {
  uint8_t Index2ndBest : 4; /**< 2nd best sub array index */
  uint8_t Index3rdBest : 4; /**< 3rd best sub array index */
}
Struct_Itf4Eth_AAR_Index_t;
TYPEDEF_PACKED_END

/**
 * Structure of target type
 */
TYPEDEF_PACKED_BEGIN(struct, Struct_Itf4Eth_TargetType) {
  Struct_Itf4Eth_Frequency_t sFrequency; /**< Structure of frequency */
  Struct_Itf4Eth_MSE_t sMSE; /**< Structure of mean-squared fitting errors */
  Struct_Itf4Eth_AAR_Index_t sAAR; /**< 2nd and 3rd best AAR solution */
  uint8_t SNR;       /**< Signal-to-noise ratio          Min: 0 dB Max: 60 dB */
  uint16_t Power;    /**< Target power [dB] */
  uint8_t ModelType; /**< Enum of model type */
  uint8_t
      DetectionConfidence; /**< Detection confidence           Min: 0; Max: 1 */
  uint16_t
      PeakID; /**< Peak ID of the target (for associating double targets) */
}
Struct_Itf4Eth_TargetType_t;
TYPEDEF_PACKED_END

/**
 * Structure of signal processing parameters for physical value derivation
 */
TYPEDEF_PACKED_BEGIN(struct, Struct_Itf4Eth_Dynamic_ConfigParams) {
  float fEffectiveBandwidth;  /**< Bandwidth of the radar for the current cycle
                                   -> required to compute range [Hertz] */
  float fPulseRepetitionTime; /**< Pulse Repetition Time (PRT) for current radar
                                 cycle
                                   -> required to compute radial velocity
                                 [Second] */
  float fCarrierFrequency; /**< Carrier frequency of the radar for current radar
                              cycle
                                -> may change in case of jamming [Hertz] */
  float fNormalizedSignalPower; /**< Normalized signal power for RCS calculation
                                   [dB] */
  float fWindowLoss; /**< Signal processing power scaling for RCS calculation
                        [dB] */
}
Struct_Itf4Eth_Dynamic_ConfigParams_t;
TYPEDEF_PACKED_END

/**
 * Structure of antenna gain
 */
TYPEDEF_PACKED_BEGIN(struct, Struct_Itf4Eth_AntennaElementGain) {
  uint16_t Azimuth[181];  /**< Antenna element gain in azimuth for RCS
                             calculation [dB]; Range: -90 deg to +90 deg */
  uint16_t Elevation[37]; /**< Antenna element gain in elevation for RCS
                             calculation [dB]; Range: -18 deg to +18 deg */
}
Struct_Itf4Eth_AntennaElementGain_t;
TYPEDEF_PACKED_END

TYPEDEF_PACKED_BEGIN(struct, Struct_Itf4Eth_ContextData) {
  uint32_t InterfaceVersion;           /**< Version information */
  uint8_t SensorSerialNumber[32];      /**< Serial Number of the device  */
  uint8_t SoftwareVersion_Boot[32];    /**< Softwareversion Bootloader */
  uint8_t SoftwareVersion_RPU[32];     /**< Softwareversion R5 */
  uint8_t SoftwareVersion_APU[32];     /**< Softwareversion A53 */
  uint8_t SoftwareVersion_Overall[32]; /**< Softwareversion Overall */

  uint32_t CycleCounter;           /**< Counter of the ethernet frame */
  uint64_t TimestampMeasure_Start; /**< Time stamp of starting chirp sequence */
  uint64_t TimestampMeasure_End;   /**< End time of chirp sequence */

  uint16_t NumberOfTargets; /**< Number of Targets detected */
  uint16_t NumberOfTTMFs;   /**< Number of TTMFs */

  uint8_t
      DataValidity; /**< Validity of the Data: 0: data invalid, 1: data valid*/
  uint8_t eGPTP_Status;    /**< Synchronized Status of the STBM from R5    */
  uint8_t eFullTargetList; /**< Flag for peak list completion
                              (E_ITF4EETH_FULL_PEAK_LIST as 8 bit) */
  uint8_t Reserved;

  float fEffectiveRange;      /**< Maximum calculated range of the targets */
  uint16_t IndexTemperature;  /**< IndexTemperature */
  float fCenterFrequencyStep; /**< Shift of center frequencies between
                                 consecutive chirps
                                   -> required for stepped FMCW [Hertz] */
  float fAdcSamplingInterval; /**< ADC sampling inverval
                                   -> required to resolve range-Doppler coupling
                                 in SFMCW [seconds] */
}
Struct_Itf4Eth_ContextData_t;
TYPEDEF_PACKED_END

/**
 * Structure of misalignment
 */
TYPEDEF_PACKED_BEGIN(struct, Struct_Itf4Eth_BlindDetection) {
  uint16_t JammingStatus; /**< Status of jamming: 0: none, 1: low, 2: medium, 3:
                             high, 4: error */
}
Struct_Itf4Eth_BlindDetection_t;
TYPEDEF_PACKED_END

/**
 * Structure for wrapping all head information
 */
TYPEDEF_PACKED_BEGIN(struct, Struct_Itf4Eth_Header_t) {
  Struct_Itf4Eth_ContextData_t sContextData;
  Struct_Itf4Eth_Dynamic_ConfigParams_t sDynamicConfigParams;
  Struct_Itf4Eth_AntennaElementGain_t sAntennaElementGain;
  Struct_Itf4Eth_BlindDetection_t sBlindDetection;
}
Struct_Itf4Eth_Header_t;
TYPEDEF_PACKED_END

/**
 * Compressed noise structure
 */
TYPEDEF_PACKED_BEGIN(struct, Struct_Itf4Eth_NoiseSpectrum_t) {
  float Offset;       /**< Minimum value of compressed noise values */
  float DynamicRange; /**< Maximum - minimum value of compressde noise values */
  uint8_t NoiseValues[SizeOfNoiseLevels]; /**< Compressed noise values UQ0.8
                                             normalized to [0, 1) */
}
Struct_Itf4Eth_NoiseSpectrum_t;
TYPEDEF_PACKED_END

/**
 * Payload containing either Point Target List (PTL) and HR nosise or
 * calibration data (disabled for ethernet interpreter)
 */
TYPEDEF_PACKED_BEGIN(struct, Union_Itf4Eth_Payload_t) {
  Struct_Itf4Eth_TargetType_t
      sTargets[SizeOfCompressedTargetList]; /**< Table of Targets detected */
  Struct_Itf4Eth_NoiseSpectrum_t sNoise;
}
Struct_Itf4Eth_Payload_t;
TYPEDEF_PACKED_END

/**
 * Structure of Target List
 */
TYPEDEF_PACKED_BEGIN(struct, Struct_Itf4Eth_Primary_Data) {
  Struct_Itf4Eth_Header_t sHeader;
  Union_Itf4Eth_Payload_t sPayload;
  uint32_t uiCRC32;
  uint8_t
      reservedfor_RPU_8bytes[8]; /**< Anti Replay Value  ==> Patched by R5 */
  uint8_t
      reservedfor_RPU_16bytes[16]; /**< CMAC               ==> Patched by R5 */
}
Struct_Itf4Eth_Primary_Data_t;
TYPEDEF_PACKED_END

} // namespace ZF_FRGen21_PTL_Interface_R01_00

#if defined(_MSC_VER)
#pragma pack(pop)
#endif

#ifdef __cplusplus
}
#endif