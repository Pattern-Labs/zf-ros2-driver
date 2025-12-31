/*********************************************************************
 *   MODULE SPECIFICATION:
 *
 *	   R1PhysicalValueDerivation Class.
 *      Calculate DerivedPointTarget values from ZF Radar Point Target Interface
 *- Primary Point Targets
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
 *      CopyrightDate: (C) ZF Friedrichshafen AG 2019-2022.
 *
 *      Last updated: 2022-10-12.
 *
 ********************************************************************/

#pragma once

#include <cmath>
#include <limits>
#include <string.h>

#include "R01.00/ZF_R1RCSRangeCorrection.h"
#include "R01.00/ZF_R1RadarPointTargetInterface.h"

namespace ZF_FRGen21_PTL_Interface_R01_00 {
constexpr float INVALID_ANGLE =
    INFINITY; /**< Value to mark angles of 2nd and 3rd best hypotheses as
                 unresolvable */

typedef struct _MSE_t {
  float fRange;    /**< MSE in range dimension, relative to target power [dB] */
  float fVelocity; /**< MSE in velocity dimension, relative to target power [dB]
                    */
  float
      fAzimuth; /**< MSE in azimuth dimension, relative to target power [dB] */
  float fElevation; /**< MSE in elevation dimension, relative to target power
                       [dB] */
  float fSubArray; /**< MSE of best AAR solution, relative to MIMO channel power
                      [dB] */
  float fSubArray2ndBest; /**< MSE 2nd best AAR solution, relative to MIMO
                             channel power [dB] */
} MSE_t;

typedef struct _AAR_t {
  float fAzimuth2ndBest; /**< Azimuth angle of 2nd best AAR solution [rad] */
  float
      fElevation2ndBest; /**< Elevation angle of 2nd best AAR solution [rad] */
  float fAzimuth3rdBest; /**< Azimuth angle of 3rd best AAR solution [rad] */
  float
      fElevation3rdBest; /**< Elevation angle of 3rd best AAR solution [rad] */
} AAR_t;

typedef struct _PrimaryInterface_t {
  MSE_t sMSE;   /**< Mean-squared-errors in each dimension*/
  AAR_t sAAR;   /**< Angles of 2nd and 3rd best AAR solution */
  float fSNR;   /**< Signal-to-noise ratio relative to CFAR noise [dB] */
  float fPower; /**< Target power [dB] */
} PrimaryInterface_t;

typedef struct _FrameUnambiguousInterval_t {
  float fRange;     /**< Maximum unambiguous range (range mode) [m] */
  float fVelocity;  /**< Maximum unambiguous velocity [m/sec] */
  float fAzimuth;   /**< Maximum unambiguous azimuth angle [rad] */
  float fElevation; /**< Maximum unambiguous elevation angle [rad] */
} FrameUnambiguousInterval_t;

typedef struct _FrameSeparationCapability_t {
  float fRange;     /**< Range resolution [m] */
  float fVelocity;  /**< Velocity resolution [m/sec] */
  float fAzimuth;   /**< Azimuth angle resolution [rad] */
  float fElevation; /**< Elevation angle resolution [rad] */
} FrameSeparationCapability_t;

typedef struct _PointTargetsDerived_t {
  float fRange;               /**< Range coordinate [m] */
  float fVelocity;            /**< Velocity coordinate (ambiguous) [m/sec] */
  float fAzimuthAngle;        /**< Azimuth angle coordinate [rad] */
  float fElevationAngle;      /**< Elevation angle coordinate [rad] */
  float fRCS;                 /**< Radar cross section [dBsm] */
  float fDetectionConfidence; /**< Relative confidence measure ranging from 0
                                 to 1. */
  E_ITF4EETH_MODEL_TYPE
  eModelType; /**< Model type enum without partial peak flag */
  E_ITF4EETH_MODEL_TYPE ePartialPeakFlag; /**< Partial peak flag */
  PrimaryInterface_t sPrimaryInt; /**< Signal processing intarface data */
} PointTargetsDerived_t;

typedef struct _R1_IRSP_DerivedPointTargetData_t {
  uint16_t
      ui16NumPointTargets; /**< Number of point targets in point target list */
  PointTargetsDerived_t sPointTargetsDerived[SizeOfCompressedTargetList];
  FrameUnambiguousInterval_t sFrameUnambiguousInterval;
  FrameSeparationCapability_t sFrameSeparationCapability;
  float fEffectiveRange; /**< Maximum processing range for current radar frame
                            [m] */
  float fNoiseLevels[SizeOfNoiseLevels]; /**< Noise levels per range bin [dB] */
  uint32_t ui32InterfaceVersion;
  uint64_t ui64TimestampMeasureStart; /**< Start of chirping [ns] */
  uint64_t ui64TimestampMeasureEnd;   /**< End of chirping [ns] */
} R1_IRSP_DerivedPointTargetData_t;

// Convert frequency values to sensible SI-units using the parameters set in
// modulation configuration

class R1PhysicalValueDerivation {
public:
  // ***************** static variables ******************
  static constexpr int NumAntennaAzGainValues =
      181; // -90 degrees to +90 degrees in 1 degree steps.
  static constexpr int NumAntennaElGainValues =
      37; // -18 degrees to +18 degrees in 1 degree steps.

  static const float fSpeedOfLight; // Speed of electromagnetic waves [m/s]
  static const float fAntennaSpacingHorizontal; // Antenna spacing of URA [m]
  static const float fAntennaSpacingVertical;   // Antenna spacing of URA [m]

  static constexpr int NoFastTimeSamples =
      512; // Number of time samples per chirp
  static constexpr int NoSlowTimeSamples = 512; // Number of chirps per cycle
  static constexpr int NoHorizontalURA_Channels =
      17; // Horizontal grid size of URA (including gap)
  static constexpr int NoVerticalURA_Channels = 8; // Vertical grid size of URA
  static constexpr int NoEffectiveURA_Channels =
      128; // Total number of channels on URA (excluding gap)

  // ***************** public functions ******************
  R1PhysicalValueDerivation();
  ~R1PhysicalValueDerivation();
  int CalculateDerivedTargetValues(
      const Struct_Itf4Eth_Primary_Data_t *ptr_PointTargetList_PrimaryData,
      R1_IRSP_DerivedPointTargetData_t *ptr_sDerivedPointTargetList);

private:
  // ***************** static variables ******************
  static const float fRCS_Min;
  static const float
      fRCS_TemperatureCalibrationFactor; // compensate temperature dependence of
                                         // signal power [dB/Celsius]
  static const float
      fRCS_TemperatureCalibrationOffset; // reference temperature for
                                         // calibration [Celsius]

  static constexpr int MaxNumAntennaAzGainValues =
      182U; // Set even to provide 8-byte alignment.
  static constexpr int MaxNumAntennaElGainValues =
      38U; // Set even to provide 8-byte alignment.

  // ***************** private variables ******************
  float m_fAntennaGainVecAzimuth[MaxNumAntennaAzGainValues];
  float m_fAntennaGainVecElevation[MaxNumAntennaElGainValues];
  struct FrequencyFilterGainLUT {
    float fLUT[NoFastTimeSamples];
    float fStepSize;
  } m_sFrequenyFilterGainLUT;
  bool m_bInit;

  struct RangeDopplerMatrix {
    float xx;
    float xy;
    float yx;
    float yy;
  } m_RangeDopplerMatrix;

  // ***************** private functions ******************
  void GetSoftwareVersion(
      const Struct_Itf4Eth_Primary_Data_t *ptr_PointTargetList_PrimaryData,
      int *ptr_iVersionMajor, int *ptr_iVersionMinor);
  float RangeMapping(float fRangeFrequency, float fDopplerFrequency,
                     float fFastTimeBandwidth);
  float VelocityMapping(float fRange, float fDopplerFrequency,
                        float fCenterFrequencyStep, float fCarrierFrequency,
                        float fPRT);
  float AzimuthMapping(float fAzimuthFrequency, float fElevationFrequency,
                       float f_dy_ura, float f_dz_ura);
  float ElevationMapping(float fFrequencyElevation_x, float f_dz_ura);
  float AntennaGain(float fAzimuth, float fElevation);
  float FrequencyFilterGain(float fRange);
  float CalcRCS(float fPower, float f_rcs_calibration, float fAzimuth,
                float fElevation, float fRange);
  void FrequencyShiftAAR(float fAzimuthFrequency, float fElevationFrequency,
                         int iIndexAAR, float *const ptr_fAzimuthFrequencyNext,
                         float *const ptr_fElevationFrequencyNext);
  float LinearInterpolation(float fx, float *fValues, size_t numValues);
}; //! class R1PhysicalValueDerivation

} // namespace ZF_FRGen21_PTL_Interface_R01_00