/*********************************************************************
 *   MODULE SPECIFICATION:
 *
 *      R2PhysicalValueDerivation Class.
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

#include "R02.00/R2PhysicalValueDerivation.h"

#define _USE_MATH_DEFINES
#include <algorithm>
#include <math.h>

using namespace ZF_FRGen21_PTL_Interface_R02_00;

// ***************** static variables ******************
const float substrate_delay =
    0.13f; /**< substrate and feeding line delay, in meter */

const float R2PhysicalValueDerivation::fRCS_Min =
    -1000.0f; /**< Minimum value to avoid log(0) */
const float R2PhysicalValueDerivation::fSpeedOfLight =
    299792458.0f; /**< Speed of electromagnetic waves [m/s] */
const float R2PhysicalValueDerivation::fRCS_TemperatureCalibrationFactor =
    0.0944f; /**< Compensate temperature dependence of signal power [dB/Celsius]
              */
const float R2PhysicalValueDerivation::fRCS_TemperatureCalibrationOffset =
    79.0f; /**< Calibration was done at 39 C, the temperature index in the PTL
              is converted into actual temperature[C] by subtracting -40 C (min
                temperature) -> total offset of 79 */

const float R2PhysicalValueDerivation::fAntennaSpacingHorizontal =
    0.00906106f; /**< Antenna spacing of URA [m] */
const float R2PhysicalValueDerivation::fAntennaSpacingVertical =
    0.02028891f; /**< Antenna spacing of URA [m] */
// ***************** Private functions *****************

namespace ZF_FRGen21_PTL_Interface_R02_00 {
template <typename T>
static T clip(const T &value, const T &lowerLimit, const T &upperLimit);

static int round(const float &fValue);

template <typename T> static T square(const T &fValue);
} // namespace ZF_FRGen21_PTL_Interface_R02_00

template <typename T>
static T ZF_FRGen21_PTL_Interface_R02_00::clip(const T &value,
                                               const T &lowerLimit,
                                               const T &upperLimit) {
  return std::max(lowerLimit, std::min(value, upperLimit));
}

static int ZF_FRGen21_PTL_Interface_R02_00::round(const float &fValue) {
  return (fValue < 0.0f) ? (int)(fValue - 0.5f) : (int)(fValue + 0.5f);
}

template <typename T>
static T ZF_FRGen21_PTL_Interface_R02_00::square(const T &fValue) {
  return fValue * fValue;
}

// ***************** Public functions ******************

R2PhysicalValueDerivation::R2PhysicalValueDerivation() { m_bInit = false; }

R2PhysicalValueDerivation::~R2PhysicalValueDerivation() {}

int R2PhysicalValueDerivation::CalculateDerivedTargetValues(
    const Struct_Itf4Eth_Primary_Data_t *ptr_PointTargetList_PrimaryData,
    R2_IRSP_DerivedPointTargetData_t *ptr_sDerivedPointTargetList) {
  const float fCarrierFrequency = ptr_PointTargetList_PrimaryData->sHeader
                                      .sDynamicConfigParams.fCarrierFrequency;
  const float fWavelength = fSpeedOfLight / fCarrierFrequency;

  ptr_sDerivedPointTargetList->ui32InterfaceVersion =
      ptr_PointTargetList_PrimaryData->sHeader.sContextData.InterfaceVersion;
  ptr_sDerivedPointTargetList->ui64TimestampMeasureStart =
      ptr_PointTargetList_PrimaryData->sHeader.sContextData
          .TimestampMeasure_Start;
  ptr_sDerivedPointTargetList->ui64TimestampMeasureEnd =
      ptr_PointTargetList_PrimaryData->sHeader.sContextData
          .TimestampMeasure_End;

  // determine software version
  int iSoftwareVersionMajor, iSoftwareVersionMinor;
  GetSoftwareVersion(ptr_PointTargetList_PrimaryData, &iSoftwareVersionMajor,
                     &iSoftwareVersionMinor);

  // Assume that the antenna gain matrix and the RCS correction tables will be
  // constant once initialised.
  if (m_bInit == false) {
    for (int iEl = 0; iEl < NumAntennaElGainValues; iEl++) {
      m_fAntennaGainVecElevation[iEl] =
          (ptr_PointTargetList_PrimaryData->sHeader.sAntennaElementGain
                   .Elevation[iEl] /
               655.0f -
           100.0f);
    }
    for (int iAz = 0; iAz < NumAntennaAzGainValues; iAz++) {
      m_fAntennaGainVecAzimuth[iAz] =
          (ptr_PointTargetList_PrimaryData->sHeader.sAntennaElementGain
                   .Azimuth[iAz] /
               655.0f -
           100.0f);
    }
    memcpy(&m_sFrequenyFilterGainLUT.fLUT[0], IF_Filter_LUT,
           sizeof(float) * SizeOfIF_FilterLUT);
    m_bInit = true;
  }

  float st_bw = ptr_PointTargetList_PrimaryData->sHeader.sContextData
                    .fCenterFrequencyStep *
                (float)(NoSlowTimeSamples - 1);
  float fc_0 = ptr_PointTargetList_PrimaryData->sHeader.sDynamicConfigParams
                   .fCarrierFrequency -
               st_bw / 2.0f;
#if 0 // Not used any more: range mapping takes fb as only input to produce more
      // stable result under different PRTs / velocities the velocity mapping
      // takes range as input instead of fb, to remove range dependence
    RangeDopplerMatrix a;
    a.xx = ptr_PointTargetList_PrimaryData->sHeader.sDynamicConfigParams.fEffectiveBandwidth / NoFastTimeSamples;
    a.xy = fc_0 * ptr_PointTargetList_PrimaryData->sHeader.sContextData.fAdcSamplingInterval;
    a.yx = ptr_PointTargetList_PrimaryData->sHeader.sContextData.fCenterFrequencyStep;
    a.yy = fc_0 * ptr_PointTargetList_PrimaryData->sHeader.sDynamicConfigParams.fPulseRepetitionTime;
    float det_a = (a.xx * a.yy - a.xy * a.yx) * (2.0f / fSpeedOfLight);
    // Compute Range-Doppler mapping matrix as inverse of A
    m_RangeDopplerMatrix.xx = +a.yy / det_a / (2.0f * (float)M_PI);
    m_RangeDopplerMatrix.xy = -a.xy / det_a / (2.0f * (float)M_PI);
    m_RangeDopplerMatrix.yx = -a.yx / det_a / (2.0f * (float)M_PI);
    m_RangeDopplerMatrix.yy = +a.xx / det_a / (2.0f * (float)M_PI);
#endif
  // Update IF_Filter_LUT step size for the current bandwidth setting
  m_sFrequenyFilterGainLUT.fStepSize =
      0.5f * fSpeedOfLight /
      ptr_PointTargetList_PrimaryData->sHeader.sDynamicConfigParams
          .fEffectiveBandwidth;

  // Calculate sFrameUnambiguousInterval values
  ptr_sDerivedPointTargetList->sFrameUnambiguousInterval.fRange =
      RangeMapping(2.0f * (float)M_PI, 0.0f,
                   ptr_PointTargetList_PrimaryData->sHeader.sDynamicConfigParams
                       .fEffectiveBandwidth);
  // the unambiguous velocity v_max. The final output will be in [-v_max/2,
  // v_max/2)
  ptr_sDerivedPointTargetList->sFrameUnambiguousInterval.fVelocity =
      VelocityMapping(0.0, 2.0f * (float)M_PI,
                      ptr_PointTargetList_PrimaryData->sHeader.sContextData
                          .fCenterFrequencyStep,
                      fc_0,
                      ptr_PointTargetList_PrimaryData->sHeader
                          .sDynamicConfigParams.fPulseRepetitionTime);
  ptr_sDerivedPointTargetList->sFrameUnambiguousInterval.fAzimuth = (float)M_PI;
  ptr_sDerivedPointTargetList->sFrameUnambiguousInterval.fElevation =
      2.0f * ElevationMapping(3.0f * (float)M_PI, fAntennaSpacingVertical *
                                                      fCarrierFrequency /
                                                      fSpeedOfLight);

  // Calculate separability values
  constexpr float fSeparationCapabilityFactorHighRes =
      0.5f; // factor by which Fourier limit is reduced when applying HighRes
            // model
  float best_bandwidth = std::max(ptr_PointTargetList_PrimaryData->sHeader
                                      .sDynamicConfigParams.fEffectiveBandwidth,
                                  ptr_PointTargetList_PrimaryData->sHeader
                                          .sContextData.fCenterFrequencyStep *
                                      (float)NoSlowTimeSamples);
  ptr_sDerivedPointTargetList->sFrameSeparationCapability.fRange =
      fSeparationCapabilityFactorHighRes * fSpeedOfLight /
      (2.0f * best_bandwidth);
  ptr_sDerivedPointTargetList->sFrameSeparationCapability.fVelocity =
      fSeparationCapabilityFactorHighRes * fWavelength /
      (2.0f * NoSlowTimeSamples *
       ptr_PointTargetList_PrimaryData->sHeader.sDynamicConfigParams
           .fPulseRepetitionTime);
  ptr_sDerivedPointTargetList->sFrameSeparationCapability.fAzimuth =
      fSeparationCapabilityFactorHighRes * asinf(1.0f / 39.0f);
  ptr_sDerivedPointTargetList->sFrameSeparationCapability.fElevation = asinf(
      1.0f / 41.0f); // no HighRes model applied in elevation -> Fourier limit

  // Copy effective range
  ptr_sDerivedPointTargetList->fEffectiveRange =
      ptr_PointTargetList_PrimaryData->sHeader.sContextData.fEffectiveRange;

  // Update parameters for calculating RCS
  float fRCS_TemperatureCalibrationCoeff =
      fRCS_TemperatureCalibrationFactor *
      (ptr_PointTargetList_PrimaryData->sHeader.sContextData.IndexTemperature -
       fRCS_TemperatureCalibrationOffset);
  float f_rcs_calibration = ptr_PointTargetList_PrimaryData->sHeader
                                .sDynamicConfigParams.fNormalizedSignalPower +
                            ptr_PointTargetList_PrimaryData->sHeader
                                .sDynamicConfigParams.fWindowLoss +
                            fRCS_TemperatureCalibrationCoeff;

  ptr_sDerivedPointTargetList->ui16NumPointTargets =
      ptr_PointTargetList_PrimaryData->sHeader.sContextData.NumberOfTargets;

  // fill properties for each target
  for (int iPT = 0;
       iPT <
       ptr_PointTargetList_PrimaryData->sHeader.sContextData.NumberOfTargets;
       iPT++) {
    float fFvHatWrapped =
        ptr_PointTargetList_PrimaryData->sPayload.sTargets[iPT]
            .sFrequency.Velocity /
        10430.0f;
    float fRange =
        RangeMapping(ptr_PointTargetList_PrimaryData->sPayload.sTargets[iPT]
                             .sFrequency.Range /
                         10430.0f,
                     fFvHatWrapped,
                     ptr_PointTargetList_PrimaryData->sHeader
                         .sDynamicConfigParams.fEffectiveBandwidth);

    // wrap velocity frequency from [0..2*pi) to [-pi..pi) for velocity values
    // in [-v_max/2, v_max)
    if (fFvHatWrapped >= M_PI) {
      fFvHatWrapped -= (float)(2.0f * M_PI);
    }

    ptr_sDerivedPointTargetList->sPointTargetsDerived[iPT].fVelocity =
        VelocityMapping(fRange, fFvHatWrapped,
                        ptr_PointTargetList_PrimaryData->sHeader.sContextData
                            .fCenterFrequencyStep,
                        fc_0,
                        ptr_PointTargetList_PrimaryData->sHeader
                            .sDynamicConfigParams.fPulseRepetitionTime);
    // the real range of any objects in the scene shall remove the deley in
    // substrate and feedling line
    ptr_sDerivedPointTargetList->sPointTargetsDerived[iPT].fRange =
        std::max(fRange - substrate_delay, 0.0f);
    ptr_sDerivedPointTargetList->sPointTargetsDerived[iPT].fAzimuthAngle =
        AzimuthMapping(
            ptr_PointTargetList_PrimaryData->sPayload.sTargets[iPT]
                    .sFrequency.Azimuth /
                2086.0f,
            ptr_PointTargetList_PrimaryData->sPayload.sTargets[iPT]
                    .sFrequency.Elevation /
                948.0f,
            fAntennaSpacingHorizontal * fCarrierFrequency / fSpeedOfLight,
            fAntennaSpacingVertical * fCarrierFrequency / fSpeedOfLight);

    ptr_sDerivedPointTargetList->sPointTargetsDerived[iPT].fElevationAngle =
        ElevationMapping(ptr_PointTargetList_PrimaryData->sPayload.sTargets[iPT]
                                 .sFrequency.Elevation /
                             948.0f,
                         fAntennaSpacingVertical * fCarrierFrequency /
                             fSpeedOfLight);

    // Unwrap Doppler
    // TODO: clean up, use modulo
    int num_wraps = 0;
    while (
        ptr_sDerivedPointTargetList->sPointTargetsDerived[iPT].fVelocity <
        -0.5f *
            ptr_sDerivedPointTargetList->sFrameUnambiguousInterval.fVelocity) {
      ptr_sDerivedPointTargetList->sPointTargetsDerived[iPT].fVelocity +=
          ptr_sDerivedPointTargetList->sFrameUnambiguousInterval.fVelocity;
      num_wraps++;
    }
    while (
        ptr_sDerivedPointTargetList->sPointTargetsDerived[iPT].fVelocity >
        +0.5f *
            ptr_sDerivedPointTargetList->sFrameUnambiguousInterval.fVelocity) {
      ptr_sDerivedPointTargetList->sPointTargetsDerived[iPT].fVelocity -=
          ptr_sDerivedPointTargetList->sFrameUnambiguousInterval.fVelocity;
      num_wraps--;
    }

    ptr_sDerivedPointTargetList->sPointTargetsDerived[iPT].sPrimaryInt.fPower =
        ptr_PointTargetList_PrimaryData->sPayload.sTargets[iPT].Power / 545.0f +
        40.0f;

    ptr_sDerivedPointTargetList->sPointTargetsDerived[iPT].fRCS =
        CalcRCS(ptr_sDerivedPointTargetList->sPointTargetsDerived[iPT]
                    .sPrimaryInt.fPower,
                f_rcs_calibration,
                (float)(ptr_sDerivedPointTargetList->sPointTargetsDerived[iPT]
                            .fAzimuthAngle *
                        180.0f / M_PI),
                (float)(ptr_sDerivedPointTargetList->sPointTargetsDerived[iPT]
                            .fElevationAngle *
                        180.0f / M_PI),
                ptr_sDerivedPointTargetList->sPointTargetsDerived[iPT].fRange);

    ptr_sDerivedPointTargetList->sPointTargetsDerived[iPT]
        .fDetectionConfidence =
        ptr_PointTargetList_PrimaryData->sPayload.sTargets[iPT]
            .DetectionConfidence /
        255.0f;

    // Extract model type from combined model type & partial peak flag
    ptr_sDerivedPointTargetList->sPointTargetsDerived[iPT].eModelType =
        static_cast<E_ITF4EETH_MODEL_TYPE>(
            ptr_PointTargetList_PrimaryData->sPayload.sTargets[iPT].ModelType &
            0x3);

    // Extract partial peak flag from combined model type & partial peak flag
    ptr_sDerivedPointTargetList->sPointTargetsDerived[iPT].ePartialPeakFlag =
        static_cast<E_ITF4EETH_MODEL_TYPE>(
            ptr_PointTargetList_PrimaryData->sPayload.sTargets[iPT].ModelType &
            E_ITF4EETH_MODEL_TYPE_FLAG_PARTIAL_PEAK);

    // Set MSE values and SNR
    // MSE decompressed = MSE compressed / scaling factor + offset
    ptr_sDerivedPointTargetList->sPointTargetsDerived[iPT]
        .sPrimaryInt.sMSE.fRange =
        ptr_PointTargetList_PrimaryData->sPayload.sTargets[iPT].sMSE.Range *
            0.25f -
        60.0f;
    ptr_sDerivedPointTargetList->sPointTargetsDerived[iPT]
        .sPrimaryInt.sMSE.fVelocity =
        ptr_PointTargetList_PrimaryData->sPayload.sTargets[iPT].sMSE.Velocity *
            0.25f -
        60.0f;
    ptr_sDerivedPointTargetList->sPointTargetsDerived[iPT]
        .sPrimaryInt.sMSE.fAzimuth =
        ptr_PointTargetList_PrimaryData->sPayload.sTargets[iPT].sMSE.Azimuth *
            0.25f -
        60.0f;
    ptr_sDerivedPointTargetList->sPointTargetsDerived[iPT]
        .sPrimaryInt.sMSE.fElevation =
        ptr_PointTargetList_PrimaryData->sPayload.sTargets[iPT].sMSE.Elevation *
            0.25f -
        60.0f;
    ptr_sDerivedPointTargetList->sPointTargetsDerived[iPT]
        .sPrimaryInt.sMSE.fSubArray =
        ptr_PointTargetList_PrimaryData->sPayload.sTargets[iPT].sMSE.SubArray *
            0.25f -
        20.0f;
    ptr_sDerivedPointTargetList->sPointTargetsDerived[iPT]
        .sPrimaryInt.sMSE.fSubArray2ndBest =
        ptr_PointTargetList_PrimaryData->sPayload.sTargets[iPT]
                .sMSE.SubArray2ndBest *
            0.25f -
        20.0f;
    ptr_sDerivedPointTargetList->sPointTargetsDerived[iPT].sPrimaryInt.fSNR =
        ptr_PointTargetList_PrimaryData->sPayload.sTargets[iPT].SNR * 0.25f;

    // Unpack 2nd and 3rd best AAR solutions (frequencies)
    FrequencyShiftAAR(ptr_PointTargetList_PrimaryData->sPayload.sTargets[iPT]
                              .sFrequency.Azimuth /
                          2086.0f,
                      ptr_PointTargetList_PrimaryData->sPayload.sTargets[iPT]
                              .sFrequency.Elevation /
                          948.0f,
                      ptr_PointTargetList_PrimaryData->sPayload.sTargets[iPT]
                          .sAAR.Index2ndBest,
                      &ptr_sDerivedPointTargetList->sPointTargetsDerived[iPT]
                           .sPrimaryInt.sAAR.fAzimuth2ndBest,
                      &ptr_sDerivedPointTargetList->sPointTargetsDerived[iPT]
                           .sPrimaryInt.sAAR.fElevation2ndBest);
    FrequencyShiftAAR(ptr_PointTargetList_PrimaryData->sPayload.sTargets[iPT]
                              .sFrequency.Azimuth /
                          2086.0f,
                      ptr_PointTargetList_PrimaryData->sPayload.sTargets[iPT]
                              .sFrequency.Elevation /
                          948.0f,
                      ptr_PointTargetList_PrimaryData->sPayload.sTargets[iPT]
                          .sAAR.Index3rdBest,
                      &ptr_sDerivedPointTargetList->sPointTargetsDerived[iPT]
                           .sPrimaryInt.sAAR.fAzimuth3rdBest,
                      &ptr_sDerivedPointTargetList->sPointTargetsDerived[iPT]
                           .sPrimaryInt.sAAR.fElevation3rdBest);

    // Convert 2nd and 3rd best AAR solutions from frequencies to angles
    ptr_sDerivedPointTargetList->sPointTargetsDerived[iPT]
        .sPrimaryInt.sAAR.fAzimuth2ndBest = AzimuthMapping(
        ptr_sDerivedPointTargetList->sPointTargetsDerived[iPT]
            .sPrimaryInt.sAAR.fAzimuth2ndBest,
        ptr_sDerivedPointTargetList->sPointTargetsDerived[iPT]
            .sPrimaryInt.sAAR.fElevation2ndBest,
        fAntennaSpacingHorizontal * fCarrierFrequency / fSpeedOfLight,
        fAntennaSpacingVertical * fCarrierFrequency / fSpeedOfLight);
    ptr_sDerivedPointTargetList->sPointTargetsDerived[iPT]
        .sPrimaryInt.sAAR.fElevation2ndBest = ElevationMapping(
        ptr_sDerivedPointTargetList->sPointTargetsDerived[iPT]
            .sPrimaryInt.sAAR.fElevation2ndBest,
        fAntennaSpacingVertical * fCarrierFrequency / fSpeedOfLight);
    ptr_sDerivedPointTargetList->sPointTargetsDerived[iPT]
        .sPrimaryInt.sAAR.fAzimuth3rdBest = AzimuthMapping(
        ptr_sDerivedPointTargetList->sPointTargetsDerived[iPT]
            .sPrimaryInt.sAAR.fAzimuth3rdBest,
        ptr_sDerivedPointTargetList->sPointTargetsDerived[iPT]
            .sPrimaryInt.sAAR.fElevation3rdBest,
        fAntennaSpacingHorizontal * fCarrierFrequency / fSpeedOfLight,
        fAntennaSpacingVertical * fCarrierFrequency / fSpeedOfLight);
    ptr_sDerivedPointTargetList->sPointTargetsDerived[iPT]
        .sPrimaryInt.sAAR.fElevation3rdBest = ElevationMapping(
        ptr_sDerivedPointTargetList->sPointTargetsDerived[iPT]
            .sPrimaryInt.sAAR.fElevation3rdBest,
        fAntennaSpacingVertical * fCarrierFrequency / fSpeedOfLight);
  }

  // Scale noise levels
  // noise decompressed = noise_compressed / 255 * factor + offset
  for (int i = 0; i < SizeOfNoiseLevels; i++) {
    ptr_sDerivedPointTargetList->fNoiseLevels[i] =
        (float)ptr_PointTargetList_PrimaryData->sPayload.sNoise.NoiseValues[i] *
            ptr_PointTargetList_PrimaryData->sPayload.sNoise.DynamicRange /
            255.0f +
        ptr_PointTargetList_PrimaryData->sPayload.sNoise.Offset;
  }

  return 0;
}

void R2PhysicalValueDerivation::GetSoftwareVersion(
    const Struct_Itf4Eth_Primary_Data_t *ptr_PointTargetList_PrimaryData,
    int *ptr_iVersionMajor, int *ptr_iVersionMinor) {
  char aSoftwareVersionBuffer[3]; // +1 char for null terminator
  aSoftwareVersionBuffer[2] = '\0';
  memcpy(aSoftwareVersionBuffer,
         &ptr_PointTargetList_PrimaryData->sHeader.sContextData
              .SoftwareVersion_Overall[1],
         2 * sizeof(char));
  *ptr_iVersionMajor = atoi(aSoftwareVersionBuffer);
  memcpy(aSoftwareVersionBuffer,
         &ptr_PointTargetList_PrimaryData->sHeader.sContextData
              .SoftwareVersion_Overall[4],
         2 * sizeof(char));
  *ptr_iVersionMinor = atoi(aSoftwareVersionBuffer);
  return;
}

// ***************** Private functions ******************
/**
 *  Convert the fast-time frequency to range
 *  param[in] fRangeFrequency: in interval [0, 2*PI)
 *  param[in] fDopplerFrequency: The effective slow-time frequency in [0, 2*PI)
 *  param[in] fFastTimeBandwidth: the effective fast-time ramp bandwidth, in Hz
 *  return: the range in meters
 **/
float R2PhysicalValueDerivation::RangeMapping(float fRangeFrequency,
                                              float fDopplerFrequency,
                                              float fFastTimeBandwidth) {
  return NoFastTimeSamples * fSpeedOfLight / (2.0f * fFastTimeBandwidth) *
         fRangeFrequency / (2.0f * (float)M_PI);
#if 0 // Not used any more: the range mapping is take only fb as input
    return m_RangeDopplerMatrix.xx * fRangeFrequency + m_RangeDopplerMatrix.xy * fDopplerFrequency;
#endif
}

/**
 *  Convert the slow-time frequency to velocity
 *  param[in] fRange: in meter. For classical FMCW mode, this value will have no
 *influence due to fCenterFrequencyStep = 0. param[in] fDopplerFrequency: The
 *slow-time frequency in [-PI, PI) param[in] fCenterFrequencyStep: the increment
 *of center frequencies between chirps. Zero for FMCW mode, in Hertz. param[in]
 *fCarrierFreq_0: carrier frequency of the frist chirp, in Hertz param[in] fPRT:
 *pulse repetition time, in seconds return: the velocity in m/s within the
 *unambiguous interval of [-v_max/2, v_max/2)
 **/
float R2PhysicalValueDerivation::VelocityMapping(float fRange,
                                                 float fDopplerFrequency,
                                                 float fCenterFrequencyStep,
                                                 float fCarrierFreq_0,
                                                 float fPRT) {
  // Compute the frequency sub-component caused by range in slow-time signal
  float Fr = fmod(2.0f / fSpeedOfLight * fCenterFrequencyStep * fRange, 1.0f);
  float fVelocityFrequency = fDopplerFrequency - Fr * 2.0f * (float)M_PI;
  if (fVelocityFrequency < -(float)M_PI) {
    fVelocityFrequency += 2.0f * (float)M_PI;
  }
  return fVelocityFrequency * fSpeedOfLight / (2.0f * fPRT * fCarrierFreq_0) /
         (2.0f * (float)M_PI);
}

float R2PhysicalValueDerivation::AzimuthMapping(
    float fFrequencyAzimuth, float fFrequencyElevation,
    float fAntennaSpacingHorizontalURA, float fAntennaSpacingVerticalURA) {
  if (fFrequencyAzimuth == INVALID_ANGLE ||
      fFrequencyElevation == INVALID_ANGLE)
    return INVALID_ANGLE;
  fFrequencyAzimuth /= (2.0f * (float)M_PI) * fAntennaSpacingHorizontalURA;
  fFrequencyElevation /= (2.0f * (float)M_PI) * fAntennaSpacingVerticalURA;
  return asinf(
      clip(fFrequencyAzimuth / sqrtf(1.0f - square(fFrequencyElevation)), -1.0f,
           1.0f));
}

float R2PhysicalValueDerivation::ElevationMapping(
    float fFrequencyElevation, float fAntennaSpacingVerticalURA) {
  if (fFrequencyElevation == INVALID_ANGLE)
    return INVALID_ANGLE;
  return asinf(clip(fFrequencyElevation / (2.0f * (float)M_PI) /
                        fAntennaSpacingVerticalURA,
                    -1.0f, 1.0f));
}

float R2PhysicalValueDerivation::AntennaGain(float fAzimuth, float fElevation) {
  // summed up antenna gain in azimuth and elevation
  return LinearInterpolation(fAzimuth + (NumAntennaAzGainValues / 2),
                             m_fAntennaGainVecAzimuth, NumAntennaAzGainValues) +
         LinearInterpolation(fElevation + (NumAntennaElGainValues / 2),
                             m_fAntennaGainVecElevation,
                             NumAntennaElGainValues);
}

float R2PhysicalValueDerivation::FrequencyFilterGain(float fRange) {
  return LinearInterpolation(fRange / m_sFrequenyFilterGainLUT.fStepSize,
                             &m_sFrequenyFilterGainLUT.fLUT[0],
                             SizeOfIF_FilterLUT);
}

float R2PhysicalValueDerivation::LinearInterpolation(float fx, float *fValues,
                                                     size_t numValues) {
  // float to array index
  int x0 = std::max<int>(0, (int)floorf(fx));
  int x1 = std::min<int>((int)ceilf(fx), numValues - 1);
  // introduce alias for readability
  const float y0 = fValues[x0];
  const float y1 = fValues[x1];
  // division by x1-x0 not required due to ceil-floor = 1 or 0, where 0 is the
  // case fx=x0=x1
  return y0 + (fx - x0) * (y1 - y0);
}

float R2PhysicalValueDerivation::CalcRCS(float fPower, float fRCS_Calibration,
                                         float fAzimuth, float fElevation,
                                         float fRange) {
  // summed up gains in both angles
  float fRCS = fRCS_Min;
  float fGain = AntennaGain(fAzimuth, fElevation) + FrequencyFilterGain(fRange);

  if (fRange > 0.0f) {
    fRCS = fPower + (40.0f * log10f(fRange) + fRCS_Calibration - fGain);
  }
  return fRCS;
}

void R2PhysicalValueDerivation::FrequencyShiftAAR(
    float fAzimuthFrequency, float fElevationFrequency, int indexAAR,
    float *const ptr_fAzimuthFrequencyNext,
    float *const ptr_fElevationFrequencyNext) {
  // maps the resolved azimuth and elevation frequency back to the URA frequency
  // and applies the AAR solution given by indexAAR compute the URA frequency
  // from the 1st best AAR solution
  float fAzimuthFrequencyURA =
      fmodf(fAzimuthFrequency + 5.0f * (float)M_PI, 2.0f * (float)M_PI) -
      (float)M_PI;
  float fElevationFrequencyURA =
      fmodf(fElevationFrequency + 11.0f * (float)M_PI, 2.0f * (float)M_PI) -
      (float)M_PI;
  if (1.0f - fabsf(fAzimuthFrequencyURA * (float)M_1_PI) < 1.0e-3 ||
      1.0f - fabsf(fElevationFrequencyURA * (float)M_1_PI) < 1.0e-3) {
    // URA frequency is too close to interval edges, rounding might cause false
    // wrapping
    *ptr_fAzimuthFrequencyNext = INVALID_ANGLE;
    *ptr_fElevationFrequencyNext = INVALID_ANGLE;
  } else {
    // unravel index, mirror sign convention, and scale by 2pi
    float fAzimuthFrequencyShiftAAR =
        2.0f * (float)M_PI * ((4.0f - (float)(indexAAR % 5)) - 2.0f);
    float fElevationFrequencyShiftAAR =
        2.0f * (float)M_PI * ((2.0f - (float)(indexAAR / 5)) - 1.0f);
    *ptr_fAzimuthFrequencyNext =
        fAzimuthFrequencyURA + fAzimuthFrequencyShiftAAR;
    *ptr_fElevationFrequencyNext =
        fElevationFrequencyURA + fElevationFrequencyShiftAAR;
  }
}