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
 *      Last updated: 2022-08-18.
 *
 ********************************************************************/

#pragma once

// output structure
#include "R01.00/ZF_R1RadarPointTargetInterface.h"

// input structure
#include "R01.00/ZF_R1RadarPointTargetInterface_ZX.h"

static_assert(
    sizeof(ZF_FRGen21_PTL_Interface_R01_00::Struct_Itf4Eth_Primary_Data_t) ==
        65012,
    "sizeof(Struct_Itf4Eth_Primary_Data_t) must be 65012!");

static_assert(
    sizeof(ZF_FRGen21_PTL_Interface_R01_00::Struct_Itf4Eth_AAR_Index) ==
        sizeof(ZF_FRGen21_PTL_Interface_ZX_R01_00::Struct_ZX_Itf4Eth_AAR_Index),
    "Incompatible data types 'Struct_Itf4Eth_AAR_Index' and "
    "'Struct_ZX_Itf4Eth_AAR_Index'; must have equal size!");

static_assert(
    sizeof(ZF_FRGen21_PTL_Interface_R01_00::Struct_Itf4Eth_MSE) ==
        sizeof(ZF_FRGen21_PTL_Interface_ZX_R01_00::Struct_ZX_Itf4Eth_MSE),
    "Incompatible data types 'Struct_Itf4Eth_MSE' and "
    "'Struct_ZX_Itf4Eth_MSE'; must have equal size!");

/**
 * @brief Swaps the byteorder between little and big endian
 * @param val[in] : input value
 * @return : input value in reversed byteorder
 * @author A. Donges
 * @date 2021-11-16
 */
template <typename T> static T SwapByteOrder(T val);

/**
 * @brief Map the ZX custom interface to the default FRGen21 interface and
 * swap the byteorder from big (network order) to little endian
 * @note Physical value derivation (PVD) requires the
 * ZF_FRGen21_PTL_Interface_R01_00::Struct_Itf4Eth_Primary_Data_t data
 * structure. Call this function before passing its output to PVD
 * @param ptr_PointTargetListOutput[out] : output data in little endian
 * @param ptr_PointTargetListInput[in] : input data in big endian
 * @author A. Donges
 * @date 2021-11-16
 */
void PointTargetInterfaceMapping(
    ZF_FRGen21_PTL_Interface_R01_00::Struct_Itf4Eth_Primary_Data_t
        *const ptr_PointTargetListOutput,
    ZF_FRGen21_PTL_Interface_ZX_R01_00::Struct_ZX_If4Eth_GenericTypes_COMPLEX_t
        *const sDynCalibCoeffs,
    const ZF_FRGen21_PTL_Interface_ZX_R01_00::struct_ZX_Itf4Eth_Primary_Data_t
        *const ptr_PointTargetListInput);