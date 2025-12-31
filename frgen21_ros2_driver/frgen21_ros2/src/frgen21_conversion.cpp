#include "frgen21_conversion.h"

void Frgen21Conversion::get_radar_mode(std::string &range_mode,
                                       std::string &chirp_mode,
                                       std::string &channel, float bandwidth,
                                       float carrier_frequency,
                                       float center_frequency_step) {
  std::unordered_map<std::string, double> BANDWIDTHS = {{"short", 720e6},
                                                        {"mid", 365e6},
                                                        {"far", 195e6},
                                                        {"SFMCW mid", 280e6},
                                                        {"SFMCW far", 195e6}};

  struct ChannelInfo {
    std::string channel;
    std::string chirp;
  };

  std::unordered_map<std::string, std::unordered_map<float, ChannelInfo>>
      CARRIER_FREQUENCIES = {
          {"short",
           {{76600000512.0, {"low", "up"}}, {76430000128.0, {"low", "down"}}}},
          {"mid",
           {{76379996160.0, {"low", "up"}},
            {76279996416.0, {"low", "down"}},
            {76759998464.0, {"high", "up"}},
            {76619997184.0, {"high", "down"}}}},
          {"far",
           {{76249997312.0, {"low", "up"}},
            {76170002432.0, {"low", "down"}},
            {76549996544.0, {"mid", "up"}},
            {76470001664.0, {"mid", "down"}},
            {76800000000.0, {"high", "up"}},
            {76770000896.0, {"high", "down"}}}},
          {"SFMCW mid", {{76515000320.0, {"low", "up"}}}},
          {"SFMCW far", {{76515000320.0, {"low", "up"}}}}};

  if (std::floor(bandwidth / 10e5) * 10e5 == BANDWIDTHS["short"]) {
    range_mode = "short";
  } else if (std::ceil(bandwidth / 10e5) * 10e5 == BANDWIDTHS["mid"]) {
    range_mode = "mid";
  } else if (center_frequency_step != 0.0 &&
             carrier_frequency ==
                 CARRIER_FREQUENCIES["SFMCW mid"].begin()->first) {
    if (bandwidth >= 23e7 && bandwidth <= 27e7) {
      range_mode = "SFMCW mid";
    } else if (bandwidth >= 16e7 && bandwidth <= 2e8) {
      range_mode = "SFMCW far";
    } else
      range_mode = "unknown";
  } else if (std::floor(bandwidth / 10e5) * 10e5 == BANDWIDTHS["far"]) {
    range_mode = "far";
  } else
    range_mode = "unknown";

  if (CARRIER_FREQUENCIES.find(range_mode) != CARRIER_FREQUENCIES.end() &&
      CARRIER_FREQUENCIES[range_mode].find(carrier_frequency) !=
          CARRIER_FREQUENCIES[range_mode].end()) {
    chirp_mode = CARRIER_FREQUENCIES[range_mode][carrier_frequency].chirp;
    channel = CARRIER_FREQUENCIES[range_mode][carrier_frequency].channel;
  }
}

sensor_msgs::msg::PointCloud2
Frgen21Conversion::targetsToPointCloud2(std::shared_ptr<char[]> &targets,
                                        std::string frame_id) {
  const ZF_FRGen21_PTL_Interface_R02_00::R2_IRSP_DerivedPointTargetData_t
      *target_data;
  target_data =
      reinterpret_cast<const ZF_FRGen21_PTL_Interface_R02_00::
                           R2_IRSP_DerivedPointTargetData_t *>(targets.get());
  int number_of_points = target_data->ui16NumPointTargets;
  uint32_t sec =
      static_cast<uint32_t>(target_data->ui64TimestampMeasureStart / 1e9);
  uint32_t nsec = static_cast<uint32_t>(target_data->ui64TimestampMeasureStart %
                                        static_cast<uint64_t>(1e9));

  sensor_msgs::msg::PointCloud2 msg_pc;
  sensor_msgs::PointCloud2Modifier modifier(msg_pc);

  modifier.setPointCloud2Fields(
      8, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
      sensor_msgs::msg::PointField::FLOAT32, "z", 1,
      sensor_msgs::msg::PointField::FLOAT32, "velocity", 1,
      sensor_msgs::msg::PointField::FLOAT32, "rcs", 1,
      sensor_msgs::msg::PointField::FLOAT32, "confidence", 1,
      sensor_msgs::msg::PointField::FLOAT32, "snr", 1,
      sensor_msgs::msg::PointField::FLOAT32, "power", 1,
      sensor_msgs::msg::PointField::FLOAT32);

  msg_pc.header.frame_id = frame_id;
  msg_pc.header.stamp.sec = sec;
  msg_pc.header.stamp.nanosec = nsec;
  msg_pc.height = 1;
  msg_pc.width = target_data->ui16NumPointTargets;
  modifier.resize(msg_pc.height * msg_pc.width);

  sensor_msgs::PointCloud2Iterator<float> iter_x(msg_pc, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(msg_pc, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(msg_pc, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_vel(msg_pc, "velocity");
  sensor_msgs::PointCloud2Iterator<float> iter_rcs(msg_pc, "rcs");
  sensor_msgs::PointCloud2Iterator<float> iter_conf(msg_pc, "confidence");
  sensor_msgs::PointCloud2Iterator<float> iter_snr(msg_pc, "snr");
  sensor_msgs::PointCloud2Iterator<float> iter_power(msg_pc, "power");

  // TODO use SIMD
  for (int i = 0; i < number_of_points; i++) {
    *iter_x = target_data->sPointTargetsDerived[i].fRange *
              std::cos(target_data->sPointTargetsDerived[i].fElevationAngle) *
              std::cos(target_data->sPointTargetsDerived[i].fAzimuthAngle);
    *iter_y = target_data->sPointTargetsDerived[i].fRange *
              std::cos(target_data->sPointTargetsDerived[i].fElevationAngle) *
              std::sin(target_data->sPointTargetsDerived[i].fAzimuthAngle);
    *iter_z = target_data->sPointTargetsDerived[i].fRange *
              std::sin(target_data->sPointTargetsDerived[i].fElevationAngle);
    *iter_vel = target_data->sPointTargetsDerived[i].fVelocity;
    *iter_rcs = target_data->sPointTargetsDerived[i].fRCS;
    *iter_conf = target_data->sPointTargetsDerived[i].fDetectionConfidence;
    *iter_snr = target_data->sPointTargetsDerived[i].sPrimaryInt.fSNR;
    *iter_power = target_data->sPointTargetsDerived[i].sPrimaryInt.fPower;

    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_vel;
    ++iter_rcs;
    ++iter_conf;
    ++iter_snr;
    ++iter_power;
  }

  return msg_pc;
}

frgen21_msgs::msg::RadarScanExtended
Frgen21Conversion::targetsToRadarScanExtended(std::shared_ptr<char[]> &targets,
                                              std::string frame_id) {
  const ZF_FRGen21_PTL_Interface_R02_00::R2_IRSP_DerivedPointTargetData_t
      *target_data;
  target_data =
      reinterpret_cast<const ZF_FRGen21_PTL_Interface_R02_00::
                           R2_IRSP_DerivedPointTargetData_t *>(targets.get());
  int number_of_points = target_data->ui16NumPointTargets;
  uint32_t sec =
      static_cast<uint32_t>(target_data->ui64TimestampMeasureStart / 1e9);
  uint32_t nsec = static_cast<uint32_t>(target_data->ui64TimestampMeasureStart %
                                        static_cast<uint64_t>(1e9));

  frgen21_msgs::msg::RadarScanExtended msg_scan;
  msg_scan.header.frame_id = frame_id;
  msg_scan.header.stamp.sec = sec;
  msg_scan.header.stamp.nanosec = nsec;

  for (int i = 0; i < number_of_points; i++) {
    frgen21_msgs::msg::RadarTargetExtended msg_target_tmp;

    msg_target_tmp.range = target_data->sPointTargetsDerived[i].fRange;
    msg_target_tmp.azimuth = target_data->sPointTargetsDerived[i].fAzimuthAngle;
    msg_target_tmp.elevation =
        target_data->sPointTargetsDerived[i].fElevationAngle;
    msg_target_tmp.velocity = target_data->sPointTargetsDerived[i].fVelocity;
    msg_target_tmp.snr = target_data->sPointTargetsDerived[i].sPrimaryInt.fSNR;
    msg_target_tmp.power =
        target_data->sPointTargetsDerived[i].sPrimaryInt.fPower;
    msg_target_tmp.rcs = target_data->sPointTargetsDerived[i].fRCS;
    msg_target_tmp.detectionconfidence =
        target_data->sPointTargetsDerived[i].fDetectionConfidence;
    msg_target_tmp.mean_square_error_range =
        target_data->sPointTargetsDerived[i].sPrimaryInt.sMSE.fRange;
    msg_target_tmp.mean_square_error_velocity =
        target_data->sPointTargetsDerived[i].sPrimaryInt.sMSE.fVelocity;
    msg_target_tmp.mean_square_error_azimuth =
        target_data->sPointTargetsDerived[i].sPrimaryInt.sMSE.fAzimuth;
    msg_target_tmp.mean_square_error_elevation =
        target_data->sPointTargetsDerived[i].sPrimaryInt.sMSE.fElevation;
    msg_target_tmp.mean_square_error_subarray =
        target_data->sPointTargetsDerived[i].sPrimaryInt.sMSE.fSubArray;
    msg_target_tmp.mean_square_error_subarray2nd_best =
        target_data->sPointTargetsDerived[i].sPrimaryInt.sMSE.fSubArray2ndBest;

    // FRGen21 does not output remaining signals
    // msg_target_tmp.std_dev_range =
    // msg_target_tmp.std_dev_velocity =
    // msg_target_tmp.std_dev_azimuth_angle =
    // msg_target_tmp.std_dev_elevation_angle =
    // msg_target_tmp.std_dev_rcs =

    msg_scan.targets.push_back(msg_target_tmp);
  }

  msg_scan.umabiguos_int_range = target_data->sFrameUnambiguousInterval.fRange;
  msg_scan.umabiguos_int_velocity =
      target_data->sFrameUnambiguousInterval.fVelocity;
  msg_scan.umabiguos_int_azimuth =
      target_data->sFrameUnambiguousInterval.fAzimuth;
  msg_scan.umabiguos_int_elevation =
      target_data->sFrameUnambiguousInterval.fElevation;

  // TODO add FrameSeparationCapability_t

  return msg_scan;
}

void Frgen21Conversion::targetsToROS(
    std::shared_ptr<char[]> &targets, std::string frame_id,
    sensor_msgs::msg::PointCloud2 &msg_pc,
    frgen21_msgs::msg::RadarScanExtended &msg_scan) {
  const ZF_FRGen21_PTL_Interface_R02_00::R2_IRSP_DerivedPointTargetData_t
      *target_data;
  target_data =
      reinterpret_cast<const ZF_FRGen21_PTL_Interface_R02_00::
                           R2_IRSP_DerivedPointTargetData_t *>(targets.get());
  int number_of_points = target_data->ui16NumPointTargets;
  uint32_t sec =
      static_cast<uint32_t>(target_data->ui64TimestampMeasureStart / 1e9);
  uint32_t nsec = static_cast<uint32_t>(target_data->ui64TimestampMeasureStart %
                                        static_cast<uint64_t>(1e9));

  sensor_msgs::PointCloud2Modifier modifier(msg_pc);
  modifier.setPointCloud2Fields(
      8, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1,
      sensor_msgs::msg::PointField::FLOAT32, "z", 1,
      sensor_msgs::msg::PointField::FLOAT32, "velocity", 1,
      sensor_msgs::msg::PointField::FLOAT32, "rcs", 1,
      sensor_msgs::msg::PointField::FLOAT32, "confidence", 1,
      sensor_msgs::msg::PointField::FLOAT32, "snr", 1,
      sensor_msgs::msg::PointField::FLOAT32, "power", 1,
      sensor_msgs::msg::PointField::FLOAT32);

  msg_pc.header.frame_id = frame_id;
  msg_pc.header.stamp.sec = sec;
  msg_pc.header.stamp.nanosec = nsec;
  msg_pc.height = 1;
  msg_pc.width = target_data->ui16NumPointTargets;
  modifier.resize(msg_pc.height * msg_pc.width);

  sensor_msgs::PointCloud2Iterator<float> iter_x(msg_pc, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(msg_pc, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(msg_pc, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_vel(msg_pc, "velocity");
  sensor_msgs::PointCloud2Iterator<float> iter_rcs(msg_pc, "rcs");
  sensor_msgs::PointCloud2Iterator<float> iter_conf(msg_pc, "confidence");
  sensor_msgs::PointCloud2Iterator<float> iter_snr(msg_pc, "snr");
  sensor_msgs::PointCloud2Iterator<float> iter_power(msg_pc, "power");

  msg_scan.header.frame_id = frame_id;
  msg_scan.header.stamp.sec = sec;
  msg_scan.header.stamp.nanosec = nsec;

  for (int i = 0; i < number_of_points; i++) {
    *iter_x = target_data->sPointTargetsDerived[i].fRange *
              std::cos(target_data->sPointTargetsDerived[i].fElevationAngle) *
              std::cos(target_data->sPointTargetsDerived[i].fAzimuthAngle);
    *iter_y = target_data->sPointTargetsDerived[i].fRange *
              std::cos(target_data->sPointTargetsDerived[i].fElevationAngle) *
              std::sin(target_data->sPointTargetsDerived[i].fAzimuthAngle);
    *iter_z = target_data->sPointTargetsDerived[i].fRange *
              std::sin(target_data->sPointTargetsDerived[i].fElevationAngle);
    *iter_vel = target_data->sPointTargetsDerived[i].fVelocity;
    *iter_rcs = target_data->sPointTargetsDerived[i].fRCS;
    *iter_conf = target_data->sPointTargetsDerived[i].fDetectionConfidence;
    *iter_snr = target_data->sPointTargetsDerived[i].sPrimaryInt.fSNR;
    *iter_power = target_data->sPointTargetsDerived[i].sPrimaryInt.fPower;
    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_vel;
    ++iter_rcs;
    ++iter_conf;
    ++iter_snr;
    ++iter_power;

    frgen21_msgs::msg::RadarTargetExtended msg_target_tmp;
    msg_target_tmp.range = target_data->sPointTargetsDerived[i].fRange;
    msg_target_tmp.azimuth = target_data->sPointTargetsDerived[i].fAzimuthAngle;
    msg_target_tmp.elevation =
        target_data->sPointTargetsDerived[i].fElevationAngle;
    msg_target_tmp.velocity = target_data->sPointTargetsDerived[i].fVelocity;
    msg_target_tmp.snr = target_data->sPointTargetsDerived[i].sPrimaryInt.fSNR;
    msg_target_tmp.power =
        target_data->sPointTargetsDerived[i].sPrimaryInt.fPower;
    msg_target_tmp.rcs = target_data->sPointTargetsDerived[i].fRCS;
    msg_target_tmp.detectionconfidence =
        target_data->sPointTargetsDerived[i].fDetectionConfidence;
    msg_target_tmp.mean_square_error_range =
        target_data->sPointTargetsDerived[i].sPrimaryInt.sMSE.fRange;
    msg_target_tmp.mean_square_error_velocity =
        target_data->sPointTargetsDerived[i].sPrimaryInt.sMSE.fVelocity;
    msg_target_tmp.mean_square_error_azimuth =
        target_data->sPointTargetsDerived[i].sPrimaryInt.sMSE.fAzimuth;
    msg_target_tmp.mean_square_error_elevation =
        target_data->sPointTargetsDerived[i].sPrimaryInt.sMSE.fElevation;
    msg_target_tmp.mean_square_error_subarray =
        target_data->sPointTargetsDerived[i].sPrimaryInt.sMSE.fSubArray;
    msg_target_tmp.mean_square_error_subarray2nd_best =
        target_data->sPointTargetsDerived[i].sPrimaryInt.sMSE.fSubArray2ndBest;
    msg_scan.targets.push_back(msg_target_tmp);
  }

  msg_scan.umabiguos_int_range = target_data->sFrameUnambiguousInterval.fRange;
  msg_scan.umabiguos_int_velocity =
      target_data->sFrameUnambiguousInterval.fVelocity;
  msg_scan.umabiguos_int_azimuth =
      target_data->sFrameUnambiguousInterval.fAzimuth;
  msg_scan.umabiguos_int_elevation =
      target_data->sFrameUnambiguousInterval.fElevation;
}

frgen21_msgs::msg::RadarInfo
Frgen21Conversion::frameToRadarInfo(std::shared_ptr<char[]> &frame,
                                    std::string frame_id) {
  const ZF_FRGen21_PTL_Interface_ZX_R01_00::struct_ZX_Itf4Eth_Primary_Data_t
      *tmp_package;
  tmp_package =
      reinterpret_cast<const ZF_FRGen21_PTL_Interface_ZX_R01_00::
                           struct_ZX_Itf4Eth_Primary_Data_t *>(frame.get());
  std::unique_ptr<ZF_FRGen21_PTL_Interface_R01_00::Struct_Itf4Eth_Primary_Data>
      ptrPrimaryData = std::make_unique<
          ZF_FRGen21_PTL_Interface_R01_00::Struct_Itf4Eth_Primary_Data>();
  PointTargetInterfaceMapping(ptrPrimaryData.get(), tmp_package);

  uint32_t sec = static_cast<uint32_t>(
      ptrPrimaryData->sHeader.sContextData.TimestampMeasure_Start / 1e9);
  uint32_t nsec = static_cast<uint32_t>(
      ptrPrimaryData->sHeader.sContextData.TimestampMeasure_Start %
      static_cast<uint64_t>(1e9));

  frgen21_msgs::msg::RadarInfo msg_radar_info;
  msg_radar_info.header.frame_id = frame_id;
  msg_radar_info.header.stamp.sec = sec;
  msg_radar_info.header.stamp.nanosec = nsec;
  msg_radar_info.interface_version =
      ptrPrimaryData->sHeader.sContextData.InterfaceVersion;
  msg_radar_info.sensor_serial_number =
      reinterpret_cast<const char *>(const_cast<uint8_t *>(
          ptrPrimaryData->sHeader.sContextData.SensorSerialNumber));
  msg_radar_info.software_version_boot =
      reinterpret_cast<const char *>(const_cast<uint8_t *>(
          ptrPrimaryData->sHeader.sContextData.SoftwareVersion_Boot));
  msg_radar_info.software_version_rpu =
      reinterpret_cast<const char *>(const_cast<uint8_t *>(
          ptrPrimaryData->sHeader.sContextData.SoftwareVersion_RPU));
  msg_radar_info.software_version_apu =
      reinterpret_cast<const char *>(const_cast<uint8_t *>(
          ptrPrimaryData->sHeader.sContextData.SoftwareVersion_APU));
  msg_radar_info.software_version_overall =
      reinterpret_cast<const char *>(const_cast<uint8_t *>(
          ptrPrimaryData->sHeader.sContextData.SoftwareVersion_Overall));
  msg_radar_info.cycle_counter =
      ptrPrimaryData->sHeader.sContextData.CycleCounter;
  msg_radar_info.timestamp_measure_start =
      ptrPrimaryData->sHeader.sContextData.TimestampMeasure_Start;
  msg_radar_info.timestamp_measure_end =
      ptrPrimaryData->sHeader.sContextData.TimestampMeasure_End;
  msg_radar_info.number_of_targets =
      ptrPrimaryData->sHeader.sContextData.NumberOfTargets;
  msg_radar_info.number_of_ttmf =
      ptrPrimaryData->sHeader.sContextData.NumberOfTTMFs;
  msg_radar_info.data_validity =
      ptrPrimaryData->sHeader.sContextData.DataValidity;
  msg_radar_info.gptp_status =
      ptrPrimaryData->sHeader.sContextData.eGPTP_Status;
  msg_radar_info.full_target_list =
      ptrPrimaryData->sHeader.sContextData.eFullTargetList;
  msg_radar_info.reserved = ptrPrimaryData->sHeader.sContextData.Reserved;
  msg_radar_info.effective_range =
      ptrPrimaryData->sHeader.sContextData.fEffectiveRange;
  msg_radar_info.index_temperature =
      ptrPrimaryData->sHeader.sContextData.IndexTemperature - 40;
  msg_radar_info.center_frequency_step =
      ptrPrimaryData->sHeader.sContextData.fCenterFrequencyStep;
  msg_radar_info.adc_sampling_interval =
      ptrPrimaryData->sHeader.sContextData.fAdcSamplingInterval;
  msg_radar_info.effective_bandwidth =
      ptrPrimaryData->sHeader.sDynamicConfigParams.fEffectiveBandwidth;
  msg_radar_info.pulse_repetition_time =
      ptrPrimaryData->sHeader.sDynamicConfigParams.fPulseRepetitionTime;
  msg_radar_info.carrier_frequency =
      ptrPrimaryData->sHeader.sDynamicConfigParams.fCarrierFrequency;
  msg_radar_info.normalized_signal_power =
      ptrPrimaryData->sHeader.sDynamicConfigParams.fNormalizedSignalPower;
  msg_radar_info.window_loss =
      ptrPrimaryData->sHeader.sDynamicConfigParams.fWindowLoss;
  msg_radar_info.jamming_status =
      ptrPrimaryData->sHeader.sBlindDetection.JammingStatus;

  std::string range_mode;
  std::string chirp_mode;
  std::string channel;

  get_radar_mode(
      range_mode, chirp_mode, channel,
      ptrPrimaryData->sHeader.sDynamicConfigParams.fEffectiveBandwidth,
      ptrPrimaryData->sHeader.sDynamicConfigParams.fCarrierFrequency,
      ptrPrimaryData->sHeader.sContextData.fCenterFrequencyStep);

  msg_radar_info.range_mode = range_mode;
  msg_radar_info.chirp_mode = chirp_mode;
  msg_radar_info.channel = channel;

  return msg_radar_info;
}

frgen21_msgs::msg::RadarFrame Frgen21Conversion::frameToRadarFrame(
    std::shared_ptr<char[]> &frame, uint32_t frame_size, std::string frame_id) {

  const ZF_FRGen21_PTL_Interface_ZX_R01_00::struct_ZX_Itf4Eth_Primary_Data_t
      *tmp_package;
  tmp_package =
      reinterpret_cast<const ZF_FRGen21_PTL_Interface_ZX_R01_00::
                           struct_ZX_Itf4Eth_Primary_Data_t *>(frame.get());
  std::unique_ptr<ZF_FRGen21_PTL_Interface_R01_00::Struct_Itf4Eth_Primary_Data>
      ptrPrimaryData = std::make_unique<
          ZF_FRGen21_PTL_Interface_R01_00::Struct_Itf4Eth_Primary_Data>();
  PointTargetInterfaceMapping(ptrPrimaryData.get(), tmp_package);

  uint32_t sec = static_cast<uint32_t>(
      ptrPrimaryData->sHeader.sContextData.TimestampMeasure_Start / 1e9);
  uint32_t nsec = static_cast<uint32_t>(
      ptrPrimaryData->sHeader.sContextData.TimestampMeasure_Start %
      static_cast<uint64_t>(1e9));

  frgen21_msgs::msg::RadarFrame msg_radar_frame;
  msg_radar_frame.header.frame_id = frame_id;
  msg_radar_frame.header.stamp.sec = sec;
  msg_radar_frame.header.stamp.nanosec = nsec;
  msg_radar_frame.version =
      reinterpret_cast<const char *>(const_cast<uint8_t *>(
          ptrPrimaryData->sHeader.sContextData.SoftwareVersion_Overall));
  msg_radar_frame.data.resize(frame_size);
  std::copy(frame.get(), frame.get() + frame_size,
            msg_radar_frame.data.begin());

  return msg_radar_frame;
}
