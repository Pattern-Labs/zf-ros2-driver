# frgen21_msgs/msg/RadarInfo.msg

std_msgs/Header header

# based on Struct_Itf4Eth_ContextData
uint32 interface_version                        # version of radar frame struct
string sensor_serial_number                     # serial number of device
string software_version_boot                    # software version bootloader
string software_version_rpu                     # software version R5
string software_version_apu                     # software version A53
string software_version_overall                 # software verion overall
uint32 cycle_counter                            # counter of the radar frame
uint64 timestamp_measure_start                  # time stamp of starting chirp sequence [ns]
uint64 timestamp_measure_end                    # end time of chirp sequence [ns]
uint16 number_of_targets                        # number of detected targets
uint16 number_of_ttmf                           # number of two target model fitting (TTMF)
uint8 data_validity                             # 0 = data invalid, 1 = data valid
uint8 gptp_status                               # gPTP synchronization status: 0 = not synched, 8 = synched
uint8 full_target_list                          # flag for peak list completion: E_PEAK_LIST_NOT_FINISHED = 0, E_PEAK_LIST_FINISHED = 1
uint8 reserved
float32 effective_range                         # maximum calculated range of the targets
uint16 index_temperature                        # index temperature (offset of 40 was subtracted) [°C]
float32 center_frequency_step                   # shift of center frequencies between consecutive chirps (required for stepped FMCW) [Hz]
float32 adc_sampling_interval                   # ADC sampling interval (required to resolve range-Doppler coupling in SFMCW) [s]

# based on Struct_Itf4Eth_Dynamic_ConfigParams
float32 effective_bandwidth                     # bandwidth of the radar for the current cycle [Hz]
float32 pulse_repetition_time                   # Pulse Repetition Time (PRT) for current radar cycle [s]
float32 carrier_frequency                       # carrier frequency of the radar for current radar cycle [Hz]
float32 normalized_signal_power                 # normalized signal power for RCS calculation [dB]
float32 window_loss                             # signal processing power scaling for RCS calculation [dB]

# based on Struct_Itf4Eth_BlindDetection
uint16 jamming_status                           # status of jamming: 0 = none, 1 = low, 2 = medium, 3 = high

string range_mode
string chirp_mode
string channel