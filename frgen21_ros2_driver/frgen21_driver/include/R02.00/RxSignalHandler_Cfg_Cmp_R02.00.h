#ifndef RXSIGNALHANDLER_CFG_CMP_R02_00_H
#define RXSIGNALHANDLER_CFG_CMP_R02_00_H

#include <stdint.h>

#pragma pack(push, 1)

struct CommsInputSignalsR02_00 {
  uint16_t VehicleData_CRC; // CRC has to be calculated based on example code in
                            // release notes
  uint8_t VehicleData_AliveCounter; // to be incremented with every new message
  uint16_t VehicleSpeed; // VehicleSpeed = (PhysicalValue * 100) + 32768
                         // (0x8000)  PhysicalValue in m/s
  uint8_t SensingMode;   // 0: Not Radiating Mode, 1: Sensing Mode, 2: Shutdown
                         // Ack, 3: stepped FMCW
  uint8_t RangeMode;   // 0: short, 1: mid, 2: long, 3: SFMCW-260, 4: SFMCW-390
  uint8_t PRTMode;     // 0: low, 1: high, 2: alternating
  uint8_t ChirpMode;   // 0: up chirp, 1: down chirp
  uint8_t FreqMuxMode; // selected Range by RangeMode[0-2]    0: low,  1:
                       // mid(long-Range) high(mid-Range),  (2: high only in
                       // long Range)
  uint64_t
      AntiReplayCounter; // Anti Replay Counter, Low Value in the first Byte
  uint8_t CMAC[16];      // CMAC with StdKey[16]
};

#pragma pack(pop)

#endif /* RXSIGNALHANDLER_CFG_CMP_R02_00_H */
