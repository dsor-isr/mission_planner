/**
 * Copyright (c) 2023 GBS Elektronik https://www.gbs-elektronik.de/en/downloads/downloads-nuclear-measurements.php
 * Copyright (c) 2023 Ploatech https://ploatech.com
 *
 * This file is part of mca_comm.
 *
 * mca_comm is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
 *
 * mca_comm is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program. If not, see
 * <https://www.gnu.org/licenses/>.
 *
 * This code has been adapted and made into a ROS wrapper from the tarball in
 * https://www.gbs-elektronik.de/en/downloads/downloads-nuclear-measurements.php
 */

#ifndef GBS_MCA_COMM_H
#define GBS_MCA_COMM_H

#include <mca_comm/gbs_mca_types.h>

#include <ftd2xx.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <iostream>
#include <list>
#include <thread>

namespace McaComm
{

// Constants -------------------------------------------------------------------------------

const int txBufferLength = 12;   // Length of the MCA commands
const int rxBufferLength = 136;  // Length of the MCA response
const int preampleLength = 2;
const int WriteTimeout = 100;  // in ms

const unsigned long PossibleBaudrates[] = { 3000000, 307200, 115200, 38400 };

const std::string TriggerFilterDesc[] = {
  "-1; +1", "-1; 0; +1", "+1; -2; +1", "+1; 0; -2; 0; +1", "4* -1; 12* 0; 4* +1", "4* +1; 4* 0; 4* -2; 4* 0; 4* +1"
};

// Enumerations ---------------------------------------------------------------------------

enum E_ERROR_FLAG
{
  ERROR_OK = 0,                       // successful data transfer
  ERROR_INTERFACE = 1,                // communication port is not initialized
  ERROR_UNKNOWN_COMMAND = 2,          // unknown command
  ERROR_COMMUNICATION = 3,            // faulty data transfer
  ERROR_INVALID_PARAM = 4,            // invalid parameter
  ERROR_RUNNING_MEAS = 5,             // measurement is running, but stopped
                                      // measurement is required for this command
  ERROR_VIOLATED_RIGHT = 6,           // execution right violation
  ERROR_STOPPED_MEAS = 7,             // measurement is stopped, but running
                                      // measurement is required for this command
  ERROR_WRONG_MODE = 8,               // wrong mode for using this command
  ERROR_UNHANDLED_COMMAND = 9,        // not handled by this firmware version
  ERROR_FILE_WRITING_IN_PROCESS = 10  // file writing is in process, this command must
                                      // not be called before the process is finished
};

enum E_MCAcmds  //...........................................................................
{
  CMD_INIT = 0x41,
  CMD_START = 0x42,
  CMD_STOP = 0x43,
  CMD_SET_ADC_RES_DISCR = 0x46,
  CMD_SET_THRESHOLD = 0x47,
  CMD_SET_PRESETS = 0x48,
  CMD_SET_GAIN = 0x4C,
  CMD_SET_BIAS = 0x4F,
  CMD_SET_SHAPING_TIME = 0x52,
  CMD_SET_MCA_INPUT = 0x54,
  CMD_SET_INPUT_POLARITY = 0x56,
  CMD_SET_MEASURE_PZC = 0x58,
  CMD_QUERY_POWER = 0x59,
  CMD_QUERY_STATE = 0x5A,
  CMD_QUERY_SPECTRA = 0x5B,
  CMD_QUERY_SYSTEM_DATA = 0x62,

  // additional commands for MCA-527 only

  CMD_QUERY_STATE527 = 0x0101,
  CMD_QUERY_SPECTRA_EX = 0x0102,
  CMD_SET_TRIGGER_FILTER = 0x0103,
  CMD_SHAPING_TIME_PAIR = 0x010C,
  CMD_SET_THRESHOLD_TENTHS = 0x010D,
  CMD_QUERY_STATE527_EX = 0x0110,
  CMD_SET_FLAT_TOP_TIME = 0x0113
};

// Structures of result data ------------------------------------------------------------------
#pragma pack(1)

struct QUERY_STATE  //..........................................................................
{
  uint16_t McaMode, Presets;
  uint32_t PresetValue, ElPreset;
  uint16_t Repeat, ElSweeps, TimePerChannel, ElTimePerChan;
  uint32_t RealTime, CountsPerSec, DeadTime, BusyTime;
  uint16_t Channels, Threshold, Lld, Uld, RoiBegin, RoiEnd, CoarseGain, FineGain, SlowDiscr, FastDiscr, DetectorBias,
      DetectorBiasPoly, PreampPower, PzcValue, PzcDtc1Offset, PzcDtc3Offset, StabState, StabResult, StabRoiBegin,
      StabRoiEnd, McaInputAdc, McaInputPol, Dtc, McaPur, McaInputMcs, McaNumber, HardwareVersion, FirmwareVersion,
      McsChannels, LastPowerState, BatteryCapacity, BatteryLifeTime;
  uint32_t StartTime;
  uint16_t Tdf, CommandAndParameter[4], BufferState;
  uint32_t AmpliDacVal;
  uint16_t DiffDeadTime;
  int16_t HvInhibitMode;
  uint16_t HvInhibitState, CheckSum, McaState, Reserve;
};

struct QUERY_STATE527  //.......................................................................
{
  uint16_t HwVersion, FwVersion, HwModification, FwModification;
  uint32_t Features;
  uint16_t Year;
  uint8_t Month, Day, Hour, Minute, Second, Reserve;
  uint32_t TestingPhase;
  int16_t McaTemperature;
  uint16_t GeneralMode;
  uint32_t DiscardedCycles;
  uint16_t CoreClock;
  uint8_t TriggerFilterLow, TriggerFilterHigh;
  uint16_t Expander, OffsetDac;
  int16_t DetectorTemperature;
  int16_t PowerModuleTemperature;
  uint16_t McaNumber;
  int16_t AmIRightHolder;
  uint8_t RightHoldersIp[4];
  uint16_t RightHoldersPort;
  int16_t ValidityOfRight;
  uint16_t MaxChannels;
  uint8_t PowerModuleVersions[2];
  uint16_t PowerModuleNumber, PowerModuleId, PowerModuleMaxHv, ThresholdTenths;
  uint32_t FastDeadTime;
  uint16_t FilterType, FlatTopTime, FilterSize, TriggerLevel;
  int16_t McaTemperatureAtStop, DetectorTemperatureAtStop;
  uint8_t IpAddressSet[4];
  uint8_t IpAddressActual[4];
  uint32_t TimePerChannel, ElTimePerChan;
  int32_t TriggerThreshold;
  int16_t PowerModuleTemperatureAtStop;
  uint16_t CommandAndParameter[4];
  uint8_t JitterCorrection, BaselineRestoring;
  int32_t SetTriggerThreshold;
  uint8_t Input, MaxShapingTime, GatingMode, GatingPol;
  uint8_t GatingShift;
  uint8_t CoarseGainLevels;
  uint16_t CheckSum, McaState, DiffFastDeadTime;
};

struct QUERY_STATE527_EX  //.................................................................
{
  uint32_t CommonMemorySize, CommonMemoryFillStop, CommonMemoryFillLevel;
  int16_t OsciTimeResolution;
  uint16_t OsciTriggerSource, OsciTriggerPosition, OsciTriggerThreshold;
  uint32_t PurCounter;
  uint8_t ExtPortPartA, ExtPortPartB, ExtPortPartC, ExtPortPartD, ExtPortPartE, ExtPortPartF, ExtPortParts,
      ExtPortStates, ExtPortPolarities, MaxFlattopTime;
  uint16_t BootPresetsDataSize;
  uint32_t ExtPortPulser1Period, ExtPortPulser2Period, ExtPortPulser1Width, ExtPortPulser2Width;
  uint16_t ExtPortRs232Baudrate, ExtPortRs232Flags;
  uint32_t ExtPortCounter1, ExtPortCounter1Cps, ExtPortCounter1Prev, ExtPortCounter2, ExtPortCounter2Cps,
      ExtPortCounter2Prev;
  uint16_t ExtPortRs232TxByteCount, FractionalRealTime;
  uint32_t PurCounterPrev, TriggerFilterAvailability;
  int16_t TriggerFilterValue1, TriggerFilterValue2;
  uint8_t TtlLowLevel, TtlHighLevel;
  uint16_t CoeffAutoThreshold;
  uint32_t AdcOverflowsPerSecond;
  uint16_t AdcSamplingRate, CommandAndParameter[4], NeededFileSize;
  uint32_t SdCardTotalMemorySize, SdCardFreeMemorySize;
  int8_t FileWritingState, FileWritingResult;
  uint16_t CheckSum, McaState, Rs485BaudRate;
};

struct QUERY_SPECTRA  //......................................................................
{
  uint32_t ChannelContents[32];
  uint16_t BufferState, CheckSum;
};

struct QUERY_POWER  //.......................................................................
{
  uint32_t BatteryCurrent, HvPrimaryCurrent, P12PrimaryCurrent, M12PrimaryCurrent, P24PrimaryCurrent, M24PrimaryCurrent,
      BatteryVoltage, HighVoltage, HvState;
  uint8_t P12Voltage, M12Voltage, P24Voltage, M24Voltage;
  uint32_t BiasCurrentValue;
  uint16_t Pin3Voltage, Pin5Voltage;
  uint32_t PowerSwitches, ChargerCurrent;
  uint16_t Pin5CurrentSourceValue, Pin5CurrentSourceState, Pin5Resistor;
  int8_t Pin5Offset, Pin5Gain;
  uint32_t BatteryCurrentAtStop, HvPrimaryCurrentAtStop, P12PrimaryCurrentAtStop, M12PrimaryCurrentAtStop,
      P24PrimaryCurrentAtStop, M24PrimaryCurrentAtStop, BatteryVoltageAtStop, HighVoltageAtStop;
  int8_t Pin3Offset, Pin3Gain;
  uint16_t HvControlVoltage;
  uint8_t P12VoltageAtStop, M12VoltageAtStop, P24VoltageAtStop, M24VoltageAtStop;
  uint16_t Pin3VoltageAtStop, CommandAndParameter[4], Pin5VoltageAtStop;
  uint32_t ChargerCurrentAtStop;
  uint8_t NotUsed1[4], PowerModulInfo, PowerModuleFeatures;
  uint16_t CheckSum, McaState;
  uint8_t NotUsed2[2];
};

struct QUERY_SYSTEM_DATA  //...............................................................
{
  uint64_t PdCounter,         // Peak detect counter, only for MCA-166
      Imp,                    // Fast detect counter
      PdPre,                  // MCA-166: Peak detect counter at time -1
                              // MCA-527: Counts outside the spectrum
      ImpPre;                 // MCA-166: Fast detect counter at time -1
                              // MCA-527: Counts outside the spectrum of previous sweep
  uint32_t BtPre,             // Busy time [ms] at time -1, only for MCA-166
      Time,                   // MCA on time [s]
      RealTimePrevSweep,      // Real time [s] of previous sweep
      DeadTimePrevSweep,      // Dead time [ms] of previous sweep
      StartTimePrevSweep,     // Start time of previous sweep
      FastDTimePrevSweep,     // Fast dead time [ms] of previous sweep, only for MCA-527
      ElSweeps,               // Elapsed sweeps
      BusyTimePrevSweep;      // Busy time [ms] of previous sweep
  uint16_t FractRTPrevSweep;  // MCA-527: fract. digits of the real time [ms] of prev. sweep
  uint64_t PdPrevSweep;       // Peak detect counter of prev. sweep, only for MCA-166

  uint16_t ImpPrevSweepLow;  // Fast detect counter of prev. sweep
  uint32_t ImpPrevSweepHigh,
      StabCounter;         // Counter of stabilization steps
  int32_t AmpliDacOffset,  // Current stabilization offset
      AmpliDacOffsetMin,   // Maximal negative stabilization offset
      AmpliDacOffsetMax;   // Maximal positive stabilization offset
  uint32_t RecCounter,     // Counter of received commands
      RecErrorCounter;     // Counter of unsuccessful commands
  uint16_t RecRate, CommandAndParameter[4], BufferState;
  uint32_t StabArea;  // Stabilization area preset
  uint16_t StabTime;  // Stabilization time preset [s]
  uint8_t LowShapingTime, HighShapingTime,
      MinRecomClock,    // Minimum recommended core clock, only for MCA-527
      MaxAllowedClock;  // Maximum allowed core clock, only for MCA-527
  uint16_t CheckSum, McaState, ADCsampleRate;
};

#pragma pack()

// Functions -------------------------------------------------------------------------------

uint32_t ToMcaTime(time_t time);
void FromMcaTime(uint32_t time, struct tm* outTime);
std::string GetTriggerFilterDesc(uint16_t index);

//-------------------------------------------------------------------------------------------

class GBS_MCA_Comm
{
protected:
  FT_HANDLE mcaHandle = NULL;
  bool connected = false;
  unsigned long CurrentBaudrate = 0;

  char* txbuffer;
  char* rxbuffer;
  int ReadTimeout;   // in ms
  int CommAttempts;  // number of attempts to read a response from MCA

  bool Connect(int index, unsigned long baudRate);
  bool CheckDevice(unsigned long baudRate);
  E_ERROR_FLAG CheckFrame();
  E_ERROR_FLAG DoCommunication();

  E_ERROR_FLAG QuerySpectraBasic(uint16_t FirstChannel, uint32_t CompressFactor);
  E_ERROR_FLAG QuerySpectraExBasic(uint16_t FirstChannel, uint16_t CompressFactor, uint16_t BufferControl);

public:
  GBS_MCA_Comm();
  ~GBS_MCA_Comm();

  // Functions of the communication dll .........................................

  // Establishes a connection to an MCA. The first MCA that is found will be used.
  // If baudRate = 0: the highest possible baud rate is set
  bool COMM_INIT(long timeout = 1000, /* Read timeout */
                 int tryAgain = 8,
                 unsigned long baudRate = 0);  // if baudRate = 0: the highest possible baud rate is set
  void COMM_CLOSE();

  E_ERROR_FLAG MMCA_RESET();
  E_ERROR_FLAG MMCA_START_ACQUIRE(uint16_t Flags, uint32_t StartTime);
  E_ERROR_FLAG MMCA_STOP_ACQUIRE();
  E_ERROR_FLAG MMCA_SET_PRESET_NONE();
  E_ERROR_FLAG MMCA_SET_PRESET_LIVE_TIME(uint32_t LiveTime);
  E_ERROR_FLAG MMCA_SET_PRESET_REAL_TIME(uint32_t RealTime);
  E_ERROR_FLAG MMCA_SET_ADC_RES_DISCR(uint16_t Channels, uint16_t LLD, uint16_t ULD);
  E_ERROR_FLAG MMCA_SET_GAIN(uint16_t CoarseGain, uint16_t FineGain);
  E_ERROR_FLAG MMCA_SET_MCA_INPUT_AMPLIFIER_POS();
  E_ERROR_FLAG MMCA_SET_MCA_INPUT_AMPLIFIER_NEG();
  E_ERROR_FLAG MMCA_SET_THRESHOLD(uint16_t Threshold);
  E_ERROR_FLAG MMCA_SET_THRESHOLD_TENTHS(uint16_t Threshold);
  E_ERROR_FLAG MMCA_SET_SHAPING_TIME_LOW();
  E_ERROR_FLAG MMCA_SET_SHAPING_TIME_HIGH();
  E_ERROR_FLAG MMCA_SET_SHAPING_TIME_PAIR(uint16_t Low, uint16_t High);
  E_ERROR_FLAG MMCA_SET_TRIGGER_FILTER(uint16_t Low, uint16_t High);
  E_ERROR_FLAG MMCA_SET_PZC_MANUAL(uint16_t PzcValue, int16_t* offset, uint16_t* MeasuredPulses,
                                   bool HighPrecision = true);
  E_ERROR_FLAG MMCA_SET_HIGH_VOLTAGES(uint16_t HighVoltage, int32_t Inhibit);
  E_ERROR_FLAG MMCA_SET_FLAT_TOP_TIME(uint16_t FlatTopTime);
  E_ERROR_FLAG MMCA_QUERY_POWER(struct QUERY_POWER* rec_data);
  E_ERROR_FLAG MMCA_QUERY_STATE(struct QUERY_STATE* rec_data);
  E_ERROR_FLAG MMCA_QUERY_STATE527(struct QUERY_STATE527* rec_data);
  E_ERROR_FLAG MMCA_QUERY_STATE527_EX(struct QUERY_STATE527_EX* rec_data);
  E_ERROR_FLAG MMCA_QUERY_SYSTEM_DATA(struct QUERY_SYSTEM_DATA* rec_data);
  E_ERROR_FLAG MMCA_QUERY_SPECTRA(uint16_t FirstChannel, uint32_t CompressFactor, QUERY_SPECTRA* rec_data);
  E_ERROR_FLAG MMCA_QUERY_SPECTRA_EX(uint16_t FirstChannel, uint16_t CompressFactor, uint16_t BufferControl,
                                     QUERY_SPECTRA* rec_data);

  // Additional communication functions, which are not available in the traditional communication library

  E_ERROR_FLAG MMCA_QUERY_SPECTRA_TO_BUFFER(uint16_t FirstChannel, uint32_t CompressFactor, uint32_t* buffer);
  E_ERROR_FLAG MMCA_QUERY_SPECTRA_EX_TO_BUFFER(uint16_t FirstChannel, uint16_t CompressFactor, uint16_t BufferControl,
                                               uint32_t* buffer);
  E_ERROR_FLAG MMCA_QUERY_COMPLETE_SPECTRUM(uint16_t FirstChannel, uint16_t LastChannel, uint16_t CompressFactor,
                                            uint32_t* buffer);
  E_ERROR_FLAG MMCA_QUERY_COMPLETE_SPECTRUM(uint16_t FirstChannel, uint16_t LastChannel, uint16_t CompressFactor,
                                            uint16_t BufferControl, uint32_t* buffer);

  // Some information about the communication object .............................................

  bool IsConnected() const;
  unsigned long GetBaudrate() const;
  int GetNumDevices();
};
}  // namespace McaComm

#endif  // GBS_MCA_COMM_H
