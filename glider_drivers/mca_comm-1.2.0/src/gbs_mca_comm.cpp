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

#include <mca_comm/gbs_mca_comm.h>

#define MAX_DEVICES 5  // max. number of potentially connected devices

// Functions -----------------------------------------------------------------

uint32_t McaComm::ToMcaTime(time_t time)  //....................
{
  struct tm NullTime = { 0 };  // 31.12.1969 16:00
  NullTime.tm_mday = 31;
  NullTime.tm_mon = 11;   // December
  NullTime.tm_year = 69;  // years since 1900
  NullTime.tm_hour = 16;

  double seconds = difftime(time, mktime(&NullTime));

  if (seconds < 0)
    return 0;
  else
    return (uint32_t)seconds;
}

void McaComm::FromMcaTime(uint32_t time, struct tm* outTime)  //..................................
{
  memset(outTime, 0, sizeof(struct tm));  // initialize with 31.12.1969 16:00
  outTime->tm_mday = 31;
  outTime->tm_mon = 11;   // December
  outTime->tm_year = 69;  // years since 1900
  outTime->tm_hour = 16;

  outTime->tm_sec += time;  // add seconds since 31.12.1969 16:00
  time_t tmpTime = mktime(outTime);
  outTime = localtime(&tmpTime);  // consider overflow
}

std::string McaComm::GetTriggerFilterDesc(uint16_t index)  //.......................................
{
  int FieldLen = sizeof(McaComm::TriggerFilterDesc) / sizeof(McaComm::TriggerFilterDesc[0]);
  if (index < FieldLen)
    return McaComm::TriggerFilterDesc[index];
  else
    return "";
}

// Class GBS_MCA_comm ----------------------------------------------------------
//..............................................................................
McaComm::GBS_MCA_Comm::GBS_MCA_Comm()
{
  txbuffer = new char[txBufferLength];
  rxbuffer = new char[rxBufferLength];
  txbuffer[0] = 0xA5;  // Preample
  txbuffer[1] = 0x5A;
  txbuffer[10] = 0xB9;  // End flag
  txbuffer[11] = 0x9B;
}

McaComm::GBS_MCA_Comm::~GBS_MCA_Comm()  //................................................
{
  COMM_CLOSE();
  delete[] txbuffer;
  delete[] rxbuffer;
}

// Some information about the communication object ---------------------------------------

unsigned long McaComm::GBS_MCA_Comm::GetBaudrate() const  //................................
{
  return CurrentBaudrate;
}

bool McaComm::GBS_MCA_Comm::IsConnected() const  //........................................
{
  return connected;
}

// Functions of the communication dll ------------------------------------------

int McaComm::GBS_MCA_Comm::GetNumDevices()
{
  DWORD numDevs;  // number of connected devices
  char* BufLD[MAX_DEVICES + 1];
  char ConnectedDevices[MAX_DEVICES][64];

  // Prepare FT_ListDevices
  for (int i = 0; i < MAX_DEVICES; i++)
  {
    BufLD[i] = ConnectedDevices[i];
  }
  BufLD[MAX_DEVICES] = NULL;
  memset(ConnectedDevices, 0, sizeof(ConnectedDevices));

  const auto mcaStatus = FT_ListDevices(BufLD, &numDevs, FT_LIST_ALL | FT_OPEN_BY_DESCRIPTION);

  return mcaStatus == FT_OK ? numDevs : -1;
}

// Establishes a connection to an MCA. The first responding MCA that is found will be used.
// If baudRate = 0: the highest possible baud rate is set
// returns true if successful, false else
//..............................................................................
bool McaComm::GBS_MCA_Comm::COMM_INIT(long timeout, int tryAgain, unsigned long baudRate)
{
  if (IsConnected())
    return false;

  const auto numDevs = GetNumDevices();
  if (numDevs < 1)
  {
    return false;
  }

  ReadTimeout = std::max(timeout, 10L);
  CommAttempts = std::max(tryAgain, 1);

  bool success = false;
  for (int i = 0; i < numDevs; i++)
  {
    success = Connect(i, baudRate > 0 ? baudRate : 38400);
    if (success)
    {
      if (baudRate == 0)  // find highest possible baudrate
      {
        int j;
        int CntRates = sizeof(PossibleBaudrates) / sizeof(unsigned long);
        CommAttempts = std::max(tryAgain, CntRates + 1);
        const std::chrono::duration<int, std::milli> SleepAfterClose(10);

        for (j = 0; j < CntRates; j++)
        {
          if (success = CheckDevice(PossibleBaudrates[j]))
            break;
          std::this_thread::sleep_for(SleepAfterClose);
        }
        if (success && (PossibleBaudrates[j] == 307200))  // try 3000000 baud again
        {
          std::this_thread::sleep_for(SleepAfterClose);
          success = CheckDevice(3000000);
          if (!success)
          {
            const auto mcaStatus = FT_SetBaudRate(mcaHandle, 307200);  // turn back
            success = mcaStatus == FT_OK;
            if (success)
              CurrentBaudrate = 307200;
          }
        }
      }
      if (success)
        break;  // connected to the first available device
      else
        COMM_CLOSE();
    }
    else
      COMM_CLOSE();
  }

  return success;
}

void McaComm::GBS_MCA_Comm::COMM_CLOSE()  //...........................................................
{
  FT_Close(mcaHandle);
  connected = false;
}

// Reads the MCA state ...............................................................................
// if rec_data is null, no data are copied (e.g. for test if MCA answers)
McaComm::E_ERROR_FLAG McaComm::GBS_MCA_Comm::MMCA_QUERY_STATE(struct QUERY_STATE* rec_data)
{
  E_ERROR_FLAG rc = ERROR_OK;
  if (IsConnected() == false)
    return ERROR_INTERFACE;
  txbuffer[2] = (uint8_t)CMD_QUERY_STATE;
  memset(&txbuffer[3], 0x00, 7);
  rc = DoCommunication();

  if (rc == ERROR_OK && rec_data != 0)
    memcpy((char*)rec_data, rxbuffer + preampleLength, rxBufferLength - 4);
  return rc;
}

// Reads additional state information, for MCA-527 only ........................................
McaComm::E_ERROR_FLAG McaComm::GBS_MCA_Comm::MMCA_QUERY_STATE527(struct QUERY_STATE527* rec_data)
{
  E_ERROR_FLAG rc = ERROR_OK;

  if (IsConnected() == false)
    return ERROR_INTERFACE;
  *((uint16_t*)(&(txbuffer[2]))) = (uint16_t)CMD_QUERY_STATE527;
  memset(&txbuffer[4], 0x00, 6);
  rc = DoCommunication();

  if (rc == ERROR_OK && rec_data != 0)
  {
    memcpy((char*)rec_data, rxbuffer + preampleLength, rxBufferLength - 4);

    uint32_t tmpSpan = *(uint32_t*)(&(rec_data->Year));
    struct tm tmpDate;  // initialize with January 1, 2008
    memset(&tmpDate, 0, sizeof(tmpDate));
    tmpDate.tm_mday = 1;
    tmpDate.tm_mon = 0;     // January
    tmpDate.tm_year = 108;  //  years since 1900

    tmpDate.tm_mday += ((tmpSpan & 0xFFFE0000) >> 17);  // add days since January 1, 2008
    time_t tmpTime = mktime(&tmpDate);
    tmpDate = *localtime(&tmpTime);  // consider overflow
    rec_data->Year = (uint16_t)(tmpDate.tm_year + 1900);
    rec_data->Month = (uint8_t)(tmpDate.tm_mon + 1);
    rec_data->Day = (uint8_t)tmpDate.tm_mday;
    rec_data->Hour = (uint8_t)((tmpSpan & 0x0001F000) >> 12);
    rec_data->Minute = (uint8_t)((tmpSpan & 0x00000FC0) >> 6);
    rec_data->Second = (uint8_t)(tmpSpan & 0x0000003F);
  }
  return rc;
}

// Reads additional state information, for MCA-527 only ...............................................
McaComm::E_ERROR_FLAG McaComm::GBS_MCA_Comm::MMCA_QUERY_STATE527_EX(struct QUERY_STATE527_EX* rec_data)
{
  E_ERROR_FLAG rc = ERROR_OK;

  if (IsConnected() == false)
    return ERROR_INTERFACE;
  *((uint16_t*)(&(txbuffer[2]))) = (uint16_t)CMD_QUERY_STATE527_EX;
  memset(&txbuffer[4], 0x00, 6);
  rc = DoCommunication();

  if (rc == ERROR_OK && rec_data != 0)
  {
    memcpy((char*)rec_data, rxbuffer + preampleLength, rxBufferLength - 4);
  }
  return rc;
}

// Reads the MCA power state ...........................................................................
McaComm::E_ERROR_FLAG McaComm::GBS_MCA_Comm::MMCA_QUERY_POWER(struct QUERY_POWER* rec_data)
{
  E_ERROR_FLAG rc = ERROR_OK;

  if (IsConnected() == false)
    return ERROR_INTERFACE;
  txbuffer[2] = (uint8_t)CMD_QUERY_POWER;
  memset(&txbuffer[3], 0x00, 7);
  rc = DoCommunication();

  if (rc == ERROR_OK && rec_data != 0)
    memcpy((char*)rec_data, rxbuffer + preampleLength, rxBufferLength - 4);
  return rc;
}

// Reads the MMCA system data...............................................................................
McaComm::E_ERROR_FLAG McaComm::GBS_MCA_Comm::MMCA_QUERY_SYSTEM_DATA(QUERY_SYSTEM_DATA* rec_data)
{
  E_ERROR_FLAG rc = ERROR_OK;

  if (IsConnected() == false)
    return ERROR_INTERFACE;
  txbuffer[2] = (uint8_t)CMD_QUERY_SYSTEM_DATA;
  memset(&txbuffer[3], 0x00, 7);
  rc = DoCommunication();

  if (rc == ERROR_OK && rec_data != 0)
  {
    memcpy((char*)rec_data, rxbuffer + preampleLength, rxBufferLength - 4);

    // convert 48 Bit integers
    uint64_t tmp = rec_data->PdCounter;
    rec_data->PdCounter = *(((uint16_t*)&tmp) + 1) + (*(((uint32_t*)&tmp) + 1) << 16);
    tmp = rec_data->Imp;
    rec_data->Imp = *(((uint16_t*)&tmp) + 1) + (*(((uint32_t*)&tmp) + 1) << 16);
    tmp = rec_data->PdPre;
    rec_data->PdPre = *(((uint16_t*)&tmp) + 1) + (*(((uint32_t*)&tmp) + 1) << 16);
    tmp = rec_data->ImpPre;
    rec_data->ImpPre = *(((uint16_t*)&tmp) + 1) + (*(((uint32_t*)&tmp) + 1) << 16);
    tmp = rec_data->PdPrevSweep;
    rec_data->PdPrevSweep = *(((uint16_t*)&tmp) + 1) + (*(((uint32_t*)&tmp) + 1) << 16);

    if (rec_data->LowShapingTime == 0 && rec_data->HighShapingTime == 0)
    {
      rec_data->LowShapingTime = 10;
      rec_data->HighShapingTime = 20;
    }
  }
  return rc;
}

// Query Spectrum ----------------------------------------------------------------------------------------

McaComm::E_ERROR_FLAG McaComm::GBS_MCA_Comm::QuerySpectraBasic(uint16_t FirstChannel, uint32_t CompressFactor)
{
  E_ERROR_FLAG rc = ERROR_OK;

  if (IsConnected() == false)
    return ERROR_INTERFACE;
  txbuffer[2] = (uint8_t)CMD_QUERY_SPECTRA;
  txbuffer[3] = 0x00;
  *((uint16_t*)(&(txbuffer[4]))) = FirstChannel;
  *((uint16_t*)(&(txbuffer[6]))) = (uint16_t)CompressFactor;
  memset(&txbuffer[8], 0x00, 2);
  rc = DoCommunication();

  return rc;
}

McaComm::E_ERROR_FLAG McaComm::GBS_MCA_Comm::QuerySpectraExBasic(uint16_t FirstChannel, uint16_t CompressFactor,
                                                                 uint16_t BufferControl)
{
  E_ERROR_FLAG rc = ERROR_OK;

  if ((FirstChannel >= 4096) || (CompressFactor > 32) ||
      (BufferControl >= 16))  // These parameters are only accepted by CMD_QUERY_SPECTRA_EX
  {
    if (IsConnected() == false)
      return ERROR_INTERFACE;

    *((uint16_t*)(&(txbuffer[2]))) = (uint16_t)CMD_QUERY_SPECTRA_EX;
    *((uint16_t*)(&(txbuffer[4]))) = FirstChannel;
    *((uint16_t*)(&(txbuffer[6]))) = CompressFactor;
    *((uint16_t*)(&(txbuffer[8]))) = BufferControl;
    rc = DoCommunication();
  }
  else
    rc = QuerySpectraBasic(FirstChannel | ((BufferControl << 12) & 0xF000), CompressFactor);

  return rc;
}

// Reads the MMCA spectrum data (132 Bytes) ....................................................
McaComm::E_ERROR_FLAG McaComm::GBS_MCA_Comm::MMCA_QUERY_SPECTRA(uint16_t FirstChannel, uint32_t CompressFactor,
                                                                QUERY_SPECTRA* rec_data)
{
  McaComm::E_ERROR_FLAG rc = QuerySpectraBasic(FirstChannel, CompressFactor);
  if (rc == ERROR_OK && rec_data != 0)
    memcpy((char*)rec_data, rxbuffer + preampleLength, rxBufferLength - 4);
  return rc;
}

//  Reads the MMCA spectrum data (132 Bytes) ................................................................
McaComm::E_ERROR_FLAG McaComm::GBS_MCA_Comm::MMCA_QUERY_SPECTRA_EX(uint16_t FirstChannel, uint16_t CompressFactor,
                                                                   uint16_t BufferControl, QUERY_SPECTRA* rec_data)
{
  E_ERROR_FLAG rc = QuerySpectraExBasic(FirstChannel, CompressFactor, BufferControl);
  if (rc == ERROR_OK && rec_data != 0)
    memcpy((char*)rec_data, rxbuffer + preampleLength, rxBufferLength - 4);
  return rc;
}

// These functions read the spectrum data directly to the specified buffer position ........................

McaComm::E_ERROR_FLAG McaComm::GBS_MCA_Comm::MMCA_QUERY_SPECTRA_TO_BUFFER(uint16_t FirstChannel,
                                                                          uint32_t CompressFactor, uint32_t* buffer)
{
  E_ERROR_FLAG rc = QuerySpectraBasic(FirstChannel, CompressFactor);
  if (rc == ERROR_OK && buffer != 0)
  {
    for (int i = 0, offset = preampleLength; i < 32; i++, offset += 4)
      buffer[i] = *((uint32_t*)(rxbuffer + offset));
  }
  return rc;
}

McaComm::E_ERROR_FLAG McaComm::GBS_MCA_Comm::MMCA_QUERY_SPECTRA_EX_TO_BUFFER(uint16_t FirstChannel,
                                                                             uint16_t CompressFactor,
                                                                             uint16_t BufferControl, uint32_t* buffer)
{
  E_ERROR_FLAG rc = QuerySpectraExBasic(FirstChannel, CompressFactor, BufferControl);
  if (rc == ERROR_OK && buffer != 0)
  {
    for (int i = 0, offset = preampleLength; i < 32; i++, offset += 4)
      buffer[i] = *((uint32_t*)(rxbuffer + offset));
  }
  return rc;
}

// These functions read a complete spectrum or part of a spectrum by calling  MMCA_QUERY_SPECTRA_EX_TO_BUFFER
// several times ........................................................................................

McaComm::E_ERROR_FLAG McaComm::GBS_MCA_Comm::MMCA_QUERY_COMPLETE_SPECTRUM(uint16_t FirstChannel, uint16_t LastChannel,
                                                                          uint16_t CompressFactor, uint32_t* buffer)
{
  return MMCA_QUERY_COMPLETE_SPECTRUM(FirstChannel, LastChannel, CompressFactor, 0, buffer);
}

McaComm::E_ERROR_FLAG McaComm::GBS_MCA_Comm::MMCA_QUERY_COMPLETE_SPECTRUM(uint16_t FirstChannel, uint16_t LastChannel,
                                                                          uint16_t CompressFactor,
                                                                          uint16_t BufferControl, uint32_t* buffer)
{
  E_ERROR_FLAG rc = ERROR_OK;
  int ch, offset;
  for (ch = FirstChannel, offset = FirstChannel / CompressFactor; ch < LastChannel;
       ch += (CompressFactor << 5 /* *32 */), offset += 32)
  {
    rc = MMCA_QUERY_SPECTRA_EX_TO_BUFFER(ch, CompressFactor, BufferControl, &(buffer[offset]));
    if (rc != ERROR_OK)
      break;
  }
  return rc;
}

//------------------------------------------------------------------------------------------------------

// Starts an acquisition ...................................................................
McaComm::E_ERROR_FLAG McaComm::GBS_MCA_Comm::MMCA_START_ACQUIRE(uint16_t Flags, uint32_t StartTime)
{
  E_ERROR_FLAG rc = ERROR_OK;

  if (IsConnected() == false)
    return ERROR_INTERFACE;
  txbuffer[2] = (uint8_t)CMD_START;
  txbuffer[3] = 0x00;
  *((uint16_t*)(&(txbuffer[4]))) = Flags;
  *((uint32_t*)(&(txbuffer[6]))) = StartTime;
  rc = DoCommunication();
  return rc;
}

// The acquisition is stopped ...............................................................
McaComm::E_ERROR_FLAG McaComm::GBS_MCA_Comm::MMCA_STOP_ACQUIRE()
{
  E_ERROR_FLAG rc = ERROR_OK;

  if (IsConnected() == false)
    return ERROR_INTERFACE;
  txbuffer[2] = (uint8_t)CMD_STOP;
  memset(&txbuffer[3], 0x00, 7);
  rc = DoCommunication();
  return rc;
}

// Resets all MCA parameters to their initial state, all spectra are cleared and the measurement is
// aborted ............................................................................................
McaComm::E_ERROR_FLAG McaComm::GBS_MCA_Comm::MMCA_RESET()
{
  E_ERROR_FLAG rc = ERROR_OK;

  if (IsConnected() == false)
    return ERROR_INTERFACE;
  txbuffer[2] = (uint8_t)CMD_INIT;
  memset(&txbuffer[3], 0x00, 7);
  rc = DoCommunication();
  return rc;
}

// Settings --------------------------------------------------------------------------------------------

// sets the detector high voltage and controls the HV-inhibit-signal .........................
McaComm::E_ERROR_FLAG McaComm::GBS_MCA_Comm::MMCA_SET_HIGH_VOLTAGES(uint16_t HighVoltage, int32_t Inhibit)
{
  E_ERROR_FLAG rc = ERROR_OK;

  if (IsConnected() == false)
    return ERROR_INTERFACE;
  txbuffer[2] = (uint8_t)CMD_SET_BIAS;
  txbuffer[3] = 0x00;
  *((uint16_t*)(&(txbuffer[4]))) = HighVoltage;
  *((int32_t*)(&(txbuffer[6]))) = Inhibit;
  rc = DoCommunication();
  return rc;
}

// sets none automatic stop condition .......................................................
McaComm::E_ERROR_FLAG McaComm::GBS_MCA_Comm::MMCA_SET_PRESET_NONE()
{
  McaComm::E_ERROR_FLAG rc = ERROR_OK;

  if (IsConnected() == false)
    return ERROR_INTERFACE;
  txbuffer[2] = (uint8_t)CMD_SET_PRESETS;
  txbuffer[3] = 0x00;
  *((uint16_t*)(&(txbuffer[4]))) = PRESET_NONE;
  memset(&txbuffer[6], 0x00, 4);
  rc = DoCommunication();
  return rc;
}

// sets the time for the automatic stop condition (dead time corrected) ......................
McaComm::E_ERROR_FLAG McaComm::GBS_MCA_Comm::MMCA_SET_PRESET_LIVE_TIME(uint32_t LiveTime)
{
  McaComm::E_ERROR_FLAG rc = ERROR_OK;

  if (IsConnected() == false)
    return ERROR_INTERFACE;
  txbuffer[2] = (uint8_t)CMD_SET_PRESETS;
  txbuffer[3] = 0x00;
  *((uint16_t*)(&(txbuffer[4]))) = PRESET_LIVE;
  *((uint32_t*)(&(txbuffer[6]))) = LiveTime;
  rc = DoCommunication();
  return rc;
}

// sets the time for the automatic stop condition .............................................
McaComm::E_ERROR_FLAG McaComm::GBS_MCA_Comm::MMCA_SET_PRESET_REAL_TIME(uint32_t RealTime)
{
  McaComm::E_ERROR_FLAG rc = ERROR_OK;

  if (IsConnected() == false)
    return ERROR_INTERFACE;
  txbuffer[2] = (uint8_t)CMD_SET_PRESETS;
  txbuffer[3] = 0x00;
  *((uint16_t*)(&(txbuffer[4]))) = PRESET_REAL;
  *((uint32_t*)(&(txbuffer[6]))) = RealTime;
  rc = DoCommunication();
  return rc;
}

// sets the ADC resolution and the software discriminator range ......................................
McaComm::E_ERROR_FLAG McaComm::GBS_MCA_Comm::MMCA_SET_ADC_RES_DISCR(uint16_t Channels, uint16_t LLD, uint16_t ULD)
{
  McaComm::E_ERROR_FLAG rc = ERROR_OK;

  if (IsConnected() == false)
    return ERROR_INTERFACE;
  txbuffer[2] = (uint8_t)CMD_SET_ADC_RES_DISCR;
  txbuffer[3] = 0x00;
  *((uint16_t*)(&(txbuffer[4]))) = Channels;
  *((uint16_t*)(&(txbuffer[6]))) = LLD;
  *((uint16_t*)(&(txbuffer[8]))) = ULD;
  rc = DoCommunication();
  return rc;
}

// sets the analog threshold ..........................................................................
McaComm::E_ERROR_FLAG McaComm::GBS_MCA_Comm::MMCA_SET_THRESHOLD(uint16_t Threshold)
{
  McaComm::E_ERROR_FLAG rc = ERROR_OK;

  if (IsConnected() == false)
    return ERROR_INTERFACE;
  txbuffer[2] = (uint8_t)CMD_SET_THRESHOLD;
  txbuffer[3] = 0x00;
  txbuffer[4] = (uint8_t)Threshold;
  memset(&txbuffer[5], 0x00, 5);
  rc = DoCommunication();
  return rc;
}

// sets the analog threshold, only available for MCA527 ...............................................
McaComm::E_ERROR_FLAG McaComm::GBS_MCA_Comm::MMCA_SET_THRESHOLD_TENTHS(uint16_t Threshold)
{
  McaComm::E_ERROR_FLAG rc = ERROR_OK;

  if (IsConnected() == false)
    return ERROR_INTERFACE;
  *((uint16_t*)(&(txbuffer[2]))) = (uint16_t)CMD_SET_THRESHOLD_TENTHS;
  *((uint16_t*)(&(txbuffer[4]))) = Threshold;
  memset(&txbuffer[6], 0x00, 4);
  rc = DoCommunication();
  return rc;
}

// sets the amplifier coarse and fine gain ............................................................
McaComm::E_ERROR_FLAG McaComm::GBS_MCA_Comm::MMCA_SET_GAIN(uint16_t CoarseGain, uint16_t FineGain)
{
  McaComm::E_ERROR_FLAG rc = ERROR_OK;

  if (IsConnected() == false)
    return ERROR_INTERFACE;
  txbuffer[2] = (uint8_t)CMD_SET_GAIN;
  txbuffer[3] = 0x00;
  *((uint16_t*)(&(txbuffer[4]))) = CoarseGain;
  *((uint16_t*)(&(txbuffer[6]))) = FineGain;
  memset(&txbuffer[8], 0x00, 2);
  rc = DoCommunication();
  return rc;
}

// sets the amplifier input polarity to positive .........................................................
McaComm::E_ERROR_FLAG McaComm::GBS_MCA_Comm::MMCA_SET_MCA_INPUT_AMPLIFIER_POS()
{
  McaComm::E_ERROR_FLAG rc = ERROR_OK;

  if (IsConnected() == false)
    return ERROR_INTERFACE;

  // set ADC input to amplifier
  txbuffer[2] = (uint8_t)CMD_SET_MCA_INPUT;
  txbuffer[3] = 0x00;
  txbuffer[4] = (uint8_t)ADC_AMPLIFIER;
  memset(&txbuffer[5], 0x00, 5);
  rc = DoCommunication();

  if (rc == ERROR_OK)
  {
    // set input polarity
    txbuffer[2] = (uint8_t)CMD_SET_INPUT_POLARITY;
    txbuffer[3] = 0x00;
    txbuffer[4] = (uint8_t)POLARITY_POSITIVE;
    memset(&txbuffer[5], 0x00, 5);
    rc = DoCommunication();
  }
  return rc;
}

// sets the amplifier input polarity to negative ..........................................................
McaComm::E_ERROR_FLAG McaComm::GBS_MCA_Comm::MMCA_SET_MCA_INPUT_AMPLIFIER_NEG()
{
  McaComm::E_ERROR_FLAG rc = ERROR_OK;

  if (IsConnected() == false)
    return ERROR_INTERFACE;

  // set ADC input to amplifier
  txbuffer[2] = (uint8_t)CMD_SET_MCA_INPUT;
  txbuffer[3] = 0x00;
  txbuffer[4] = (uint8_t)ADC_AMPLIFIER;
  memset(&txbuffer[5], 0x00, 5);
  rc = DoCommunication();

  if (rc == ERROR_OK)
  {
    // set input polarity
    txbuffer[2] = (uint8_t)CMD_SET_INPUT_POLARITY;
    txbuffer[3] = 0x00;
    txbuffer[4] = (uint8_t)POLARITY_NEGATIVE;
    memset(&txbuffer[5], 0x00, 5);
    rc = DoCommunication();
  }
  return rc;
}

// sets the PZC value and returns the PZC offset
// in case of an error the offset remains unchanged ..........................................................
McaComm::E_ERROR_FLAG McaComm::GBS_MCA_Comm::MMCA_SET_PZC_MANUAL(uint16_t PzcValue, int16_t* offset,
                                                                 uint16_t* MeasuredPulses, bool HighPrecision)
{
  McaComm::E_ERROR_FLAG rc = ERROR_OK;

  if (IsConnected() == false)
    return ERROR_INTERFACE;

  // pole zero cancellation
  txbuffer[2] = (uint8_t)CMD_SET_MEASURE_PZC;
  txbuffer[3] = 0x00;
  *((uint16_t*)(&(txbuffer[4]))) = 15;  // set PZC and measure offset
  *((uint16_t*)(&(txbuffer[6]))) = PzcValue;
  memset(&txbuffer[8], 0x00, 2);
  int tmp = ReadTimeout;
  ReadTimeout = 1000;  // needs some time for measurement to determine offset
  rc = DoCommunication();
  if (rc == ERROR_OK)
  {
    *MeasuredPulses = *((uint16_t*)(rxbuffer + preampleLength + 128));
    *offset = *((int16_t*)(rxbuffer + preampleLength + (HighPrecision ? 124 : 130))) * -1;
  }
  ReadTimeout = tmp;
  return rc;
}

// sets the low shaping time ................................................................................
McaComm::E_ERROR_FLAG McaComm::GBS_MCA_Comm::MMCA_SET_SHAPING_TIME_LOW()
{
  McaComm::E_ERROR_FLAG rc = ERROR_OK;

  if (IsConnected() == false)
    return ERROR_INTERFACE;

  txbuffer[2] = (uint8_t)CMD_SET_SHAPING_TIME;
  txbuffer[3] = 0x00;
  txbuffer[4] = 0x01;  // shaping time low
  memset(&txbuffer[5], 0x00, 5);
  rc = DoCommunication();
  return rc;
}

// sets the high shaping time ................................................................................
McaComm::E_ERROR_FLAG McaComm::GBS_MCA_Comm::MMCA_SET_SHAPING_TIME_HIGH()
{
  McaComm::E_ERROR_FLAG rc = ERROR_OK;

  if (IsConnected() == false)
    return ERROR_INTERFACE;

  txbuffer[2] = (uint8_t)CMD_SET_SHAPING_TIME;
  txbuffer[3] = 0x00;
  txbuffer[4] = 0x03;  // shaping time high
  memset(&txbuffer[5], 0x00, 5);
  rc = DoCommunication();
  return rc;
}

// Sets the high and low shaping time values. Only useable for MCA-527 ......................................
// shaping time = Low (High) * 0.1 us
McaComm::E_ERROR_FLAG McaComm::GBS_MCA_Comm::MMCA_SET_SHAPING_TIME_PAIR(uint16_t Low, uint16_t High)
{
  McaComm::E_ERROR_FLAG rc = ERROR_OK;

  if (IsConnected() == false)
    return ERROR_INTERFACE;

  *((uint16_t*)(&(txbuffer[2]))) = (uint16_t)CMD_SHAPING_TIME_PAIR;
  *((uint16_t*)(&(txbuffer[4]))) = Low;
  *((uint16_t*)(&(txbuffer[6]))) = High;
  memset(&txbuffer[8], 0x00, 2);
  rc = DoCommunication();
  return rc;
}

// Sets the trigger filter used for high and low shaping time. Only useable for MCA-527
// ......................................
McaComm::E_ERROR_FLAG McaComm::GBS_MCA_Comm::MMCA_SET_TRIGGER_FILTER(uint16_t Low, uint16_t High)
{
  McaComm::E_ERROR_FLAG rc = ERROR_OK;

  if (IsConnected() == false)
    return ERROR_INTERFACE;

  *((uint16_t*)(&(txbuffer[2]))) = (uint16_t)CMD_SET_TRIGGER_FILTER;
  txbuffer[4] = (uint8_t)Low;
  txbuffer[5] = 0x00;
  txbuffer[6] = (uint8_t)High;
  memset(&txbuffer[7], 0x00, 3);
  rc = DoCommunication();
  return rc;
}

McaComm::E_ERROR_FLAG McaComm::GBS_MCA_Comm::MMCA_SET_FLAT_TOP_TIME(uint16_t FlatTopTime)
{
  McaComm::E_ERROR_FLAG rc = ERROR_OK;

  if (IsConnected() == false)
    return ERROR_INTERFACE;

  *((uint16_t*)(&(txbuffer[2]))) = (uint16_t)CMD_SET_FLAT_TOP_TIME;
  txbuffer[4] = (uint8_t)FlatTopTime;
  memset(&txbuffer[5], 0x00, 5);
  rc = DoCommunication();
  return rc;
}

// protected functions -----------------------------------------------------------------------------------

McaComm::E_ERROR_FLAG McaComm::GBS_MCA_Comm::CheckFrame()  //..................................................
{
  if ((rxbuffer[0] != (char)0xA5) || (rxbuffer[1] != (char)0x5A))  // Preamble
    return ERROR_COMMUNICATION;
  else if ((rxbuffer[rxBufferLength - 2] == (char)0xB9) && (rxbuffer[rxBufferLength - 1] == (char)0x9B))
    return ERROR_OK;
  else if ((rxbuffer[rxBufferLength - 2] == (char)0xA8) && (rxbuffer[rxBufferLength - 1] == (char)0xAA))
    return ERROR_FILE_WRITING_IN_PROCESS;
  else if ((rxbuffer[rxBufferLength - 2] == (char)0xA9) && (rxbuffer[rxBufferLength - 1] == (char)0xAA))
    return ERROR_UNHANDLED_COMMAND;
  else if ((rxbuffer[rxBufferLength - 2] == (char)0xAA) && (rxbuffer[rxBufferLength - 1] == (char)0xAA))
    return ERROR_INVALID_PARAM;
  else if ((rxbuffer[rxBufferLength - 2] == (char)0xAB) && (rxbuffer[rxBufferLength - 1] == (char)0xAA))
    return ERROR_UNKNOWN_COMMAND;
  else if ((rxbuffer[rxBufferLength - 2] == (char)0xAC) && (rxbuffer[rxBufferLength - 1] == (char)0xAA))
    return ERROR_RUNNING_MEAS;
  else if ((rxbuffer[rxBufferLength - 2] == (char)0xAD) && (rxbuffer[rxBufferLength - 1] == (char)0xAA))
    return ERROR_VIOLATED_RIGHT;
  else if ((rxbuffer[rxBufferLength - 2] == (char)0xAE) && (rxbuffer[rxBufferLength - 1] == (char)0xAA))
    return ERROR_STOPPED_MEAS;
  else if ((rxbuffer[rxBufferLength - 2] == (char)0xAF) && (rxbuffer[rxBufferLength - 1] == (char)0xAA))
    return ERROR_WRONG_MODE;
  else
    return ERROR_COMMUNICATION;  // unknown end flag
}

// connection to an MCA is assumed
McaComm::E_ERROR_FLAG McaComm::GBS_MCA_Comm::DoCommunication()  //..............................................
{
  E_ERROR_FLAG rc = ERROR_COMMUNICATION;
  int attempts = CommAttempts;
  FT_STATUS mcaStatus;
  DWORD BytesWritten;
  DWORD BytesReceived;

  mcaStatus = FT_SetTimeouts(mcaHandle, ReadTimeout, WriteTimeout);
  do
  {
    FT_Purge(mcaHandle, FT_PURGE_RX | FT_PURGE_TX);
    mcaStatus = FT_Write(mcaHandle, txbuffer, txBufferLength, &BytesWritten);
    if ((mcaStatus == FT_OK) && (BytesWritten == txBufferLength))
    {
      int total = 0;
      do
      {
        mcaStatus = FT_Read(mcaHandle, rxbuffer + total, rxBufferLength - total, &BytesReceived);
        total += BytesReceived;
      } while ((mcaStatus == FT_OK) && (total < rxBufferLength) && (BytesReceived > 0));

      if (total == rxBufferLength)
        rc = CheckFrame();
    }
  } while (rc != ERROR_OK && --attempts > 0);

  return rc;
}

// Checks if the currently opened MCA answers with the specified baud rate
bool McaComm::GBS_MCA_Comm::CheckDevice(unsigned long baudRate)  //............................................
{
  // eventually set lower read timeout just for testing?
  FT_STATUS mcaStatus = FT_SetBaudRate(mcaHandle, baudRate);
  if (mcaStatus != FT_OK)
  {
    return false;
  }
  CurrentBaudrate = baudRate;
  E_ERROR_FLAG rc = MMCA_QUERY_STATE(0);
  return (rc == ERROR_OK);
}

// Checks if device can be opened and an MCA answers
// returns true if successful, otherwise false
bool McaComm::GBS_MCA_Comm::Connect(int index, unsigned long baudRate)  //....................................
{
  if (IsConnected() == false)
  {
    FT_STATUS mcaStatus;
    mcaStatus = FT_Open(index, &mcaHandle);
    // std::cout << "Open Device " << index << ": Status = " << mcaStatus << std::endl;

    if (mcaStatus == FT_OK)
    {
      connected = true;
      return CheckDevice(baudRate);
    }
    else
      return false;
  }
  else
    return false;
}
