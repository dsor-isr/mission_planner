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

#include <mca_comm/gbs_mca_data.h>

McaData::GBS_MCA_ROI::GBS_MCA_ROI(ushort boundary1, ushort boundary2)  //..............
{
  SetRoi(boundary1, boundary2);
}

void McaData::GBS_MCA_ROI::SetRoi(ushort boundary1, ushort boundary2)  //..............
{
  if (boundary1 == boundary2)
  {  // invalid ROI
    begin = 0;
    end = 0;
  }
  else if (boundary1 < boundary2)
  {
    begin = boundary1;
    end = boundary2;
  }
  else
  {
    begin = boundary2;
    end = boundary1;
  }
}

McaData::GBS_MCA_ROI& McaData::GBS_MCA_ROI::operator=(const GBS_MCA_ROI& roi)  //..............
{
  begin = roi.begin;
  end = roi.end;
  return *this;
}
bool McaData::GBS_MCA_ROI::operator==(const GBS_MCA_ROI& roi)  //.....................
{
  return ((begin == roi.begin) && (end == roi.end));
}
bool McaData::GBS_MCA_ROI::operator<(const GBS_MCA_ROI& roi)  //......................
{
  return ((begin < roi.begin) || ((begin == roi.begin) && (end < roi.end)));
}
bool McaData::GBS_MCA_ROI::operator>(const GBS_MCA_ROI& roi)  //......................
{
  return ((begin > roi.begin) || ((begin == roi.begin) && (end > roi.end)));
}

//---------------------------------------------------------------------------

McaData::GBS_MCA_ROIS::GBS_MCA_ROIS(ushort newChannels, bool newOverlap)
{
  if (newChannels < MinChannels)
    channels = MinChannels;
  else if (newChannels > MaxChannels)
    channels = MaxChannels;
  else
    channels = newChannels;
  overlap = newOverlap;
}

// fügt ROI sortiert ein ....................................................
// rc > 0 ROI wurde erfolgreich eingefügt, ROI-Nr = rc
// rc = 0: Einfügen war nicht möglich
/*int McaData::GBS_MCA_ROIS::AddRoi(GBS_MCA_ROI newRoi)
{
    if (!newRoi.Valid())
        return 0;
    if (newRoi.End() >= channels)
        return 0;

    int i;
    for (i=0; i<rois.count(); i++)
    {
        if (!(rois[i] < newRoi))
            break;
    }
    if (i < rois.count() && rois[i] == newRoi)   // ROI existiert schon
        return 0;
    if ( (overlap == false)
       &&( ( (i < rois.count())
           &&(rois[i].Begin() <= newRoi.End()))      // überlappend mit Nachfolger
         ||( (i > 0)
           &&(rois[i-1].End()) >= newRoi.Begin())))  // überlappend mit Vorgänger
        return 0;

    rois.insert(i, newRoi);
    return i+1;
}
/*
void McaData::GBS_MCA_ROIS::DeleteRoi(int RoiNr) //...................................
{
    if (  (RoiNr < rois.count())
       && (RoiNr > 0))
        rois.removeAt(RoiNr-1);
}

McaData::GBS_MCA_ROI McaData::GBS_MCA_ROIS::GetRoi(int RoiNr) const //..........................
{
    if (  (RoiNr > rois.count())
       || (RoiNr <= 0))
        return (GBS_MCA_ROI(0,0));  // ungültige ROI
    else
        return (rois[RoiNr-1]);
}

// returns 0xffff if RoiNr does not exist
ushort McaData::GBS_MCA_ROIS::GetBeginn(int RoiNr) const //............................
{
    if (  (RoiNr > rois.count())
       || (RoiNr <= 0))
        return (0xffff);
    else
        return (rois[RoiNr - 1].Begin());
}

// returns 0 if RoiNr does not exist
ushort McaData::GBS_MCA_ROIS::GetEnd(int RoiNr) const //...............................
{
    if (  (RoiNr > rois.count())
       || (RoiNr <= 0))
        return (0);
    else
        return (rois[RoiNr - 1].End());
}

// returns the number of the first ROI, where channel is inside //...........
// returns -1 else
int McaData::GBS_MCA_ROIS::IsRoi(ushort channel) const
{
    int rc = -1;
    for (int i = 0; i < rois.count(); i++)
    {
        if ( (rois[i].Begin() <= channel)
           &&(rois[i].End() >= channel))
        {
            rc = i+1;
            break;
        }
    }
    return rc;
}

// begin and end are output parameter .........................................
// if channel is within a ROI:
//   --> returns ROI-Nr, begin and end of the first matching ROI
// else
//   --> returns -1, begin and end = possible area to set a ROI
//       (depending on overlap)

int McaData::GBS_MCA_ROIS::IsRoi(ushort channel, ushort* begin, ushort* end) const
{
    int rc = -1;
    *begin = 0;
    *end = channels-1;

    for (int i = 0; i < rois.count(); i++)
    {
        if (rois[i].End() >= channel)
        {
            if (rois[i].Begin() <= channel)
            {
                rc = i+1;
                *begin = rois[i].Begin();
                *end = rois[i].End();
            }
            else
            {
                *end = rois[i].Begin()-1;
                if (i > 0)
                    *begin = rois[i-1].End()+1;
            }
            break;
        }
    }
    if ((rois.count() > 0) && (channel > rois.last().End()))
    {
        *begin = rois.last().End()+1;
    }
    return rc;
}*/

//----------------------------------------------------------------------------

McaData::GBS_MCA_ENERGY_CALIBRATION::GBS_MCA_ENERGY_CALIBRATION(int newChannels)
{
  Init(newChannels);
}

//.............................................................................
McaData::GBS_MCA_ENERGY_CALIBRATION::GBS_MCA_ENERGY_CALIBRATION(int newChannels, float newOffset, float newSlope)
{
  Init(newChannels);
  SetCoefficients(newOffset, newSlope);
}

//.............................................................................
McaData::GBS_MCA_ENERGY_CALIBRATION::GBS_MCA_ENERGY_CALIBRATION(int newChannels, float newChannel1, float newEnergy1,
                                                                float newChannel2, float newEnergy2)
{
  Init(newChannels);
  Set2Points(newChannel1, newEnergy1, newChannel2, newEnergy2);
}

//.............................................................................
McaData::GBS_MCA_ENERGY_CALIBRATION::GBS_MCA_ENERGY_CALIBRATION(int newChannels, float newChannel1, float newEnergy1,
                                                                float newChannel2, float newEnergy2, float newChannel3,
                                                                float newEnergy3)
{
  Init(newChannels);
  Set3Points(newChannel1, newEnergy1, newChannel2, newEnergy2, newChannel3, newEnergy3);
}

void McaData::GBS_MCA_ENERGY_CALIBRATION::Reset()  //...................................
{
  slope = 0.0F;
  offset = 0.0F;
  square = 0.0F;
  points = 0;
}

//..............................................................................
void McaData::GBS_MCA_ENERGY_CALIBRATION::SetCoefficients(float newOffset, float newSlope)
{
  if (newSlope <= 0.0)
    return;

  slope = newSlope;
  offset = newOffset;
  points = 0;
}

//.............................................................................
void McaData::GBS_MCA_ENERGY_CALIBRATION::Set2Points(float newChannel1, float newEnergy1, float newChannel2,
                                                     float newEnergy2)
{
  if ((newEnergy1 == newEnergy2) || (newChannel1 == newChannel2))
  {
    Reset();  // invalid
  }
  else
  {
    points = 2;
    channel1 = newChannel1;
    energy1 = newEnergy1;
    channel2 = newChannel2;
    energy2 = newEnergy2;
    channel3 = 0.0F;
    energy3 = 0.0F;
    Do2PointCalibration();
  }
}

//.............................................................................
void McaData::GBS_MCA_ENERGY_CALIBRATION::Set3Points(float newChannel1, float newEnergy1, float newChannel2,
                                                     float newEnergy2, float newChannel3, float newEnergy3)
{
  if ((newEnergy1 == newEnergy2) || (newChannel1 == newChannel2) || (newEnergy1 == newEnergy3) ||
      (newChannel1 == newChannel3) || (newEnergy2 == newEnergy3) || (newChannel2 == newChannel3))
  {
    Reset();  // invalid
  }
  else
  {
    points = 3;
    channel1 = newChannel1;
    energy1 = newEnergy1;
    channel2 = newChannel2;
    energy2 = newEnergy2;
    channel3 = newChannel3;
    energy3 = newEnergy3;
    Do3PointCalibration();
  }
}

// returns channel of an energy calibration point
float McaData::GBS_MCA_ENERGY_CALIBRATION::GetChannel(short i) const  //.......................
{
  switch (i)
  {
    case 1:
      return channel1;
    case 2:
      return channel2;
    case 3:
      return channel3;
    default:
      return 0.0F;
  }
}

// returns energy of an energy calibration point
float McaData::GBS_MCA_ENERGY_CALIBRATION::GetEnergy(short i) const  //........................
{
  switch (i)
  {
    case 1:
      return energy1;
    case 2:
      return energy2;
    case 3:
      return energy3;
    default:
      return 0.0F;
  }
}

float McaData::GBS_MCA_ENERGY_CALIBRATION::CalculateEnergy(float channel) const  //............
{
  float energy = channel;
  if (!Valid())
    throw EnergyCalibInvalid();

  energy = (slope * channel) + offset;
  if (points == 3)  // 3-Punkt-Kalibrierung
    energy += (square * (channel * channel));

  return energy;
}

float McaData::GBS_MCA_ENERGY_CALIBRATION::CalculateChannel(float energy) const  //.....................
{
  float tmpChannel = energy;
  if (!Valid())
    throw EnergyCalibInvalid();

  if (points == 3)  // quadratic equation
  {
    tmpChannel = (-slope + (float)sqrt((slope * slope) - (4 * square * (offset - energy)))) / (2 * square);
  }
  else  // linear
  {
    tmpChannel = (energy - offset) / slope;
  }
  return tmpChannel;
}

void McaData::GBS_MCA_ENERGY_CALIBRATION::Init(int newChannels)  //.............................
{
  if (newChannels < MinChannels)
    channels = MinChannels;
  else if (newChannels > MaxChannels)
    channels = MaxChannels;
  else
    channels = newChannels;
  Reset();
}

void McaData::GBS_MCA_ENERGY_CALIBRATION::Do2PointCalibration()  //........................
{
  float a1 = (energy1 - energy2) / (channel1 - channel2);
  float b1 = energy1 - (channel1 * a1);

  CheckCalibration(a1, b1, 0.0F);
}

void McaData::GBS_MCA_ENERGY_CALIBRATION::Do3PointCalibration()  //..............................
{
  // Richtig 3 Punkte quadratisch kalibrieren (Algorithmus aus Identify übernommen)
  float c1 = (((channel1 - channel3) * (energy1 - energy2)) - ((channel1 - channel2) * (energy1 - energy3))) /
             (((channel1 * channel1) - (channel2 * channel2)) * (channel1 - channel3) -
              ((channel1 * channel1) - (channel3 * channel3)) * (channel1 - channel2));
  float a1 = ((energy1 - energy2) / (channel1 - channel2)) -
             ((c1 * ((channel1 * channel1) - (channel2 * channel2))) / (channel1 - channel2));
  float b1 = energy1 - (a1 * channel1) - (c1 * (channel1 * channel1));

  CheckCalibration(a1, b1, c1);
}

// Formeln aus Identify übernommen ...................................................
void McaData::GBS_MCA_ENERGY_CALIBRATION::CheckCalibration(float a, float b, float c)
{
  // Gültigkeit der Kalibrierung überprüfen: monoton steigend und in den oberen 2/3 positiv
  // Achtung: Funktioniert nur, wenn der erste Kanal = 0 ist.
  int i = channels / 3;
  if (((a * i) + b + (c * i * i) > 0) && (a > 0) && (a + (2 * c * channels) > 0))
  {
    slope = a;
    offset = b;
    square = c;
  }
  else
  {
    Reset();
  }
}

// functions -------------------------------------------------------------------------------------------

uint16_t McaData::GetMaxHv(struct McaComm::QUERY_STATE527* state527)  //..................................
{
  if (state527 == 0)  // MCA 166
    return 3600;
  else
    return state527->PowerModuleMaxHv;
}

uint16_t McaData::GetMaxChannels(struct McaComm::QUERY_STATE527* state527)  //..............................
{
  if (state527 == 0)  // MCA 166
    return 4096;
  else
    return state527->MaxChannels;
}

uint16_t McaData::GetMaxUld(uint16_t channels, struct McaComm::QUERY_STATE527* state527)  //.................
{
  if (state527 == 0)  // MCA 166
    return (channels - (channels >> 5)) - 1;
  else
    return channels - 1;
}

uint16_t McaData::GetMaxFineGain(struct McaComm::QUERY_STATE527* state527)  //................................
{
  if (state527 == 0)  // MCA 166
    return 15000;
  else
    return 65000;
}

uint16_t McaData::GetMaxFlattop(struct McaComm::QUERY_STATE527* state527, struct McaComm::QUERY_STATE527_EX* state527ex)
{
  if ((state527ex == 0) ||                              // MCA 166
      (state527 == 0) || (state527->FwVersion < 1307))  // Since firmware version 13.07.
    return 50;
  else
    return state527ex->MaxFlattopTime;
}

//--------------------------------------------------------------------------------------------------------

McaData::GBS_MCA_Data::GBS_MCA_Data()
{
  Clear();
}

McaData::GBS_MCA_Data::~GBS_MCA_Data()  //...................................
{
}

// initialize all data as "not set" ........................................
void McaData::GBS_MCA_Data::Clear()
{
  ApplId = "";
  ApplName = "";
  DeviceType = MCATYPE_NONE;
  SerialNr = 0;
  HwVersion = 0;
  FwVersion = 0;

  SpecRem = "";
  memset(&DateMea, 0, sizeof(DateMea));
  LifeTime = 0;
  RealTime = 0.0;
  FirstChannel = 0;
  LastChannel = 0;
  Rois = 0;  // null pointer
  Channels = 0;
  Lld = 0;
  Uld = 0;
  McaPresets = PRESET_LIVE;
  PresetValue = 0;
  PresetRoi = 0;
  PzcValue = 0;
  PzcTimeParaLow = 0;
  PzcTimeParaHigh = 0;
  FastDiscr = 0;
  SlowDiscr = 0;
  Threshold = 0.0;
  CoarseGain = 0;
  FineGain = 0;
  TriggerFilter = 0;
  FlatTop = 0;
  Dtc = 0;
  ActShapingTime = 0.0f;
  AdcInput = ADC_AMPLIFIER;
  AmplifierPolarity = POLARITY_POSITIVE;  // correct default value?
  PurState = PUR_ON;
  StabOn = false;
  StabRoiBegin = 0;
  StabRoiEnd = 0;
  StabChannel = 0;
  StabTime = 0;
  StabArea = 0;
  PowerSwitches = 0;
  HighVoltage = 0;
  HvInhibitMode = INHIBIT_OFF;
  HvPolarity = POLARITY_POSITIVE;  // correct default value?
  McsChannels = 0;
  McsInput = MCS_INPUT_TTL;
  McsTimePerChannel = 0;
  McaRepeat = 0;
  Mode = MODE_MCA;
  Tdf = 0;
  Iplus12 = 0;
  Iminus12 = 0;
  Iplus24 = 0;
  Iminus24 = 0;
  Ibattery = 0;
  Ihv = 0;
  Icharger = 0;
  Ubattery = 0;
  Uhvs = 0;
  Uplus12 = 0.0;
  Uminus12 = 0.0;
  Uplus24 = 0.0;
  Uminus24 = 0.0;
  Counts = 0;
  PdCounts = 0;
  DeadTime = 0;
  BusyTime = 0;
  StabOffset = 0;
  StabOffsetMin = 0;
  StabOffsetMax = 0;
  StabCounter = 0;
  RecCounter = 0;
  RecErrorCnt = 0;
  Integral = 0;
  RoisIntegral = 0;   // null pointer
  RoisCentroid = 0;   // null pointer
  RoisFwhm = 0;       // null pointer
  RoisArea = 0;       // null pointer
  RoisAreaError = 0;  // null pointer
  DetectorTemperature = 0;
  McaTemperature = 0;
}

void McaData::GBS_MCA_Data::FillFromMca(McaComm::QUERY_STATE* state,  //...................
                                        McaComm::QUERY_STATE527* state527, McaComm::QUERY_POWER* power,
                                        McaComm::QUERY_SYSTEM_DATA* systemData)
{
  ApplId = "GBSM";  // can be overwritten, if required
  ApplName = "(GBS MCA Data library)";

  if ((state->FirmwareVersion != 0xFFFF) || (state527 == 0))
  {
    DeviceType = MCATYPE_MCA166;
    HwVersion = state->HardwareVersion;
    FwVersion = state->FirmwareVersion;
  }
  else
  {
    switch (state527->HwModification)
    {
      case 0:
        DeviceType = MCATYPE_MCA527_FULL;
        break;
      case 1:
        DeviceType = MCATYPE_MCA527_LITE;
        break;
      case 2:
        DeviceType = MCATYPE_MCA527_OEM;
        break;
      case 3:
        DeviceType = MCATYPE_MCA527_MICRO;
        break;
      case 4:
        DeviceType = MCATYPE_MCA527_NANO;
        break;
      default:
        DeviceType = MCATYPE_MCA166;  // ??? undefined
        break;
    }
    HwVersion = state527->HwVersion;
    FwVersion = state527->FwVersion;
  }

  SerialNr = state->McaNumber;
  McaComm::FromMcaTime(state->StartTime, &DateMea);
  if ((DeviceType == MCATYPE_MCA166) || (state527 == 0))
  {
    // real time from the MCA-166 is to small --> round life time up (round dead time down)
    LifeTime = state->RealTime - (state->DeadTime / 1000);
    Threshold = state->Threshold;
    DetectorTemperature = (float)TemperatureNotAvailable;
    McaTemperature = (float)TemperatureNotAvailable;
  }
  else
  {
    // round regularly
    LifeTime = (((state->RealTime * 1000) - state->DeadTime) + 500) / 1000;
    Threshold = state527->ThresholdTenths * 0.1f;
    if (state->McaState == (uint16_t)STATE_RUN)
    {
      if (state527->DetectorTemperature == TemperatureNotAvailable)
        DetectorTemperature = (float)TemperatureNotAvailable;
      else
        DetectorTemperature = state527->DetectorTemperature * 0.0078125;
      if (state527->McaTemperature == TemperatureNotAvailable)
        McaTemperature = (float)TemperatureNotAvailable;
      else
        McaTemperature = state527->McaTemperature * 0.0078125;
    }
    else
    {
      if (state527->DetectorTemperatureAtStop == TemperatureNotAvailable)
        DetectorTemperature = (float)TemperatureNotAvailable;
      else
        DetectorTemperature = state527->DetectorTemperatureAtStop * 0.0078125;
      if (state527->McaTemperatureAtStop == TemperatureNotAvailable)
        McaTemperature = (float)TemperatureNotAvailable;
      else
        McaTemperature = state527->McaTemperatureAtStop * 0.0078125;
    }
    if (state->Dtc == 1)
      TriggerFilter = state527->TriggerFilterLow;
    else
      TriggerFilter = state527->TriggerFilterHigh;
    FlatTop = state527->FlatTopTime;
  }
  RealTime = state->RealTime;
  // ToDo: use precise realtime from MMCA_QUERY_STATE527_EX

  FirstChannel = 0;
  LastChannel = Spectrum.size() - 1;

  HvPolarity = (E_Polarity)state->DetectorBiasPoly;
  HighVoltage = state->DetectorBias;
  HvInhibitMode = (E_InhibitMode)state->HvInhibitMode;
  Mode = (E_McaMode)state->McaMode;
  McaPresets = (E_McaPresets)state->Presets;
  PresetValue = state->PresetValue;
  PzcValue = state->PzcValue;
  PzcTimeParaLow = state->PzcDtc1Offset;
  PzcTimeParaHigh = state->PzcDtc3Offset;
  FastDiscr = state->FastDiscr;
  SlowDiscr = state->SlowDiscr;
  StabOn = state->StabState > 0;
  StabRoiBegin = state->StabRoiBegin;
  StabRoiEnd = state->StabRoiEnd;
  StabChannel = state->StabState;
  PowerSwitches = state->PreampPower;
  Channels = state->Channels;
  Lld = state->Lld;
  Uld = state->Uld;
  CoarseGain = state->CoarseGain;
  FineGain = state->FineGain;
  McaRepeat = state->Repeat;
  Tdf = state->Tdf;
  if (state->Dtc == 1)
  {
    Dtc = 1;
    if (systemData != 0)
      ActShapingTime = systemData->LowShapingTime * 0.1;
  }
  else
  {
    Dtc = 2;
    if (systemData != 0)
      ActShapingTime =
          ((systemData->HighShapingTime + (systemData->HighShapingTime < systemData->LowShapingTime ? 256 : 0)) * 0.1);
  }

  AdcInput = (E_InputADC)state->McaInputAdc;
  AmplifierPolarity = (E_Polarity)state->McaInputPol;
  PurState = (E_McaPur)state->McaPur;
  DeadTime = state->DeadTime;
  BusyTime = state->BusyTime;
  Integral = CalculateIntegral();

  if (power != 0)
  {
    if (state->McaState == (uint16_t)STATE_RUN)
    {
      Iplus12 = power->P12PrimaryCurrent;
      Iminus12 = power->M12PrimaryCurrent;
      Iplus24 = power->P24PrimaryCurrent;
      Iminus24 = power->M24PrimaryCurrent;
      Ibattery = power->BatteryCurrent;
      Ihv = power->HvPrimaryCurrent;
      Icharger = power->ChargerCurrent;
      Ubattery = power->BatteryVoltage;
      Uhvs = (uint32_t)round(power->HighVoltage * 1.2);
      if (DeviceType != MCATYPE_MCA166)
      {
        Uplus12 = power->P12Voltage * 0.0625;
        Uminus12 = power->M12Voltage * 0.0625;
        Uplus24 = power->P24Voltage * 0.125;
        Uminus24 = power->M24Voltage * 0.125;
      }
    }
    else
    {
      Iplus12 = power->P12PrimaryCurrentAtStop;
      Iminus12 = power->M12PrimaryCurrentAtStop;
      Iplus24 = power->P24PrimaryCurrentAtStop;
      Iminus24 = power->M24PrimaryCurrentAtStop;
      Ibattery = power->BatteryCurrentAtStop;
      Ihv = power->HvPrimaryCurrentAtStop;
      Icharger = power->ChargerCurrentAtStop;
      Ubattery = power->BatteryVoltageAtStop;
      Uhvs = (uint32_t)round(power->HighVoltageAtStop * 1.2);
      if (DeviceType != MCATYPE_MCA166)
      {
        Uplus12 = power->P12VoltageAtStop * 0.0625;
        Uminus12 = power->M12VoltageAtStop * 0.0625;
        Uplus24 = power->P24VoltageAtStop * 0.125;
        Uminus24 = power->M24VoltageAtStop * 0.125;
      }
    }
  }
  if (systemData != 0)
  {
    Counts = systemData->Imp;
    PdCounts = systemData->PdCounter;
    StabTime = systemData->StabTime;
    StabArea = systemData->StabArea;
    RecCounter = systemData->RecCounter;
    RecErrorCnt = systemData->RecErrorCounter;
  }
}

// returns number of ROIs .......................................................................
int McaData::GBS_MCA_Data::GetRoiNumber() const
{
  if (Rois == 0)
    return 0;
  else
    return Rois->GetCount();
}

uint64_t McaData::GBS_MCA_Data::CalculateIntegral()  //..............................................
{
  uint64_t integral = 0;
  for (int i = FirstChannel; i <= LastChannel; i++)
    integral += Spectrum[i];

  return integral;
}

bool McaData::IsMca527(struct McaComm::QUERY_STATE* state)  //.......................................
{
  if ((state == 0) || (state->FirmwareVersion != 0xFFFF))
    return false;  // MCA 166
  else
    return true;  // MCA 527
}
