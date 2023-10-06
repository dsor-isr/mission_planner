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

#ifndef GBS_MCA_DATA_H
#define GBS_MCA_DATA_H

#include <mca_comm/gbs_mca_comm.h>
#include <mca_comm/gbs_mca_types.h>

#include <math.h>
#include <vector>

namespace McaData
{
// Exceptions ----------------------------------------------------------------------
struct EnergyCalibInvalid
{
};

// Region Of Interest --------------------------------------------------------------
class GBS_MCA_ROI
{
protected:
  ushort begin;
  ushort end;

public:
  GBS_MCA_ROI() : begin(0), end(0)
  {
  }
  GBS_MCA_ROI(ushort, ushort);
  GBS_MCA_ROI(const GBS_MCA_ROI& roi) : begin(roi.begin), end(roi.end)
  {
  }  // Copy constructor

  GBS_MCA_ROI& operator=(const GBS_MCA_ROI& roi);
  bool operator==(const GBS_MCA_ROI& roi);
  bool operator<(const GBS_MCA_ROI& roi);
  bool operator>(const GBS_MCA_ROI& roi);

  void SetRoi(ushort, ushort);
  inline ushort Begin() const
  {
    return begin;
  }
  inline ushort End() const
  {
    return end;
  }
  inline bool Valid() const
  {
    return (begin < end);
  }
};

//-------------------------------------------------------------------------------------
class GBS_MCA_ROIS
{
protected:
  std::list<GBS_MCA_ROI> rois;  // sortierte Liste der ROIs
  bool overlap;
  ushort channels;  // number of channels in the related spectrum

public:
  GBS_MCA_ROIS(ushort Channels, bool Overlap = false);

  int GetCount() const
  {
    return rois.size();
  }  // Anzahl Rois
  bool GetOverlap() const
  {
    return overlap;
  }
  ushort GetChannels() const
  {
    return channels;
  }
  int AddRoi(GBS_MCA_ROI);
  void DeleteRoi(int RoiNr);
  void ClearRois()
  {
    rois.clear();
  }
  GBS_MCA_ROI GetRoi(int RoiNr) const;
  ushort GetBeginn(int RoiNr) const;
  ushort GetEnd(int RoiNr) const;
  int IsRoi(ushort channel) const;
  int IsRoi(ushort channel, ushort* begin, ushort* end) const;
};

//-------------------------------------------------------------------------------------
class GBS_MCA_ENERGY_CALIBRATION
{
protected:
  ushort points;
  float channel1;
  float energy1;
  float channel2;
  float energy2;
  float channel3;
  float energy3;
  float slope;
  float offset;
  float square;
  int channels;  // Anzahl Kanäle

  void Init(int newChannels);
  void Do2PointCalibration();
  void Do3PointCalibration();
  void CheckCalibration(float a, float b, float c);

public:
  GBS_MCA_ENERGY_CALIBRATION(int newChannels);
  GBS_MCA_ENERGY_CALIBRATION(int newChannels, float newOffset, float newSlope);
  GBS_MCA_ENERGY_CALIBRATION(int newChannels, float newChannel1, float newEnergy1, float newChannel2, float newEnergy2);
  GBS_MCA_ENERGY_CALIBRATION(int newChannels, float newChannel1, float newEnergy1, float newChannel2, float newEnergy2,
                             float newChannel3, float newEnergy3);

  void Reset();
  void SetCoefficients(float newOffset, float newSlope);
  void Set2Points(float newChannel1, float newEnergy1, float newChannel2, float newEnergy2);
  void Set3Points(float newChannel1, float newEnergy1, float newChannel2, float newEnergy2, float newChannel3,
                  float newEnergy3);

  float GetSlope() const
  {
    return slope;
  }
  float GetOffset() const
  {
    return offset;
  }
  float GetSquare() const
  {
    return square;
  }
  ushort GetPoints() const
  {
    return points;
  }  // number of calibration points
  int GetChannels() const
  {
    return channels;
  }  // number of spectrum channels
  bool Valid() const
  {
    return (slope > 0.0F);
  }  // energy calibration valid?
  float GetChannel(short i) const;
  float GetEnergy(short i) const;

  float CalculateEnergy(float channel) const;
  float CalculateChannel(float energy) const;
};

// Functions -------------------------------------------------------------------------

uint16_t GetMaxHv(struct McaComm::QUERY_STATE527*);
uint16_t GetMaxChannels(struct McaComm::QUERY_STATE527*);
uint16_t GetMaxUld(uint16_t, struct McaComm::QUERY_STATE527*);
uint16_t GetMaxFineGain(struct McaComm::QUERY_STATE527*);
uint16_t GetMaxFlattop(struct McaComm::QUERY_STATE527*, struct McaComm::QUERY_STATE527_EX*);
bool IsMca527(struct McaComm::QUERY_STATE*);

struct GBS_MCA_Data  //--------------------------------------
{
  std::string ApplId;    // Application ID (4 characters)
  std::string ApplName;  // Application name and version
  E_McaTyp DeviceType;
  uint16_t SerialNr;   // MCA serial number
  uint16_t HwVersion;  // Hardware version
  uint16_t FwVersion;  // Firmware version

  std::string SpecRem;             // Operator remarks
  struct tm DateMea;               // Start date and time of measurement
  uint32_t LifeTime;               // measurement time in seconds
  float RealTime;                  // Real time in s
  uint32_t FirstChannel;           // for saved Spektrum
  uint32_t LastChannel;            // for saved Spektrum
  std::vector<uint32_t> Spectrum;  // !! must be handled by user
  /**/ GBS_MCA_ROIS* Rois;           // regions of interest
  uint16_t Channels;
  uint16_t Lld;  // lower level discriminator channel
  uint16_t Uld;  // upper level discriminator channel
  E_McaPresets McaPresets;
  uint32_t PresetValue;
  /**/ uint16_t PresetRoi;  // ROI number
  uint16_t PzcValue;
  uint16_t PzcTimeParaLow;
  uint16_t PzcTimeParaHigh;
  uint16_t FastDiscr;  // factory setting for auto threshold
  uint16_t SlowDiscr;  // factory setting for auto threshold
  float Threshold;
  uint16_t CoarseGain;  // Amplifier gain
  uint16_t FineGain;    // Amplifier gain
  uint16_t TriggerFilter;
  uint16_t FlatTop;  // Flattop time
  uint16_t Dtc;      // 1 = low shaping time, 2 = high shaping time
  float ActShapingTime;
  E_InputADC AdcInput;
  E_Polarity AmplifierPolarity;
  E_McaPur PurState;
  bool StabOn;
  uint16_t StabRoiBegin;  // ROI limits of the stabilization peak
  uint16_t StabRoiEnd;
  uint16_t StabChannel;  // Stabilization target channel
  uint16_t StabTime;
  uint32_t StabArea;
  uint16_t PowerSwitches;  // Preamplifier power supply
  uint16_t HighVoltage;
  E_Polarity HvPolarity;
  E_InhibitMode HvInhibitMode;
  /**/ uint16_t McsChannels;  // Multi channel scaler setup
  /**/ E_McsInput McsInput;
  /**/ float McsTimePerChannel;  // in ms
  uint16_t McaRepeat;          // Number if sweeps
  E_McaMode Mode;
  uint16_t Tdf;  // dead time correction factor
  uint32_t Iplus12;
  uint32_t Iminus12;
  uint32_t Iplus24;
  uint32_t Iminus24;
  uint32_t Ibattery;
  uint32_t Ihv;
  uint32_t Icharger;
  uint32_t Ubattery;
  uint32_t Uhvs;
  float Uplus12;
  float Uminus12;
  float Uplus24;
  float Uminus24;
  uint64_t Counts;           // integral counts
  uint64_t PdCounts;         // integral peak detector counts
  uint32_t DeadTime;         // in ms
  uint32_t BusyTime;         // in ms
  /**/ int32_t StabOffset;     // current offset
  /**/ int32_t StabOffsetMin;  // minimal offset
  /**/ int32_t StabOffsetMax;  // maximal offset
  /**/ uint32_t StabCounter;   // stabilization cycles
  uint32_t RecCounter;       // counter of received commands
  uint32_t RecErrorCnt;      // counter of received commands with errors
  uint64_t Integral;         // counts in the spectrum
  /**/ long* RoisIntegral;     // calculated ROI information
  /**/ double* RoisCentroid;
  /**/ double* RoisFwhm;
  /**/ long* RoisArea;
  /**/ long* RoisAreaError;
  float DetectorTemperature;
  float McaTemperature;

  // additional special or new data, possibly handled later:
  // $MCATOUCH_INFO:   - IAEA inspection information
  // $MCS_AMP_DATA:    - MCS amplitude spectral data
  // $MCA_527_OFFSET_DAC:
  // $MCA_527_TRIGGER_LEVEL:
  // $MCA_527_BASELINE_RESTORING:
  // $MCA_527_JITTER_CORRECTION:
  // $MCA_527_LF_REJECTION:
  // $MCA_527_GATING:
  // $MCS_SWEEPS:  --> MCS repeat mode type
  // $MCS_REPEAT:  --> MCA repeat mode type
  // $WINSPEC_INFO:
  // $WINSCAN_INFO:
  // $INSP_INFO:
  // $CRIT_CHECK_PARAMS:, $CRIT_CHECK_RESULT: - criticality check data
  // $NAIGEM_CALIB:, $NAIGEM_EVAL: - uranium enrichment

public:
  GBS_MCA_Data();
  GBS_MCA_Data(GBS_MCA_Data*);
  ~GBS_MCA_Data();

  void Clear();
  void FillFromMca(struct McaComm::QUERY_STATE*, struct McaComm::QUERY_STATE527*, struct McaComm::QUERY_POWER*,
                   struct McaComm::QUERY_SYSTEM_DATA*);
  int GetRoiNumber() const;  // Number of Rois
  uint64_t CalculateIntegral();
};

}  // namespace McaData

#endif  // GBS_MCA_DATA_H
