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

#ifndef GBS_MCA_SPEFILE_H
#define GBS_MCA_SPEFILE_H

#include <mca_comm/gbs_mca_data.h>

namespace SpeFile
{
// Block identifiers -------------------------------------------------------------------------
const std::string BlockAdc = "$ADC:";
const std::string BlockApplId = "$APPLICATION_ID:";
const std::string BlockBt = "$BT:";
const std::string BlockCounts = "$COUNTS:";
const std::string BlockData = "$DATA:";
const std::string BlockDateMea = "$DATE_MEA:";
const std::string BlockDevId = "$DEVICE_ID:";
const std::string BlockDt = "$DT:";
const std::string BlockDtc = "$DTC:";
const std::string BlockEnd = "$END:";                // ToDo
const std::string BlockEnerData = "$ENER_DATA:";     // ToDo
const std::string BlockEnerDataX = "$ENER_DATA_X:";  // ToDo
const std::string BlockEnerFit = "$ENER_FIT:";       // ToDo
const std::string BlockFastDiscr = "$FAST_DISCR:";
const std::string BlockFlatTop = "$FLAT_TOP:";
const std::string BlockGain = "$GAIN_VALUE:";
const std::string BlockHv = "$HV:";
const std::string BlockInput = "$INPUT:";
const std::string BlockIntegral = "$SPEC_INTEGRAL:";
const std::string BlockTriggerF = "$MCA_527_TRIGGER_FILTER:";
const std::string BlockMcaRepeat = "$MCA_REPEAT:";    // ToDo
const std::string BlockMCAtouch = "$MCATOUCH_INFO:";  // ToDo
const std::string BlockMcsChan = "$MCS_CHANNELS:";    // ToDo
const std::string BlockMcsInput = "$MCS_INPUT:";      // ToDo
const std::string BlockMcsSweeps = "$MCS_SWEEPS:";    // ToDo
const std::string BlockMcsTime = "$MCS_TIME:";        // ToDo
const std::string BlockMeasTim = "$MEAS_TIM:";
const std::string BlockMode = "$MODE:";
const std::string BlockPdCounts = "$PD_COUNTS:";
const std::string BlockPower = "$POWER:";  // ToDo
const std::string BlockPowerStat = "$POWER_STATE:";
const std::string BlockPresets = "$PRESETS:";
const std::string BlockPur = "$PUR:";
const std::string BlockPzc = "$PZC_VALUE:";
const std::string BlockRecCnt = "$REC_COUNTER:";          // ToDo
const std::string BlockRecError = "$REC_ERROR_COUNTER:";  // ToDo
const std::string BlockRoi = "$ROI:";                     // ToDo
const std::string BlockRoiInfo = "$ROI_INFO:";            // ToDo
const std::string BlockRt = "$RT:";
const std::string BlockSlowDiscr = "$SLOW_DISCR:";
const std::string BlockSpecRem = "$SPEC_REM:";
const std::string BlockStab = "$STAB:";
const std::string BlockStabCnt = "$STAB_COUNTER:";      // ToDo
const std::string BlockStabOffs = "$STAB_OFFSET:";      // ToDo
const std::string BlockStabOMax = "$STAB_OFFSET_MAX:";  // ToDo
const std::string BlockStabOMin = "$STAB_OFFSET_MIN:";  // ToDo
const std::string BlockStabParam = "$STAB_PARAM:";
const std::string BlockTdf = "$TDF:";  // ToDo
const std::string BlockTemp = "$TEMPERATURE:";
const std::string BlockThr = "$THR:";

class GBS_MCA_SpeFile;

struct SpeBlock  //----------------------------------------------------------------------
{
public:
  std::string blockId;
  void (GBS_MCA_SpeFile::*writeMethod)();
  bool (GBS_MCA_SpeFile::*blockCondition)();  // block is written if blockCondition is true

  SpeBlock(std::string Id = "", void (GBS_MCA_SpeFile::*wm)() = 0, bool (GBS_MCA_SpeFile::*bc)() = 0)
  {
    blockId = Id;
    writeMethod = wm;
    blockCondition = bc;
  }
};

class GBS_MCA_SpeFile  //-----------------------------------------
{
protected:
  McaData::GBS_MCA_Data* SpeData;

public:
  GBS_MCA_SpeFile();
  bool WriteSpectrum(std::string FileName, McaData::GBS_MCA_Data* data);

protected:
  // write methods .................................................................
  void WriteApplId();     // Application Id
  void WriteDevId();      // Device Id
  void WriteSpecRem();    // Comment and filename
  void WriteDateMea();    // Start date and time of measurement
  void WriteMeasTime();   // Measurement time in seconds
  void WriteData();       // Spectral data
  void WriteRt();         // Real time [s]
  void WriteDt();         // Dead time [ms]
  void WriteIntegral();   // Counts in the spectrum
  void WriteHv();         // High voltage
  void WriteMode();       // MCA or MCS mode
  void WritePresets();    // MCA Presets
  void WriteAdc();        // ADC resolution
  void WriteThr();        // Threshold in %
  void WriteGain();       // Amplifier gain
  void WriteTriggerF();   // Trigger filter
  void WriteFlatTop();    // Flattop time
  void WriteInput();      // ADC input and polarity
  void WritePowerStat();  // Power state at the end of the measurement
  void WriteTemp();       // Temperature
  void WritePzc();        // PZC Settings
  void WriteDtc();        // Shaping time
  void WriteCounts();     // Integral counts
  void WritePdCounts();   // Integral peak detector counts
  void WriteBt();         // Busy time
  void WriteFastDiscr();  // Fast discriminator level
  void WriteSlowDiscr();  // Slow discriminator level
  void WritePur();        // State of the pile up rejector
  void WriteStab();       // Stabilization
  void WriteStabParam();  // Stabilization parameter
  void WritePower();      // Preamplifier power supply
  void WriteMcaRepeat();  // Number of sweeps
  void WriteTdf();        // Dead time correction factor (ns)
  void WriteRecCnt();     // Counter of received commands
  void WriteRecError();   // Counter of received commands with error

  // write conditions ..............................................................
  bool CondTrue();
  bool CondSpectrumData();
  bool CondMcaMode();
  bool CondMca527();
  bool CondMca166();

  std::vector<SpeBlock> blocks;
  FILE* SpectrumStream;
  std::string SpectrumFileName;
};

}  // namespace SpeFile

#endif  // GBS_MCA_SPEFILE_H
