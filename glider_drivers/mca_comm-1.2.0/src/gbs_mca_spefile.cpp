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

#include <mca_comm/gbs_mca_spefile.h>

// public methods --------------------------------------------------------------------------------

SpeFile::GBS_MCA_SpeFile::GBS_MCA_SpeFile()
{
  SpeData = 0;  // null pointer
  SpectrumFileName = "";
  blocks.insert(blocks.end(), SpeBlock(BlockApplId, &GBS_MCA_SpeFile::WriteApplId, &GBS_MCA_SpeFile::CondTrue));
  blocks.insert(blocks.end(), SpeBlock(BlockDevId, &GBS_MCA_SpeFile::WriteDevId, &GBS_MCA_SpeFile::CondTrue));
  blocks.insert(blocks.end(), SpeBlock(BlockSpecRem, &GBS_MCA_SpeFile::WriteSpecRem, &GBS_MCA_SpeFile::CondTrue));
  blocks.insert(blocks.end(), SpeBlock(BlockDateMea, &GBS_MCA_SpeFile::WriteDateMea, &GBS_MCA_SpeFile::CondTrue));
  blocks.insert(blocks.end(), SpeBlock(BlockMeasTim, &GBS_MCA_SpeFile::WriteMeasTime, &GBS_MCA_SpeFile::CondTrue));
  blocks.insert(blocks.end(), SpeBlock(BlockData, &GBS_MCA_SpeFile::WriteData, &GBS_MCA_SpeFile::CondSpectrumData));
  blocks.insert(blocks.end(), SpeBlock(BlockAdc, &GBS_MCA_SpeFile::WriteAdc, &GBS_MCA_SpeFile::CondTrue));
  blocks.insert(blocks.end(), SpeBlock(BlockPresets, &GBS_MCA_SpeFile::WritePresets, &GBS_MCA_SpeFile::CondMcaMode));
  blocks.insert(blocks.end(), SpeBlock(BlockPzc, &GBS_MCA_SpeFile::WritePzc, &GBS_MCA_SpeFile::CondTrue));
  blocks.insert(blocks.end(), SpeBlock(BlockFastDiscr, &GBS_MCA_SpeFile::WriteFastDiscr, &GBS_MCA_SpeFile::CondTrue));
  blocks.insert(blocks.end(), SpeBlock(BlockSlowDiscr, &GBS_MCA_SpeFile::WriteSlowDiscr, &GBS_MCA_SpeFile::CondTrue));
  blocks.insert(blocks.end(), SpeBlock(BlockThr, &GBS_MCA_SpeFile::WriteThr, &GBS_MCA_SpeFile::CondTrue));
  blocks.insert(blocks.end(), SpeBlock(BlockGain, &GBS_MCA_SpeFile::WriteGain, &GBS_MCA_SpeFile::CondTrue));
  blocks.insert(blocks.end(), SpeBlock(BlockTriggerF, &GBS_MCA_SpeFile::WriteTriggerF, &GBS_MCA_SpeFile::CondMca527));
  blocks.insert(blocks.end(), SpeBlock(BlockFlatTop, &GBS_MCA_SpeFile::WriteFlatTop, &GBS_MCA_SpeFile::CondMca527));
  blocks.insert(blocks.end(), SpeBlock(BlockDtc, &GBS_MCA_SpeFile::WriteDtc, &GBS_MCA_SpeFile::CondTrue));
  blocks.insert(blocks.end(), SpeBlock(BlockInput, &GBS_MCA_SpeFile::WriteInput, &GBS_MCA_SpeFile::CondTrue));
  blocks.insert(blocks.end(), SpeBlock(BlockPur, &GBS_MCA_SpeFile::WritePur, &GBS_MCA_SpeFile::CondTrue));
  blocks.insert(blocks.end(), SpeBlock(BlockStab, &GBS_MCA_SpeFile::WriteStab, &GBS_MCA_SpeFile::CondTrue));
  blocks.insert(blocks.end(), SpeBlock(BlockStabParam, &GBS_MCA_SpeFile::WriteStabParam, &GBS_MCA_SpeFile::CondTrue));
  blocks.insert(blocks.end(), SpeBlock(BlockPower, &GBS_MCA_SpeFile::WritePower, &GBS_MCA_SpeFile::CondTrue));
  blocks.insert(blocks.end(), SpeBlock(BlockHv, &GBS_MCA_SpeFile::WriteHv, &GBS_MCA_SpeFile::CondTrue));
  blocks.insert(blocks.end(), SpeBlock(BlockMode, &GBS_MCA_SpeFile::WriteMode, &GBS_MCA_SpeFile::CondTrue));
  blocks.insert(blocks.end(),
                SpeBlock(BlockMcaRepeat, &GBS_MCA_SpeFile::WriteMcaRepeat, &GBS_MCA_SpeFile::CondMcaMode));
  blocks.insert(blocks.end(), SpeBlock(BlockTdf, &GBS_MCA_SpeFile::WriteTdf, &GBS_MCA_SpeFile::CondTrue));
  blocks.insert(blocks.end(), SpeBlock(BlockPowerStat, &GBS_MCA_SpeFile::WritePowerStat, &GBS_MCA_SpeFile::CondTrue));
  blocks.insert(blocks.end(), SpeBlock(BlockCounts, &GBS_MCA_SpeFile::WriteCounts, &GBS_MCA_SpeFile::CondTrue));
  blocks.insert(blocks.end(), SpeBlock(BlockPdCounts, &GBS_MCA_SpeFile::WritePdCounts, &GBS_MCA_SpeFile::CondMca166));
  blocks.insert(blocks.end(), SpeBlock(BlockRt, &GBS_MCA_SpeFile::WriteRt, &GBS_MCA_SpeFile::CondTrue));
  blocks.insert(blocks.end(), SpeBlock(BlockDt, &GBS_MCA_SpeFile::WriteDt, &GBS_MCA_SpeFile::CondTrue));
  blocks.insert(blocks.end(), SpeBlock(BlockBt, &GBS_MCA_SpeFile::WriteBt, &GBS_MCA_SpeFile::CondTrue));
  blocks.insert(blocks.end(), SpeBlock(BlockRecCnt, &GBS_MCA_SpeFile::WriteRecCnt, &GBS_MCA_SpeFile::CondTrue));
  blocks.insert(blocks.end(), SpeBlock(BlockRecError, &GBS_MCA_SpeFile::WriteRecError, &GBS_MCA_SpeFile::CondTrue));
  blocks.insert(blocks.end(), SpeBlock(BlockIntegral, &GBS_MCA_SpeFile::WriteIntegral, &GBS_MCA_SpeFile::CondTrue));
  blocks.insert(blocks.end(), SpeBlock(BlockTemp, &GBS_MCA_SpeFile::WriteTemp, &GBS_MCA_SpeFile::CondMca527));
}

bool SpeFile::GBS_MCA_SpeFile::WriteSpectrum(std::string FileName,
                                             McaData::GBS_MCA_Data* data)  //-----------------------
{
  if (data == 0)
    return false;
  SpeData = data;
  bool rc = false;

  SpectrumStream = fopen(FileName.c_str(), "w");
  if (SpectrumStream != NULL)
  {
    SpectrumFileName = FileName;
    for (auto it = begin(blocks); it != end(blocks); ++it)
    {
      if ((this->*(*it).blockCondition)())
      {
        fprintf(SpectrumStream, "%s\n", (*it).blockId.c_str());
        (this->*(*it).writeMethod)();
      }
    }
    rc = true;
  }
  fclose(SpectrumStream);
  return rc;
}

// write methods =========================================================================================

void SpeFile::GBS_MCA_SpeFile::WriteApplId()  // Application Id Block -------------------------------------
{
  fprintf(SpectrumStream, "%s %s\n", SpeData->ApplId.c_str(), SpeData->ApplName.c_str());
}

void SpeFile::GBS_MCA_SpeFile::WriteDevId()  //-------------------------------------------------------------
{
  switch (SpeData->DeviceType)
  {
    case MCATYPE_MCA166:
      fprintf(SpectrumStream, "MCA-166\n");
      break;
    case MCATYPE_MCA527_FULL:
      fprintf(SpectrumStream, "MCA-527\n");
      break;
    case MCATYPE_MCA527_LITE:
      fprintf(SpectrumStream, "MCA-527 lite\n");
      break;
    case MCATYPE_MCA527_OEM:
      fprintf(SpectrumStream, "MCA-527 OEM\n");
      break;
    case MCATYPE_NONE:
    default:
      break;
  }
  fprintf(SpectrumStream, "SN# %d\n", SpeData->SerialNr);
  fprintf(SpectrumStream, "HW# %04X\n", SpeData->HwVersion);  // hex
  fprintf(SpectrumStream, "FW# %04X\n", SpeData->FwVersion);  // hex
}

void SpeFile::GBS_MCA_SpeFile::WriteDateMea()  // Start date and time of measurement -------------------
{
  if (SpeData->DateMea.tm_year > 0)
    fprintf(SpectrumStream, "%02d/%02d/%4d %02d:%02d:%02d\n", SpeData->DateMea.tm_mon + 1, SpeData->DateMea.tm_mday,
            SpeData->DateMea.tm_year + 1900, SpeData->DateMea.tm_hour, SpeData->DateMea.tm_min,
            SpeData->DateMea.tm_sec);
}

void SpeFile::GBS_MCA_SpeFile::WriteMeasTime()  // Measurement time in seconds
{
  fprintf(SpectrumStream, "%d %.0f\n", SpeData->LifeTime, round(SpeData->RealTime));
}

void SpeFile::GBS_MCA_SpeFile::WriteData()  // Spectral data ---------------------------------------------
{
  fprintf(SpectrumStream, "%d %d\n", SpeData->FirstChannel, SpeData->LastChannel);
  for (int i = 0; i <= (int)(SpeData->LastChannel - SpeData->FirstChannel); i++)
  {
    if (SpeData->Spectrum.size() > i)
      fprintf(SpectrumStream, "%8d\n", SpeData->Spectrum[i]);
    else
      break;
  }
}

void SpeFile::GBS_MCA_SpeFile::WriteRt()  // Real time [s] -----------------------------------------------
{
  fprintf(SpectrumStream, "%g\n", SpeData->RealTime);
}

void SpeFile::GBS_MCA_SpeFile::WriteDt()  // Dead time [ms] -----------------------------------------------
{
  fprintf(SpectrumStream, "%d\n", SpeData->DeadTime);
}

void SpeFile::GBS_MCA_SpeFile::WriteIntegral()  // Counts in the spectrum -------------------------------
{
  fprintf(SpectrumStream, "%lu\n", SpeData->Integral);
}

void SpeFile::GBS_MCA_SpeFile::WriteHv()  // High voltage -----------------------------------------------
{
  fprintf(SpectrumStream, "%c%dV\n", (SpeData->HvPolarity == POLARITY_POSITIVE ? '+' : '-'), SpeData->HighVoltage);

  switch (SpeData->HvInhibitMode)
  {
    case INHIBIT_OFF:
      fprintf(SpectrumStream, "unused\n");
      break;
    case INHIBIT_CANBERRA:
      fprintf(SpectrumStream, "Canberra\n");
      break;
    case INHIBIT_DSG:
      fprintf(SpectrumStream, "DSG\n");
      break;
    case INHIBIT_ORTEC:
      fprintf(SpectrumStream, "Ortec\n");
      break;
  }
}

void SpeFile::GBS_MCA_SpeFile::WritePresets()  // MCA Presets ------------------------------------------
{
  switch (SpeData->McaPresets)
  {
    case PRESET_NONE:
      fprintf(SpectrumStream, "None\n");
      break;
    case PRESET_REAL:
      fprintf(SpectrumStream, "Real Time (sec)\n");
      break;
    case PRESET_LIVE:
      fprintf(SpectrumStream, "Live Time (sec)\n");
      break;
    case PRESET_INT:
      fprintf(SpectrumStream, "Integral\n");
      break;
    case PRESET_AREA:
      fprintf(SpectrumStream, "Area\n");
      break;
  }
  fprintf(SpectrumStream, "%d\n", SpeData->PresetValue);
  fprintf(SpectrumStream, "%d\n", SpeData->PresetRoi);
}

void SpeFile::GBS_MCA_SpeFile::WriteMode()  // MCA Por MCS mode ----------------------------------------
{
  if (SpeData->Mode == MODE_MCS)
    fprintf(SpectrumStream, "MCS\n");
  else
    fprintf(SpectrumStream, "MCA\n");
}

void SpeFile::GBS_MCA_SpeFile::WriteAdc()  // ADC resolution --------------------------------------------
{
  fprintf(SpectrumStream, "%d\n", SpeData->Channels);
  fprintf(SpectrumStream, "%d\n", SpeData->Lld);
  fprintf(SpectrumStream, "%d\n", SpeData->Uld);
}

void SpeFile::GBS_MCA_SpeFile::WriteThr()  // Threshold in % ------------------------------------------
{
  fprintf(SpectrumStream, "%.1f\n", SpeData->Threshold);
}

void SpeFile::GBS_MCA_SpeFile::WriteGain()  // Amplifier gain -----------------------------------------
{
  fprintf(SpectrumStream, "%d\n", SpeData->CoarseGain);
  fprintf(SpectrumStream, "%.4f\n", SpeData->FineGain / 1000.0);
}

void SpeFile::GBS_MCA_SpeFile::WriteTriggerF()  // Amplifier gain -----------------------------------------
{
  fprintf(SpectrumStream, "%d (%s)\n", SpeData->TriggerFilter,
          McaComm::GetTriggerFilterDesc(SpeData->TriggerFilter).c_str());
}

void SpeFile::GBS_MCA_SpeFile::WriteFlatTop()  // Flattop time -----------------------------------------
{
  fprintf(SpectrumStream, "%.1f\n", ((float)SpeData->FlatTop) * 0.1);
}

void SpeFile::GBS_MCA_SpeFile::WriteInput()  // ADC input and polarity ----------------------------------
{
  switch (SpeData->AdcInput)
  {
    case ADC_AMPLIFIER:
      fprintf(SpectrumStream, "Amplifier\n");
      break;
    case ADC_P3_DIRECT:  // obsolete
      fprintf(SpectrumStream, "Direkt +3V\n");
      break;
    case ADC_N3_DIRECT:  // obsolete
      fprintf(SpectrumStream, "Direkt -3V\n");
      break;
    case ADC_DIRECT:
      fprintf(SpectrumStream, "Direkt\n");
      break;
  }
  if (SpeData->AmplifierPolarity == POLARITY_POSITIVE)
    fprintf(SpectrumStream, "pos\n");
  else
    fprintf(SpectrumStream, "neg\n");
}

void SpeFile::GBS_MCA_SpeFile::WritePowerStat()  // Power state at the end of the measurement ------------
{
  fprintf(SpectrumStream, "I+12= %dmA\n", SpeData->Iplus12);
  fprintf(SpectrumStream, "I-12= %dmA\n", SpeData->Iminus12);
  fprintf(SpectrumStream, "I+24= %dmA\n", SpeData->Iplus24);
  fprintf(SpectrumStream, "I-24= %dmA\n", SpeData->Iminus24);
  fprintf(SpectrumStream, "IBAT= %dmA\n", SpeData->Ibattery);
  fprintf(SpectrumStream, "IHV= %dmA\n", SpeData->Ihv);
  fprintf(SpectrumStream, "ICHR= %dmA\n", SpeData->Icharger);
  fprintf(SpectrumStream, "UBAT= %dmV\n", SpeData->Ubattery);
  fprintf(SpectrumStream, "UHVs= %dV\n", SpeData->Uhvs);

  if (CondMca527())
  {
    fprintf(SpectrumStream, "U+12= %.3fV\n", SpeData->Uplus12);
    fprintf(SpectrumStream, "U-12= %.3fV\n", SpeData->Uminus12);
    fprintf(SpectrumStream, "U+24= %.3fV\n", SpeData->Uplus24);
    fprintf(SpectrumStream, "U-24= %.3fV\n", SpeData->Uminus24);
  }
}

void SpeFile::GBS_MCA_SpeFile::WriteTemp()  // Temperature ----------------------------------------------
{
  if (SpeData->DetectorTemperature == TemperatureNotAvailable)
    fprintf(SpectrumStream, "not measured\n");
  else
    fprintf(SpectrumStream, "%.2f\n", SpeData->DetectorTemperature);
  if (SpeData->McaTemperature == TemperatureNotAvailable)
    fprintf(SpectrumStream, "not measured\n");
  else
    fprintf(SpectrumStream, "%.2f\n", SpeData->McaTemperature);
}

void SpeFile::GBS_MCA_SpeFile::WritePzc()  // PZC Settings ----------------------------------------------
{
  fprintf(SpectrumStream, "%d\n", SpeData->PzcValue);
  fprintf(SpectrumStream, "%d\n", SpeData->PzcTimeParaLow);
  fprintf(SpectrumStream, "%d\n", SpeData->PzcTimeParaHigh);
}

void SpeFile::GBS_MCA_SpeFile::WriteDtc()  // Shaping time ----------------------------------------------
{
  fprintf(SpectrumStream, "%d\n", SpeData->Dtc);
  fprintf(SpectrumStream, "%.1f\n", SpeData->ActShapingTime);
}

void SpeFile::GBS_MCA_SpeFile::WriteCounts()  // Integral counts ----------------------------------------
{
  fprintf(SpectrumStream, "%ld\n", SpeData->Counts);
}

void SpeFile::GBS_MCA_SpeFile::WritePdCounts()  // Integral peak detector counts ------------------------
{
  fprintf(SpectrumStream, "%ld\n", SpeData->PdCounts);
}

void SpeFile::GBS_MCA_SpeFile::WriteBt()  // Busy time --------------------------------------------------
{
  fprintf(SpectrumStream, "%d\n", SpeData->BusyTime);
}

void SpeFile::GBS_MCA_SpeFile::WriteFastDiscr()  // fast discriminator level
{
  fprintf(SpectrumStream, "%d\n", SpeData->FastDiscr);
}

void SpeFile::GBS_MCA_SpeFile::WriteSlowDiscr()  // slow discriminator level
{
  fprintf(SpectrumStream, "%d\n", SpeData->SlowDiscr);
}

void SpeFile::GBS_MCA_SpeFile::WritePur()  // state of the pile up rejector
{
  if (SpeData->PurState == PUR_OFF)
    fprintf(SpectrumStream, "off\n");
  else
    fprintf(SpectrumStream, "on\n");
}

void SpeFile::GBS_MCA_SpeFile::WriteStab()  // Stabilization
{
  if (SpeData->StabOn)
    fprintf(SpectrumStream, "on\n");
  else
    fprintf(SpectrumStream, "off\n");
  fprintf(SpectrumStream, "%d\n", SpeData->StabRoiBegin);
  fprintf(SpectrumStream, "%d\n", SpeData->StabRoiEnd);
  fprintf(SpectrumStream, "%d\n", SpeData->StabChannel);
}

void SpeFile::GBS_MCA_SpeFile::WriteStabParam()  // Stabilization parameter
{
  fprintf(SpectrumStream, "%d\n", SpeData->StabTime);
  fprintf(SpectrumStream, "%d\n", SpeData->StabArea);
}

void SpeFile::GBS_MCA_SpeFile::WriteSpecRem()  // Comment and filename
{
  fprintf(SpectrumStream, "%s\n\n", SpeData->SpecRem.c_str());
  fprintf(SpectrumStream, "%s\n", SpectrumFileName.c_str());
}

void SpeFile::GBS_MCA_SpeFile::WritePower()  // Preamplifier power supply
{
  fprintf(SpectrumStream, "+12= %s\n", ((SpeData->PowerSwitches & 0x10) > 0) ? "on" : "off");
  fprintf(SpectrumStream, "-12= %s\n", ((SpeData->PowerSwitches & 0x20) > 0) ? "on" : "off");
  fprintf(SpectrumStream, "+24= %s\n", ((SpeData->PowerSwitches & 0x40) > 0) ? "on" : "off");
  fprintf(SpectrumStream, "-24= %s\n", ((SpeData->PowerSwitches & 0x80) > 0) ? "on" : "off");
}

void SpeFile::GBS_MCA_SpeFile::WriteMcaRepeat()  // Number of sweeps
{
  fprintf(SpectrumStream, "%d\n", SpeData->McaRepeat);
  fprintf(SpectrumStream, "0\n");  // MCA Repeat mode type - no repeat mode
}

void SpeFile::GBS_MCA_SpeFile::WriteTdf()  // Dead time correction factor (ns)
{
  fprintf(SpectrumStream, "%d\n", SpeData->Tdf);
}

void SpeFile::GBS_MCA_SpeFile::WriteRecCnt()  // Counter of received commands
{
  fprintf(SpectrumStream, "%d\n", SpeData->RecCounter);
}

void SpeFile::GBS_MCA_SpeFile::WriteRecError()  // Counter of received commands with error
{
  fprintf(SpectrumStream, "%d\n", SpeData->RecErrorCnt);
}

// write conditions ====================================================================================

bool SpeFile::GBS_MCA_SpeFile::CondTrue()  // Block is written in every case ----------------------------
{
  return true;
}

bool SpeFile::GBS_MCA_SpeFile::CondSpectrumData()  // true, if valid spectrum available -----------------
{
  if (SpeData->FirstChannel >= SpeData->LastChannel)
    return false;
  if (SpeData->Spectrum.size() <= (int)(SpeData->LastChannel - SpeData->FirstChannel))
    return false;

  return true;
}

bool SpeFile::GBS_MCA_SpeFile::CondMcaMode()  // true, if in MCA mde -------------------------------------
{
  return (SpeData->Mode == MODE_MCA);
}

bool SpeFile::GBS_MCA_SpeFile::CondMca527()  // MCA-527? -------------------------------------------------
{
  switch (SpeData->DeviceType)
  {
    case MCATYPE_MCA527_FULL:
    case MCATYPE_MCA527_LITE:
    case MCATYPE_MCA527_OEM:
    case MCATYPE_MCA527_MICRO:
    case MCATYPE_MCA527_NANO:
      return true;
      break;

    case MCATYPE_MCA166:
    case MCATYPE_NONE:
    default:
      return false;
      break;
  }
}

bool SpeFile::GBS_MCA_SpeFile::CondMca166()  // MCA-166? -------------------------------------------------
{
  return (SpeData->DeviceType == MCATYPE_MCA166);
}
