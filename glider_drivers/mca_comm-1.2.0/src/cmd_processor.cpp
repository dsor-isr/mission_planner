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

#include <mca_comm/cmd_processor.h>

using namespace std;

Cmd_Processor::Cmd_Processor()
//....................................................................................................
{
  verbose = true;
  mca = new McaComm::GBS_MCA_Comm();
  mcaState = new McaComm::QUERY_STATE;
  mcaPower = new McaComm::QUERY_POWER;
  mcaSystem = new McaComm::QUERY_SYSTEM_DATA;

  // will be allocated later, if necessary
  mcaState527 = 0;
  mcaState527_ex = 0;
  mcaData = 0;
}

Cmd_Processor::~Cmd_Processor()  //...................................................................
{
  delete mcaState;
  delete mcaPower;
  delete mcaSystem;
  if (mcaState527 != 0)
    delete mcaState527;
  if (mcaState527_ex != 0)
    delete mcaState527_ex;
  delete mca;
}

// Return: exit program?
bool Cmd_Processor::init(int argc,
                         char* argv[])  //..........................................................................
{
  // check command line parameters
  long timeout = 1000;  // defaults for safety
  int attempts = 8;
  ulong baudRate = 0;

  try
  {
    long tmp;
    bool ok;
    if (argc > 5)
      throw MsgError();
    if (argc > 1)
    {  // timout
      tmp = atol(argv[1]);
      if (tmp < 10L)
        throw MsgError("[timeout(ms)]: specify timeout in ms, >= 10");
      else
        timeout = tmp;
    }
    if (argc > 2)
    {  // attempts
      tmp = atol(argv[2]);
      if (tmp < 1 || tmp > INT_MAX)
        throw MsgError("[attempts]: number of communication attempts, > 0, must not exceed integer range");
      else
        attempts = (int)tmp;
    }
    if (argc > 3)
    {  // baud rate
      bool valid = false;
      tmp = atol(argv[3]);
      string ErrMsg = "[baud rate]: 0 (highest possible baud rate is used)";
      if (tmp == 0)
        valid = true;
      else
      {
        for (uint i = 0; i < (sizeof(McaComm::PossibleBaudrates) / sizeof(McaComm::PossibleBaudrates[0])); i++)
        {
          ErrMsg += ", " + to_string(McaComm::PossibleBaudrates[i]);
          if ((ulong)tmp == McaComm::PossibleBaudrates[i])
            valid = true;
        }
      }
      if (!valid)
        throw MsgError(ErrMsg);
      else
        baudRate = tmp;
    }
    if (argc > 4)
    {
      if (strcmp(argv[4], "dialog") == 0)
        verbose = true;
      else if (strcmp(argv[4], "no_dialog") == 0)
        verbose = false;
      else
        throw MsgError("5th parameter must be \"dialog\" or \"no_dialog\"");
    }

    cout << "Connecting to MCA..." << endl;
    bool success;
    switch (argc)
    {
      case 1:
        success = mca->COMM_INIT();
        break;
      case 2:
        success = mca->COMM_INIT(timeout);
        break;
      case 3:
        success = mca->COMM_INIT(timeout, attempts);
        break;
      case 4:
      case 5:
        success = mca->COMM_INIT(timeout, attempts, baudRate);
        break;
      default:  // should not appear
        throw MsgError();
    }
    if (!success)
    {
      cout << "Connection to MCA failed." << endl;
      return true;
    }
    else
    {
      McaComm::E_ERROR_FLAG rc = McaComm::ERROR_OK;
      rc = mca->MMCA_QUERY_STATE(mcaState);
      if (rc != McaComm::ERROR_OK)
        throw MsgError("Error at QUERY_STATE: " + to_string(rc));
      printf("Connected with MCA %d, baud rate: %ld\n", mcaState->McaNumber, mca->GetBaudrate());
      if (verbose)
        cout << "> " << flush;
      if (mcaState->FirmwareVersion == 0xFFFF)  // MCA-527
      {
        mcaState527 = new McaComm::QUERY_STATE527();
        rc = mca->MMCA_QUERY_STATE527(mcaState527);
        if (rc != McaComm::ERROR_OK)
          throw MsgError("Error at QUERY_STATE527: " + to_string(rc));
        mcaState527_ex = new McaComm::QUERY_STATE527_EX();
        rc = mca->MMCA_QUERY_STATE527_EX(mcaState527_ex);
        if (rc != McaComm::ERROR_OK)
          throw MsgError("Error at QUERY_STATE527_EX: " + to_string(rc));
      }
      // Get device-specific parameters
      MaxHv = McaData::GetMaxHv(mcaState527);
      MaxChannels = McaData::GetMaxChannels(mcaState527);
      MaxFlattop = McaData::GetMaxFlattop(mcaState527, mcaState527_ex);
    }
    return false;
  }
  catch (MsgError err)
  {
    cout << "usage: GBS_MCA_Cmd [timeout(ms)] [attempts] [baud rate] [dialog|no_dialog]" << endl;
    if (!err.msg.empty())
      cout << err.msg << endl;
    return true;
  }
}

// Return: exit program?
bool Cmd_Processor::parse(string line)  //...........................................................................
{
  vector<string> params = split(line);
  bool ok = false;

  // evaluate command
  if (params.size() > 0)
  {
    if (params[0].compare("quit") == 0)
    {
      return true;
    }
    else if (params[0].compare("help") == 0)
    {
      cout << "Available commands: " << endl;
      cout << "help\t- show this help text" << endl;
      cout << "quit\t- terminate the program" << endl;
      cout << "reset\t- reset all MCA parameters to their initial state" << endl;
      cout << "query_state\t- show state information on the MCA" << endl;
      cout << "query_power\t- show power information on the MCA" << endl;
      cout << "save [<filename>] [<comment>]\t- save spectrum to file, default: " << SpeDefaultName << endl;
      cout << "set_adc <channels> [<lld> <uld>]\t- set ADC resolution" << endl;
      cout << "set_amp_polarity (+|-)\t- set ADC input to shaping and amplifier polarity." << endl;
      cout << "set_gain <coarse_gain> <fine_gain>\t- set amplifier gain" << endl;
      cout << "set_hv <hv> [<inhibit>]\t- set high voltage" << endl;
      cout << "set_preset ( none | real | live ) [<time>]\t- set automatic stop condition" << endl;
      cout << "set_threshold <threshold>\t- set analog threshold" << endl;
      cout << "set_pzc <pzc_value>\t- set pole zero cancellation and return offset" << endl;
      cout << "set_pzc auto\t- evaluate PZC automatically by offset minimization" << endl;
      cout << "set_shaping_time <shaping_time>\t - set shaping time in µs" << endl;
      cout << "set_trigger_filter <index>\t - set trigger filter" << endl;
      cout << "set_flattop_time <time>\t - set flattop time" << endl;
      cout << "start\t- start an acquisition" << endl;
      cout << "stop\t- stop an acquisition" << endl;
    }
    else if (params[0].compare("start") == 0)
    {
      time_t now;
      time(&now);
      McaComm::E_ERROR_FLAG rc = mca->MMCA_START_ACQUIRE(1, McaComm::ToMcaTime(now));
      if (rc != McaComm::ERROR_OK)
        cout << "Starting acquisition failed: rc = " << rc << endl;
      else
        cout << "Acquisition started" << endl;
    }
    else if (params[0].compare("stop") == 0)
    {
      McaComm::E_ERROR_FLAG rc = mca->MMCA_STOP_ACQUIRE();
      if (rc != McaComm::ERROR_OK)
        cout << "Stopping acquisition failed: rc = " << rc << endl;
      else
        cout << "Acquisition stopped" << endl;
    }
    else if (params[0].compare("save") == 0)
    {
      string filename = SpeDefaultName;
      string comment = "";
      if (params.size() > 1)
        filename = params[1];
      if (params.size() > 2)
        comment = params[2];
      DoSave(filename, comment);
    }
    else if (params[0].compare("set_hv") == 0)
    {
      try
      {
        int32_t inhibit = 0;  // default: off
        uint16_t hv = 0;

        switch (params.size())
        {
          case 3:
            try
            {
              inhibit = stoi(params[2]);
            }
            catch (const std::exception& e)
            {
              throw MsgError("usage: set_hv <hv> [inhibit]");
            }
          case 2:
            try
            {
              hv = stoi(params[1]);
            }
            catch (const std::exception& e)
            {
              throw MsgError("usage: set_hv <hv> [inhibit]");
            }
            DoSetHv(hv, inhibit);
            break;
          default:
            throw MsgError("usage: set_hv <hv> [inhibit]");
        }
      }
      catch (MsgError err)
      {
        cout << err.msg << endl;
      }
    }
    else if (params[0].compare("set_preset") == 0)
    {
      try
      {
        uint32_t time = 0;
        switch (params.size())
        {
          case 3:
            try
            {
              if (params[1].compare("none") != 0)
                time = stol(params[2]);
            }
            catch (const std::exception& e)
            {
              throw MsgError("Time (in s) must be specified as second parameter");
            }
            break;
          case 2:
            if (params[1].compare("none") != 0)
              throw MsgError("Time (in s) must be specified as second parameter");
            break;
          default:
            throw MsgError("usage: set_preset (none | real | live) [time]");
            break;
        }
        DoSetPreset(params[1], time);
      }
      catch (MsgError err)
      {
        cout << err.msg << endl;
      }
    }
    else if (params[0].compare("reset") == 0)
    {
      DoReset();
    }
    else if (params[0].compare("set_adc") == 0)
    {
      try
      {
        uint16_t channels;
        uint16_t lld;
        uint16_t uld;

        if ((params.size() != 4) && (params.size() != 2))
          throw MsgError("");
        try
        {
          channels = stoi(params[1]);
        }
        catch (const std::exception& e)
        {
          throw MsgError("");
        }
        if (params.size() == 2)
        {
          lld = 0;
          uld = McaData::GetMaxUld(channels, mcaState527);
        }
        else
        {
          try
          {
            lld = stoi(params[2]);
            uld = stoi(params[3]);
          }
          catch (const std::exception& e)
          {
            throw MsgError("");
          }
        }
        DoSetAdc(channels, lld, uld);
      }
      catch (MsgError)
      {
        cout << "usage: set_adc <channels> [<lld> <uld>]" << endl;
      }
    }
    else if (params[0].compare("set_threshold") == 0)
    {
      try
      {
        float threshold;

        if (params.size() != 2)
          throw MsgError("usage: set_threshold <threshold>");
        try
        {
          threshold = stof(params[1]);
        }
        catch (const std::exception& e)
        {
          throw MsgError("Threshold (in %) must be specified as parameter");
        }
        DoSetThreshold(threshold);
      }
      catch (MsgError err)
      {
        cout << err.msg << endl;
      }
    }
    else if (params[0].compare("set_gain") == 0)
    {
      try
      {
        uint16_t coarseGain;
        uint16_t fineGain;
        string errMsg;

        if (params.size() != 3)
          throw MsgError("usage: set_gain <coarse_gain> <fine_gain>");
        try
        {
          errMsg = "coarse gain must be specified as first parameter";
          coarseGain = stoi(params[1]);
          errMsg = "fine gain must be specified as second parameter";
          fineGain = stoi(params[2]);
        }
        catch (const std::exception& e)
        {
          throw MsgError(errMsg);
        }
        DoSetGain(coarseGain, fineGain);
      }
      catch (MsgError err)
      {
        cout << err.msg << endl;
      }
    }
    else if (params[0].compare("set_amp_polarity") == 0)
    {
      try
      {
        bool positive = true;
        if (params.size() != 2)
          throw MsgError("usage: set_amp_polarity (+|-)");
        if (params[1].compare("+") == 0)
          positive = true;
        else if (params[1].compare("-") == 0)
          positive = false;
        else
          throw MsgError("polarity (+ or -) must be specified as parameter");
        DoSetAmpPolarity(positive);
      }
      catch (MsgError err)
      {
        cout << err.msg << endl;
      }
    }
    else if (params[0].compare("set_pzc") == 0)
    {
      try
      {
        uint16_t pzc_value;

        if (params.size() != 2)
          throw MsgError("usage: set_pzc <pzc_value>");
        if (params[1].compare("auto") == 0)
        {
          DoAutoPzc();
        }
        else
        {
          try
          {
            pzc_value = stoi(params[1]);
          }
          catch (const std::exception& e)
          {
            throw MsgError("pzc value must be specified as first parameter");
          }
          DoSetPzc(pzc_value);
        }
      }
      catch (MsgError err)
      {
        cout << err.msg << endl;
      }
    }
    else if (params[0].compare("set_shaping_time") == 0)
    {
      try
      {
        float shaping_time;
        if (params.size() != 2)
          throw MsgError("usage: set_shaping_time <shaping_time>");
        try
        {
          shaping_time = stof(params[1]);
        }
        catch (const std::exception& e)
        {
          throw MsgError("shaping time must be specified as first parameter");
        }
        DoSetShapingTime(shaping_time);
      }
      catch (MsgError err)
      {
        cout << err.msg << endl;
      }
    }
    else if (params[0].compare("set_trigger_filter") == 0)
    {
      try
      {
        uint16_t index;
        if (params.size() != 2)
          throw MsgError("usage: set_trigger_filter <index>");
        try
        {
          index = stoi(params[1]);
        }
        catch (const std::exception& e)
        {
          throw MsgError("the trigger filter index must be specified as first parameter");
        }
        int MaxIndex = (sizeof(McaComm::TriggerFilterDesc) / sizeof(McaComm::TriggerFilterDesc[0])) - 1;
        if (index > MaxIndex)
          throw MsgError("the trigger filter index must be in the range 0.." + to_string(MaxIndex));
        DoSetTriggerFilter(index);
      }
      catch (MsgError err)
      {
        cout << err.msg << endl;
      }
    }
    else if (params[0].compare("set_flattop_time") == 0)  // set_flattop_time <time>
    {
      try
      {
        float ft_time;
        if (params.size() != 2)
          throw MsgError("usage: set_flattop_time <time_in_μs>");
        try
        {
          ft_time = stof(params[1]);
        }
        catch (const std::exception& e)
        {
          throw MsgError("the flattop time must be specified as parameter");
        }
        DoSetFlattopTime(ft_time);
      }
      catch (MsgError err)
      {
        cout << err.msg << endl;
      }
    }
    else if (params[0].compare("query_state") == 0)  // sshow state information on the MCA
    {
      try
      {
        DoQueryState();
      }
      catch (MsgError err)
      {
        cout << err.msg << endl;
      }
    }
    else if (params[0].compare("query_power") == 0)  // show power information on the MCA
    {
      try
      {
        DoQueryPower();
      }
      catch (MsgError err)
      {
        cout << err.msg << endl;
      }
    }
    else
    {
      cout << "unknown Command " << params[0] << endl;
    }
  }
  if (verbose)
    cout << "> " << flush;
  return false;
}

// private functions --------------------------------------------------------------------------------------

vector<string> Cmd_Processor::split(string input)  //.....................................................
{
  vector<string> result;
  string::size_type position, start = 0;

  while (string::npos != (position = input.find(" ", start)))
  {
    if (input[start] == '"')
    {
      start++;
      position = input.find("\"", start);
      if (position == string::npos)
      {
        break;
      }
    }
    result.push_back(input.substr(start, position - start));
    start = position + 1;
  }

  result.push_back(input.substr(start));
  return result;
}

void Cmd_Processor::DoSave(string filename, string comment)  //...................................
{
  McaComm::E_ERROR_FLAG rc = McaComm::ERROR_OK;
  try
  {
    rc = mca->MMCA_QUERY_STATE(mcaState);
    if (rc != McaComm::ERROR_OK)
      throw MsgError("Error at QUERY_STATE: " + to_string(rc));
    if ((mcaState->FirmwareVersion == 0xFFFF)  // MCA-527
        && (mcaState527 != 0))
    {
      rc = mca->MMCA_QUERY_STATE527(mcaState527);
      if (rc != McaComm::ERROR_OK)
        throw MsgError("Error at QUERY_STATE527: " + to_string(rc));
    }
    rc = mca->MMCA_QUERY_POWER(mcaPower);
    if (rc != McaComm::ERROR_OK)
      throw MsgError("Error at QUERY_POWER: " + to_string(rc));
    rc = mca->MMCA_QUERY_SYSTEM_DATA(mcaSystem);
    if (rc != McaComm::ERROR_OK)
      throw MsgError("Error at QUERY_SYSTEM_DATA: " + to_string(rc));

    if (mcaData == 0)
    {
      mcaData = new McaData::GBS_MCA_Data();
      mcaData->Spectrum.resize(mcaState->Channels);
    }
    else
      mcaData->Clear();

    rc = mca->MMCA_QUERY_COMPLETE_SPECTRUM(0, mcaState->Channels - 1, 1, mcaData->Spectrum.data());
    if (rc != McaComm::ERROR_OK)
      throw MsgError("Error at QUERY_COMPLETE_SPECTRUM: " + to_string(rc));

    mcaData->FillFromMca(mcaState, mcaState527, mcaPower, mcaSystem);
    mcaData->SpecRem = comment;
    SpeFile::GBS_MCA_SpeFile speOutput;
    if (speOutput.WriteSpectrum(filename, mcaData) == false)
      throw MsgError("Error at writing spe file " + to_string(rc));

    cout << "Measurement saved to " << filename << endl;
  }
  catch (MsgError err)
  {
    cout << err.msg << endl;
  }
}

void Cmd_Processor::DoQueryState()  //...................................................................
{
  McaComm::E_ERROR_FLAG rc = mca->MMCA_QUERY_STATE(mcaState);
  if (rc != McaComm::ERROR_OK)
    throw MsgError("Error at QUERY_STATE: " + to_string(rc));

  cout << "ADC: " << mcaState->Channels << "   LLD: " << mcaState->Lld << "   ULD: " << mcaState->Uld << endl;
  cout << "Presets: ";
  switch ((E_McaPresets)mcaState->Presets)
  {
    case PRESET_NONE:
      cout << "None" << endl;
      break;
    case PRESET_REAL:
      cout << "Real Time " << mcaState->PresetValue << "s" << endl;
      break;
    case PRESET_LIVE:
      cout << "Live Time " << mcaState->PresetValue << "s" << endl;
      break;
    case PRESET_INT:
      cout << "Integral" << endl;
      break;
    case PRESET_AREA:
      cout << "Area" << endl;
      break;
  }
  cout << "HV: " << ((E_Polarity)mcaState->DetectorBiasPoly == POLARITY_POSITIVE ? '+' : '-') << mcaState->DetectorBias
       << "V";
  cout << "   HV inhibit: ";
  switch ((E_InhibitMode)mcaState->HvInhibitMode)
  {
    case INHIBIT_OFF:
      cout << "unused" << endl;
      break;
    case INHIBIT_CANBERRA:
      cout << "Canberra" << endl;
      break;
    case INHIBIT_DSG:
      cout << "DSG" << endl;
      break;
    case INHIBIT_ORTEC:
      cout << "Ortec" << endl;
      break;
  }
  cout << "Coarse gain: " << mcaState->CoarseGain << "   Fine gain: " << setprecision(4) << fixed
       << mcaState->FineGain / 1000.0;
  cout << "   Amp.Polarity: " << ((E_Polarity)mcaState->McaInputPol == POLARITY_POSITIVE ? "pos" : "neg") << endl;
  cout << setprecision(1);
  cout << "PZC value: " << mcaState->PzcValue << "   PUR: " << ((E_McaPur)mcaState->McaPur == PUR_OFF ? "off" : "on")
       << endl;

  if ((mcaState->FirmwareVersion == 0xFFFF)  // MCA-527
      && (mcaState527 != 0))
  {
    rc = mca->MMCA_QUERY_STATE527(mcaState527);
    if (rc != McaComm::ERROR_OK)
      throw MsgError("Error at QUERY_STATE527: " + to_string(rc));

    cout << "Flattop time: " << mcaState527->FlatTopTime * 0.1f;
    uint16_t triggerFilter = (mcaState->Dtc == 1 ? mcaState527->TriggerFilterLow : mcaState527->TriggerFilterHigh);
    cout << "   Trigger filter: " << triggerFilter << "  (" << McaComm::GetTriggerFilterDesc(triggerFilter) << ")"
         << endl;
    cout << "Threshold: " << mcaState527->ThresholdTenths * 0.1f << endl;
  }
  else  // MCA-166
  {
    cout << "Threshold: " << mcaState->Threshold << endl;
  }
}

void Cmd_Processor::DoQueryPower()
{
  McaComm::E_ERROR_FLAG rc = mca->MMCA_QUERY_POWER(mcaPower);
  if (rc != McaComm::ERROR_OK)
    throw MsgError("Error at QUERY_POWER: " + to_string(rc));

  cout << "High Voltage: " << std::round(mcaPower->HighVoltage * 1.2) << "   State: " << mcaPower->HvState << endl;
  cout << "High Voltage at stop: " << std::round(mcaPower->HighVoltageAtStop * 1.2) << endl;
}

void Cmd_Processor::DoSetHv(uint16_t hv, int32_t inhibit)  //.............................................
{
  McaComm::E_ERROR_FLAG rc = McaComm::ERROR_OK;
  try
  {
    if (hv > MaxHv)
      throw MsgError("HV must not be greater than " + to_string(MaxHv));

    if ((inhibit != (int32_t)INHIBIT_OFF) && (inhibit != (int32_t)INHIBIT_CANBERRA) &&
        (inhibit != (int32_t)INHIBIT_DSG) && (inhibit != (int32_t)INHIBIT_ORTEC))
      throw MsgError("Inhibit must be " + to_string(INHIBIT_OFF) + ", " + to_string(INHIBIT_CANBERRA) + ", " +
                     to_string(INHIBIT_DSG) + " or " + to_string(INHIBIT_ORTEC));

    rc = mca->MMCA_SET_HIGH_VOLTAGES(hv, inhibit);
    if (rc != McaComm::ERROR_OK)
      throw MsgError("Error at SET_HIGH_VOLTAGES: " + to_string(rc));
    cout << "High voltage set to " << hv << endl;
  }
  catch (MsgError err)
  {
    cout << err.msg << endl;
  }
}

void Cmd_Processor::DoSetPreset(std::string preset, uint32_t time)  //.........................................
{
  McaComm::E_ERROR_FLAG rc = McaComm::ERROR_OK;
  try
  {
    if (preset.compare("none") == 0)
    {
      rc = mca->MMCA_SET_PRESET_NONE();
      if (rc != McaComm::ERROR_OK)
        throw MsgError("Error at SET_PRESET_NONE: " + to_string(rc));
    }
    else if (preset.compare("real") == 0)
    {
      rc = mca->MMCA_SET_PRESET_REAL_TIME(time);
      if (rc != McaComm::ERROR_OK)
        throw MsgError("Error at SET_PRESET_REAL_TIME: " + to_string(rc));
    }
    else if (preset.compare("live") == 0)
    {
      rc = mca->MMCA_SET_PRESET_LIVE_TIME(time);
      if (rc != McaComm::ERROR_OK)
        throw MsgError("Error at SET_PRESET_LIVE_TIME: " + to_string(rc));
    }
    else
      throw MsgError("first parameter must be none, real or live");

    cout << "set preset " << preset;
    if (preset.compare("none") != 0)
      cout << " to " << time << "s";
    cout << endl;
  }
  catch (MsgError err)
  {
    cout << err.msg << endl;
  }
}

void Cmd_Processor::DoReset()  //..........................................................................
{
  McaComm::E_ERROR_FLAG rc = mca->MMCA_RESET();
  if (rc != McaComm::ERROR_OK)
    cout << "Reset failed: rc = " << rc << endl;
  else
  {
    if (mcaData != 0)
    {
      rc = mca->MMCA_QUERY_STATE(mcaState);
      if (rc != McaComm::ERROR_OK)
        cout << "Error at QUERY_STATE: " << rc << endl;
      else
        mcaData->Spectrum.resize(mcaState->Channels);
    }
    cout << "MCA reset" << endl;
  }
}

void Cmd_Processor::DoSetAdc(uint16_t channels, uint16_t lld, uint16_t uld)  //...............................
{
  McaComm::E_ERROR_FLAG rc = McaComm::ERROR_OK;
  try
  {
    bool ok = false;
    for (uint16_t i = 128; i <= 16284; i <<= 1)
    {
      if (i == channels)
        ok = true;
    }
    if (!ok)
      throw MsgError("first parameter (channels) must be 128, 256 ... 16384");
    if (channels > MaxChannels)
      throw MsgError("Channels must not be greater than " + to_string(MaxChannels));
    if (lld >= uld)
      throw MsgError("lld must be less than uld");
    uint16_t maxUld = McaData::GetMaxUld(channels, mcaState527);
    if (uld > maxUld)
      throw MsgError("uld must not be greater than " + to_string(maxUld));
    rc = mca->MMCA_SET_ADC_RES_DISCR(channels, lld, uld);
    if (rc != McaComm::ERROR_OK)
      throw MsgError("Error at SET_ADC_RES_DISCR: " + to_string(rc));
    if (mcaData != 0)
      mcaData->Spectrum.resize(channels);
    cout << "ADC set to " << channels << endl;
  }
  catch (MsgError err)
  {
    cout << err.msg << endl;
  }
}

void Cmd_Processor::DoSetThreshold(float threshold)  //...................................................
{
  McaComm::E_ERROR_FLAG rc = McaComm::ERROR_OK;
  float thresholdSet;
  try
  {
    if (threshold > 60.0)
      throw MsgError("threshold must not be greater than 60");
    if (threshold < 0.0)
      throw MsgError("threshold must not be negative");
    if (mcaState527 == 0)  // MCA 166
    {
      thresholdSet = round(threshold);
      rc = mca->MMCA_SET_THRESHOLD((uint16_t)thresholdSet);
      if (rc != McaComm::ERROR_OK)
        throw MsgError("Error at SET_THRESHOLD: " + to_string(rc));
    }
    else
    {
      thresholdSet = round(threshold * 10);
      thresholdSet /= 10;
      rc = mca->MMCA_SET_THRESHOLD_TENTHS((uint16_t)round(threshold * 10));
      if (rc != McaComm::ERROR_OK)
        throw MsgError("Error at SET_THRESHOLD_TENTHS: " + to_string(rc));
    }
    cout << "Threshold set to " << thresholdSet << endl;
  }
  catch (MsgError err)
  {
    cout << err.msg << endl;
  }
}

void Cmd_Processor::DoSetGain(uint16_t coarseGain, uint16_t fineGain)  //...................................
{
  McaComm::E_ERROR_FLAG rc = McaComm::ERROR_OK;
  try
  {
    if ((coarseGain != 2) && (coarseGain != 5) && (coarseGain != 10) && (coarseGain != 20) && (coarseGain != 50) &&
        (coarseGain != 100) && (coarseGain != 200) && (coarseGain != 500) && (coarseGain != 1000))
      throw MsgError("coarse gain must be 2, 5, 10, 20, 50, 100, 200, 500 or 1000");
    if (fineGain < 5000)
      throw MsgError("fine gain must be >= 5000");
    uint16_t max = McaData::GetMaxFineGain(mcaState527);
    if (fineGain > max)
      throw MsgError("fine gain must be <= " + to_string(max));
    if ((mcaState527 == 0) /* MCA-166 */ && (coarseGain * fineGain > 10000000))
      throw MsgError("coarse gain * fine gain must be <= 10000000");
    rc = mca->MMCA_SET_GAIN(coarseGain, fineGain);
    if (rc != McaComm::ERROR_OK)
      throw MsgError("Error at SET_GAIN: " + to_string(rc));
    cout << "Gain set to " << coarseGain << ", " << fineGain << endl;
  }
  catch (MsgError err)
  {
    cout << err.msg << endl;
  }
}

void Cmd_Processor::DoSetAmpPolarity(bool positive)  //..................................................
{
  McaComm::E_ERROR_FLAG rc;
  if (positive)
    rc = mca->MMCA_SET_MCA_INPUT_AMPLIFIER_POS();
  else
    rc = mca->MMCA_SET_MCA_INPUT_AMPLIFIER_NEG();

  if (rc != McaComm::ERROR_OK)
    cout << "Error at SET_MCA_INPUT_AMPLIFIER: " << to_string(rc) << endl;
  else
    cout << "ADC input set to shaping, amplifier polarity set to " << (positive ? "positive" : "negative") << endl;
}

void Cmd_Processor::DoSetPzc(uint16_t pzc_value)  //.....................................................
{
  McaComm::E_ERROR_FLAG rc;
  int16_t offset;
  uint16_t measPulses;
  try
  {
    if (pzc_value > 2499)
      throw MsgError("PZC value must be < 2500");
    rc = mca->MMCA_SET_PZC_MANUAL(pzc_value, &offset, &measPulses, /*HighPrecision = */ false);
    if (rc != McaComm::ERROR_OK)
      throw MsgError("Error at SET_PZC_MANUAL: " + to_string(rc));
    else
    {
      cout << "PZC value set to " << pzc_value;
      if (measPulses > 0)
        cout << ", offset = " << offset;
    }
    cout << endl;
  }
  catch (MsgError err)
  {
    cout << err.msg << endl;
  }
}

void Cmd_Processor::DoAutoPzc()  //.................................................................
{
  const int MaxCnt = 10;  // max. number of attempts
  int cnt = MaxCnt;
  int maxOffset = 5;  // maximum offset to accept
  McaComm::E_ERROR_FLAG rc;
  bool highPrecision;
  int16_t pzc_value;
  int16_t offset;
  uint16_t measPulses;

  cout << "Evaluating pole zero cancellation" << setprecision(2) << fixed << endl;
  rc = mca->MMCA_QUERY_STATE(mcaState);
  if (rc != McaComm::ERROR_OK)
    throw MsgError("Error at QUERY_STATE: " + to_string(rc));
  pzc_value = mcaState->PzcValue;                                            // current value
  highPrecision = ((mcaState527 != 0) && (mcaState527->FwVersion >= 1205));  // Since firmware version 12.05.
  if (!highPrecision)
    maxOffset = 0;

  rc = mca->MMCA_SET_PZC_MANUAL(pzc_value, &offset, &measPulses, highPrecision);
  if (rc != McaComm::ERROR_OK)
    throw MsgError("Error at SET_PZC_MANUAL: " + to_string(rc));
  if (measPulses == 0)
    pzc_value = 2499;
  do
  {
    pzc_value = (pzc_value < 0 ? 0 : (pzc_value > 2499 ? 2499 : pzc_value));
    rc = mca->MMCA_SET_PZC_MANUAL(pzc_value, &offset, &measPulses, highPrecision);
    if (rc != McaComm::ERROR_OK)
      throw MsgError("Error at SET_PZC_MANUAL: " + to_string(rc));
    if (measPulses > 0)
    {
      if ((pzc_value != SHRT_MIN) && (pzc_value != SHRT_MAX))
      {
        cout << "PZC value: " << pzc_value << "  Offset: " << (highPrecision ? ((float)offset) / 32 : offset) << endl;
        if (abs(offset) < maxOffset && cnt < MaxCnt)
          break;
        pzc_value -= offset;
      }
      cnt--;
    }
    else
    {
      cout << ". " << flush;
      if (pzc_value <= 0)
        break;
      pzc_value -= 100;
      cnt = MaxCnt;
    }
  } while (cnt > 0);
  if (measPulses == 0)
    cout << endl;
}

void Cmd_Processor::DoSetShapingTime(float shaping_time)  //...........................................
{
  McaComm::E_ERROR_FLAG rc;
  float LowShapingTime, HighShapingTime;

  try
  {
    if (shaping_time < 0.1)
      throw MsgError("Shaping time must be >= 0.1");

    rc = mca->MMCA_QUERY_SYSTEM_DATA(mcaSystem);
    if (rc != McaComm::ERROR_OK)
      throw MsgError("Error at QUERY_SYSTEM_DATA: " + to_string(rc));

    LowShapingTime = mcaSystem->LowShapingTime * 0.1f;
    if (mcaSystem->HighShapingTime < mcaSystem->LowShapingTime)
      HighShapingTime = (mcaSystem->HighShapingTime + 256) * 0.1f;
    else
      HighShapingTime = mcaSystem->HighShapingTime * 0.1f;

    if (shaping_time == LowShapingTime)
    {
      rc = mca->MMCA_SET_SHAPING_TIME_LOW();
      if (rc != McaComm::ERROR_OK)
        throw MsgError("Error at SET_SHAPING_TIME_LOW: " + to_string(rc));
    }
    else if (shaping_time == HighShapingTime)
    {
      rc = mca->MMCA_SET_SHAPING_TIME_HIGH();
      if (rc != McaComm::ERROR_OK)
        throw MsgError("Error at SET_SHAPING_TIME_HIGH: " + to_string(rc));
    }
    else
    {
      rc = mca->MMCA_QUERY_STATE(mcaState);
      if (rc != McaComm::ERROR_OK)
        throw MsgError("Error at QUERY_STATE: " + to_string(rc));

      if (!McaData::IsMca527(mcaState))  // MCA 166
        throw MsgError("For this MCA only shaping time " + to_string(LowShapingTime) + " or " +
                       to_string(HighShapingTime) + " is possible");

      if (shaping_time > (mcaState527->MaxShapingTime * 0.1))
        throw MsgError("The maximum shaping time for this MCA is " + to_string(mcaState527->MaxShapingTime * 0.1));

      uint16_t value;
      value = (uint16_t)round(shaping_time * 10);
      if (value > (mcaState527->MaxShapingTime >> 1))  // :2
      {
        rc = mca->MMCA_SET_SHAPING_TIME_PAIR(1, value);
        if (rc != McaComm::ERROR_OK)
          throw MsgError("Error at SET_SHAPING_TIME_PAIR: " + to_string(rc));
        rc = mca->MMCA_SET_SHAPING_TIME_HIGH();
        if (rc != McaComm::ERROR_OK)
          throw MsgError("Error at SET_SHAPING_TIME_HIGH: " + to_string(rc));
      }
      else
      {
        rc = mca->MMCA_SET_SHAPING_TIME_PAIR(value, value + 1);
        if (rc != McaComm::ERROR_OK)
          throw MsgError("Error at SET_SHAPING_TIME_PAIR: " + to_string(rc));
        rc = mca->MMCA_SET_SHAPING_TIME_LOW();
        if (rc != McaComm::ERROR_OK)
          throw MsgError("Error at SET_SHAPING_TIME_LOW: " + to_string(rc));
      }
    }
    cout << "Shaping time set to " << shaping_time << endl;
  }
  catch (MsgError err)
  {
    cout << err.msg << endl;
  }
}

void Cmd_Processor::DoSetTriggerFilter(uint16_t index)  //...........................................
{
  McaComm::E_ERROR_FLAG rc;
  try
  {
    rc = mca->MMCA_SET_TRIGGER_FILTER(index, index);
    if (rc != McaComm::ERROR_OK)
      throw MsgError("Error at SET_TRIGGER_FILTER: " + to_string(rc));
    else
      cout << "Trigger filter set to " << index << " (" << McaComm::GetTriggerFilterDesc(index) << ")" << endl;
  }
  catch (MsgError err)
  {
    cout << err.msg << endl;
  }
}

void Cmd_Processor::DoSetFlattopTime(float ft_time)  //.............................................
{
  McaComm::E_ERROR_FLAG rc;
  try
  {
    uint16_t ftTimeInt = (uint16_t)round(ft_time * 10);
    if (ftTimeInt < 0 || ftTimeInt > MaxFlattop)
    {
      char tmpBuf[50];
      sprintf(tmpBuf, "Flattop time must be in the range 0 .. %.1f", ((float)MaxFlattop) * 0.1);
      throw MsgError(tmpBuf);
    }
    rc = mca->MMCA_SET_FLAT_TOP_TIME(ftTimeInt);
    if (rc != McaComm::ERROR_OK)
      throw MsgError("Error at MMCA_SET_FLAT_TOP_TIME: " + to_string(rc));
    else
      cout << "Flattop time set to " << ft_time << endl;
  }
  catch (MsgError err)
  {
    cout << err.msg << endl;
  }
}
