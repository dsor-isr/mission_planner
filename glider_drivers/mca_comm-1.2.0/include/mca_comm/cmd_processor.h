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

#ifndef CMD_PROCESSOR_H
#define CMD_PROCESSOR_H

#include <mca_comm/gbs_mca_comm.h>
#include <mca_comm/gbs_mca_data.h>
#include <mca_comm/gbs_mca_spefile.h>

#include <limits.h>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <sstream>

using namespace std;

// Constants -------------------------------------------------------------------------------

const string SpeDefaultName = "mca_spectrum.spe";

// Generic error which provides a message
struct MsgError  //--------------------------------------------------------------------------
{
  string msg;
  MsgError(string m = string())
  {
    msg = m;
  }
};

class Cmd_Processor  //----------------------------------------------------------------------
{
protected:
  struct McaComm::GBS_MCA_Comm* mca;
  struct McaComm::QUERY_STATE* mcaState;
  struct McaComm::QUERY_STATE527* mcaState527;
  struct McaComm::QUERY_STATE527_EX* mcaState527_ex;
  struct McaComm::QUERY_POWER* mcaPower;
  struct McaComm::QUERY_SYSTEM_DATA* mcaSystem;
  McaData::GBS_MCA_Data* mcaData;

  uint16_t MaxHv;
  uint16_t MaxChannels;
  uint16_t MaxUld;
  uint16_t MaxFlattop;  // * 0.1 μs
  bool verbose;

private:
  vector<string> split(string input);
  void DoSave(string filename, string comment);
  void DoSetHv(uint16_t hv, int32_t inhibit);
  void DoSetPreset(std::string preset, uint32_t time);
  void DoSetAdc(uint16_t channels, uint16_t lld, uint16_t uld);
  void DoSetThreshold(float threshold);
  void DoSetGain(uint16_t coarseGain, uint16_t fineGain);
  void DoSetAmpPolarity(bool positive);
  void DoSetPzc(uint16_t pzc_value);
  void DoAutoPzc();
  void DoSetShapingTime(float shaping_time);
  void DoSetTriggerFilter(uint16_t index);
  void DoSetFlattopTime(float ft_time);
  void DoReset();
  void DoQueryState();
  void DoQueryPower();

public:
  Cmd_Processor();
  ~Cmd_Processor();

  bool init(int argc, char* argv[]);
  bool parse(string line);
};

#endif  // CMD_PROCESSOR_H
