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

#ifndef GBS_MCA_TYPES_H
#define GBS_MCA_TYPES_H

const int MinChannels = 128;    // minimum spectrum size
const int MaxChannels = 16384;  // maximum spectrum size
const unsigned short TemperatureNotAvailable = 0x8000;

// Hardware Type of the MCA -------------------------------------------------------
enum E_McaTyp
{
  MCATYPE_NONE,
  MCATYPE_MCA166,
  MCATYPE_MCA527_FULL,
  MCATYPE_MCA527_LITE,
  MCATYPE_MCA527_OEM,
  MCATYPE_MCA527_MICRO,
  MCATYPE_MCA527_NANO
};

enum E_McaPresets  //---------------------------------------------------------------
{
  PRESET_NONE = 0,
  PRESET_REAL = 1,
  PRESET_LIVE = 2,
  PRESET_INT = 3,
  PRESET_AREA = 4
};

enum E_InputADC  //------------------------------------------------------------------
{
  ADC_AMPLIFIER = 0,
  ADC_P3_DIRECT = 3,  // obsolete
  ADC_N3_DIRECT = 4,  // obsolete
  ADC_DIRECT = 5
};

enum E_Polarity  //-------------------------------------------------------------------
{
  POLARITY_POSITIVE = 0,
  POLARITY_NEGATIVE = 1
};

enum E_McaPur  //--------------------------------------------------------------------
{
  PUR_ON = 1,
  PUR_OFF = 0
};

enum E_InhibitMode  //----------------------------------------------------------------
{
  INHIBIT_OFF = 0,       // inhibit off
  INHIBIT_CANBERRA = 1,  // Canberra HPGe mode, HV shut down if inhibit input < 0.5V
  INHIBIT_DSG = 2,       // DSG HPGe mode, HV shut down if inhibit input < 0.5V
  INHIBIT_ORTEC = -1     // Ortec HPGe mode, HV shut down if inhibit input >= 5V
};

enum E_McsInput  //--------------------------------------------------------------------
{
  MCS_INPUT_TTL = 0,
  MCS_INPUT_RATE = 1,
  MCS_INPUT_DISCR = 2
};

enum E_McaMode  //-------------------------------------------------------------------
{
  MODE_MCA = 0,
  MODE_MCS = 1
};

enum E_McaState  //--------------------------------------------------------------------
{
  STATE_READY = 1,
  STATE_RUN = 2,
  STATE_SUSPEND = 3,
  STATE_FINISH = 4,
  STATE_STOP = 5,
  STATE_FAIL = 6,
  STATE_WAIT_FOR_TRIGGER = 7
};

#endif  // GBS_MCA_TYPES_H
