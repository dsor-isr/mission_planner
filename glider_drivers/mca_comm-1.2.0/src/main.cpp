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
#include <mca_comm/gbs_mca_comm.h>

using namespace std;

int main(int argc, char* argv[])
{

  bool exitProgram = false;
  string line;
  Cmd_Processor* cmdProc = new Cmd_Processor();
  exitProgram = cmdProc->init(argc, argv);

  while (!exitProgram)
  {
    getline(cin, line);
    exitProgram = cmdProc->parse(line);
  }
  delete cmdProc;
  return 0;
}
