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

#include <chrono>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

using namespace std::chrono_literals;

double queryHighVoltage(McaComm::GBS_MCA_Comm& mca)
{
  McaComm::QUERY_POWER mca_power;
  const auto error_flag = mca.MMCA_QUERY_POWER(&mca_power);
  if (error_flag != McaComm::ERROR_OK)
  {
    throw std::runtime_error("Failed to query MCA power state. Error: " + std::to_string(error_flag));
  }

  return 1.2 * mca_power.HighVoltage;
}

void setHighVoltage(McaComm::GBS_MCA_Comm& mca, const uint16_t high_voltage)
{
  const auto error_flag = mca.MMCA_SET_HIGH_VOLTAGES(high_voltage, INHIBIT_OFF);
  if (error_flag != McaComm::ERROR_OK)
  {
    throw std::runtime_error("Failed to set high voltage to " + std::to_string(high_voltage) +
                             "V. Error: " + std::to_string(error_flag));
  }
}

/**
 * MCA shutdown program
 *
 * This program connects to the MCA and if the current high voltage is not zero, it sets it to zero and waits for it to
 * reach zero.
 *
 * This program can be used in a shutdown script to make the system wait for the MCA high voltage to reach zero before
 * shutting down.
 */
int main(int argc, char* argv[])
{
  // Check out how many devices are available
  McaComm::GBS_MCA_Comm mca;
  const auto num_devices = mca.GetNumDevices();
  if (num_devices == -1)
  {
    std::cerr << "Failed to get the number of devices." << std::endl;

    return EXIT_FAILURE;
  }
  else if (num_devices == 0)
  {
    std::cout << "No device available." << std::endl;

    return EXIT_SUCCESS;
  }

  // MCA initialization configuration
  const int timeout_ms{ 1000 };
  const int attempts{ 8 };
  const int baud_rate{ 0 };

  // Connect to MCA device
  const auto success = mca.COMM_INIT(timeout_ms, attempts, baud_rate);
  if (!success)
  {
    std::cerr << "Failed to initialize MCA." << std::endl;

    return EXIT_FAILURE;
  }

  try
  {
    // Query initial high voltage
    const auto initial_high_voltage = queryHighVoltage(mca);

    // Skip if the initial high voltage is already 0V
    if (initial_high_voltage == 0.0)
    {
      std::cout << "High voltage was already zero." << std::endl;

      mca.COMM_CLOSE();

      return EXIT_SUCCESS;
    }

    std::cout << "Initial high voltage: " << initial_high_voltage << "V." << std::endl;

    // Set high voltage to 0V
    setHighVoltage(mca, 0);

    std::cout << "High voltage set to zero." << std::endl;

    // Wait for high voltage to reach 0V
    const auto start = std::chrono::steady_clock::now();
    while (true)
    {
      // Query high voltage
      const auto high_voltage = queryHighVoltage(mca);
      std::cout << "High voltage [V]: " << std::setfill(' ') << std::setw(7) << std::fixed << std::setprecision(2)
                << high_voltage << std::endl;

      // Finish if the high voltage has reached 0V
      const std::chrono::duration<double> elapsed_time = std::chrono::steady_clock::now() - start;

      if (high_voltage == 0)
      {
        std::cout << "High voltage successfully zeroed down from " << initial_high_voltage << "V in "
                  << elapsed_time.count() << "s." << std::endl;
        break;
      }

      // Abort if timed out
      //
      // It takes 281.062s to zero down an initial high voltage of 1545.6V. That is the high voltage read when the high
      // voltage is set to 1500V. That is almost 5min.
      if (elapsed_time > 6min)
      {
        std::cerr << "Timed out after waiting " << elapsed_time.count() << "s for the initial high voltage of "
                  << initial_high_voltage << "V to reach zero because it is still " << high_voltage << "V."
                  << std::endl;
      }

      // Sleep before querying the high voltage again
      std::this_thread::sleep_for(10s);
    }
  }
  catch (const std::runtime_error& ex)
  {
    std::cerr << ex.what() << std::endl;
  }

  mca.COMM_CLOSE();

  return EXIT_SUCCESS;
}
