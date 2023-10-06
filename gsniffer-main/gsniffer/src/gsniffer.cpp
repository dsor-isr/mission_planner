/**
 * Copyright (c) 2023 Ploatech https://ploatech.com
 *
 * This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program. If not, see
 * <https://www.gnu.org/licenses/>.
 */

#include <ros/ros.h>

#include <mca_comm/gbs_mca_comm.h>
#include <mca_comm/gbs_mca_data.h>
#include <mca_comm/gbs_mca_spefile.h>

#include <gsniffer_msgs/Calibration.h>
#include <gsniffer_msgs/Power.h>
#include <gsniffer_msgs/Spectrum.h>
#include <gsniffer_msgs/State.h>
#include <gsniffer_msgs/SystemData.h>

#include <boost/asio/ip/host_name.hpp>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <iterator>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

template <typename T>
std::ostream& operator<<(std::ostream& os, const std::vector<T>& v)
{
  os << '[';

  if (!v.empty())
  {
    std::copy(v.cbegin(), v.cend() - 1, std::ostream_iterator<T>(os, ", "));
    os << v.back();
  }

  os << ']';
  return os;
}

/**
 * @brief GSniffer node
 *
 * Publishers:
 * - ~/calibration : Latch publisher with the calibration coefficients
 * - ~/spectrum : Publisher with the spectrum
 * - ~/power : Lazy publisher with the power data
 * - ~/state : Lazy publisher with the state data
 * - ~/system_data : Lazy publisher with the system data
 */
class GSnifferNode
{
public:
  /**
   * @brief Constructor
   *
   * Loads the parameters, advertises the publishers, start the timers and initialize the MCA.
   *
   * If something fails during the MCA initialization, the node is shutdown.
   */
  GSnifferNode()
  {
    // Initialization parameters
    private_node_handle_.param("timeout", timeout_, timeout_);
    private_node_handle_.param("attempts", attempts_, attempts_);
    private_node_handle_.param("baud_rate", baud_rate_, baud_rate_);

    // Acquisition parameters
    private_node_handle_.param("sample_period", sample_period_, sample_period_);
    private_node_handle_.param("acquisition_mode", acquisition_mode_, acquisition_mode_);
    private_node_handle_.param("acquisition_time", acquisition_time_, acquisition_time_);
    private_node_handle_.param("stop_acquisition_before_sampling", stop_acquisition_before_sampling_,
                               stop_acquisition_before_sampling_);
    private_node_handle_.param("high_voltage", high_voltage_, high_voltage_);
    private_node_handle_.param("first_channel", first_channel_, first_channel_);
    private_node_handle_.param("last_channel", last_channel_, last_channel_);
    private_node_handle_.param("channels", channels_, channels_);
    private_node_handle_.param("lld", lld_, lld_);
    private_node_handle_.param("uld", uld_, uld_);
    private_node_handle_.param("coarse_gain", coarse_gain_, coarse_gain_);
    private_node_handle_.param("fine_gain", fine_gain_, fine_gain_);
    private_node_handle_.param("shaping_time", shaping_time_, shaping_time_);
    private_node_handle_.param("save_folder", save_folder_, save_folder_);

    if (acquisition_time_ < 0)
    {
      ROS_FATAL_STREAM("Acquisition time " << acquisition_time_ << "s is negative.");

      ros::shutdown();
      return;
    }

    if (first_channel_ < 0)
    {
      ROS_FATAL_STREAM("First channel " << first_channel_ << " is negative.");

      ros::shutdown();
      return;
    }
    else if (last_channel_ >= 0 && first_channel_ > last_channel_)
    {
      ROS_FATAL_STREAM("First channel " << first_channel_ << " is greater than the last channel " << last_channel_
                                        << ".");

      ros::shutdown();
      return;
    }

    createSaveFolder();

    // Power parameters
    private_node_handle_.param("power_publish_period", power_publish_period_, power_publish_period_);

    // State parameters
    private_node_handle_.param("state_publish_period", state_publish_period_, state_publish_period_);

    // System data parameters
    private_node_handle_.param("system_data_publish_period", system_data_publish_period_, system_data_publish_period_);

    // Calibration parameters
    private_node_handle_.param("calibration_slope", calibration_slope_, calibration_slope_);
    private_node_handle_.param("calibration_offset", calibration_offset_, calibration_offset_);
    private_node_handle_.param("calibration_square", calibration_square_, calibration_square_);

    // Initialize publishers and timers
    calibration_publisher_ = private_node_handle_.advertise<gsniffer_msgs::Calibration>("calibration", 1, true);

    spectrum_publisher_ = private_node_handle_.advertise<gsniffer_msgs::Spectrum>("spectrum", 10);
    sample_timer_ = private_node_handle_.createTimer(ros::Duration(sample_period_), &GSnifferNode::sample, this);

    power_publisher_ = private_node_handle_.advertise<gsniffer_msgs::Power>("power", 10);
    power_publish_timer_ =
        private_node_handle_.createTimer(ros::Duration(power_publish_period_), &GSnifferNode::publishPower, this);

    state_publisher_ = private_node_handle_.advertise<gsniffer_msgs::State>("state", 10);
    state_publish_timer_ =
        private_node_handle_.createTimer(ros::Duration(state_publish_period_), &GSnifferNode::publishState, this);

    system_data_publisher_ = private_node_handle_.advertise<gsniffer_msgs::SystemData>("system_data", 10);
    system_data_publish_timer_ = private_node_handle_.createTimer(ros::Duration(system_data_publish_period_),
                                                                  &GSnifferNode::publishSystemData, this);

    // Create MCA and MCA state
    mca_ = std::make_shared<McaComm::GBS_MCA_Comm>();
    mca_state_ = new McaComm::QUERY_STATE;

    // Connect to MCA device
    if (mca_->IsConnected())
    {
      ROS_WARN("MCA was already initialized.");
    }
    else
    {
      const auto timeout_ms = static_cast<int>(std::ceil(1000 * timeout_));

      const auto success = mca_->COMM_INIT(timeout_ms, attempts_, baud_rate_);
      if (!success)
      {
        ROS_FATAL("Failed to initialize MCA.");

        ros::shutdown();
        return;
      }
    }

    // Set acquisition mode and time
    if (acquisition_mode_ == "none")
    {
      const auto error_flag = mca_->MMCA_SET_PRESET_NONE();
      if (error_flag != McaComm::ERROR_OK)
      {
        ROS_FATAL_STREAM("Failed to set acquisition mode to " << acquisition_mode_ << ". Error: " << error_flag);

        mca_->COMM_CLOSE();

        ros::shutdown();
        return;
      }

      ROS_INFO_STREAM("Acquisition mode set to " << acquisition_mode_ << ".");
    }
    else if (acquisition_mode_ == "live")
    {
      const auto error_flag = mca_->MMCA_SET_PRESET_LIVE_TIME(acquisition_time_);
      if (error_flag != McaComm::ERROR_OK)
      {
        ROS_FATAL_STREAM("Failed to set live acquisition time to " << acquisition_time_ << "s. Error: " << error_flag);

        mca_->COMM_CLOSE();

        ros::shutdown();
        return;
      }

      ROS_INFO_STREAM("Live acquisition time set to " << acquisition_time_ << "s.");
    }
    else if (acquisition_mode_ == "real")
    {
      const auto error_flag = mca_->MMCA_SET_PRESET_REAL_TIME(acquisition_time_);
      if (error_flag != McaComm::ERROR_OK)
      {
        ROS_FATAL_STREAM("Failed to set real acquisition time to " << acquisition_time_ << "s. Error: " << error_flag);

        mca_->COMM_CLOSE();

        ros::shutdown();
        return;
      }

      ROS_INFO_STREAM("Real acquisition time set to " << acquisition_time_ << "s.");
    }
    else
    {
      ROS_FATAL_STREAM("Unknown acquisition mode " << acquisition_mode_ << ".");

      mca_->COMM_CLOSE();

      ros::shutdown();
      return;
    }

    // Query MCA state
    auto error_flag = McaComm::ERROR_OK;
    error_flag = mca_->MMCA_QUERY_STATE(mca_state_);
    if (error_flag != McaComm::ERROR_OK)
    {
      ROS_FATAL_STREAM("Failed to query MCA state. Error: " << error_flag);

      mca_->COMM_CLOSE();

      ros::shutdown();
      return;
    }

    if (mca_state_->FirmwareVersion == 0xFFFF)  // MCA-527
    {
      mca_state527_ = new McaComm::QUERY_STATE527();
      error_flag = mca_->MMCA_QUERY_STATE527(mca_state527_);
      if (error_flag != McaComm::ERROR_OK)
      {
        ROS_FATAL_STREAM("Failed to query MCA-527 state. Error: " << error_flag);

        mca_->COMM_CLOSE();

        ros::shutdown();
        return;
      }

      mca_state527_ex_ = new McaComm::QUERY_STATE527_EX();
      error_flag = mca_->MMCA_QUERY_STATE527_EX(mca_state527_ex_);
      if (error_flag != McaComm::ERROR_OK)
      {
        ROS_FATAL_STREAM("Failed to query MCA-527 extended state. Error: " << error_flag);

        mca_->COMM_CLOSE();

        ros::shutdown();
        return;
      }
    }

    // Query system data
    McaComm::QUERY_SYSTEM_DATA mca_system_data;
    error_flag = mca_->MMCA_QUERY_SYSTEM_DATA(&mca_system_data);
    if (error_flag != McaComm::ERROR_OK)
    {
      ROS_ERROR_STREAM("Failed to query MCA system data. Error: " << error_flag);

      mca_->COMM_CLOSE();

      ros::shutdown();
      return;
    }

    // Retrieve the maximum values for some parameters
    max_high_voltage_ = McaData::GetMaxHv(mca_state527_);
    max_channels_ = McaData::GetMaxChannels(mca_state527_);
    max_uld_ = McaData::GetMaxUld(channels_, mca_state527_);
    max_fine_gain_ = McaData::GetMaxFineGain(mca_state527_);
    max_flat_top_ = McaData::GetMaxFlattop(mca_state527_, mca_state527_ex_);

    // Validate channels is a power of 2 in the [128, 16284] range
    if (!isChannelsValid(channels_))
    {
      ROS_FATAL_STREAM("Invalid number of channels " << channels_
                                                     << ". It must be a power of 2 in the [128, 16284] range.");

      mca_->COMM_CLOSE();

      ros::shutdown();
      return;
    }

    // Validate channels is not greater than the maximum channels supported by the MCA
    if (channels_ > max_channels_)
    {
      ROS_FATAL_STREAM("The number of channels "
                       << channels_ << " must not be greater than the maximum number of channels " << max_channels_
                       << " supported.");

      mca_->COMM_CLOSE();

      ros::shutdown();
      return;
    }

    // Validate lower channel is lower than upper channel
    if (lld_ >= uld_)
    {
      ROS_FATAL_STREAM("Lower channel " << lld_ << " must be lower than the upper channel " << uld_ << ".");

      mca_->COMM_CLOSE();

      ros::shutdown();
      return;
    }

    // Validate the upper channels is not greater than the maximum upper channels supported by the MCA
    if (uld_ > max_uld_)
    {
      ROS_FATAL_STREAM("The upper channels " << uld_ << " must not be greater than the maximum upper channel "
                                             << max_uld_ << " supported.");

      mca_->COMM_CLOSE();

      ros::shutdown();
      return;
    }

    // Validate the coarse gain is 2, 5, 10, 20, 50, 100, 200, 500 or 1000
    const std::vector<int> coarse_gains{ 2, 5, 10, 20, 50, 100, 200, 500, 1000 };
    if (std::none_of(coarse_gains.cbegin(), coarse_gains.cend(),
                     [this](const auto gain) { return coarse_gain_ == gain; }))
    {
      ROS_FATAL_STREAM("The coarse gain " << coarse_gain_ << " must be one of " << coarse_gains << ".");

      mca_->COMM_CLOSE();

      ros::shutdown();
      return;
    }

    // Validate fine gain is not lower than 5000
    if (fine_gain_ < 5000)
    {
      ROS_FATAL_STREAM("The fine gain " << fine_gain_ << " must not be lower than 5000.");

      mca_->COMM_CLOSE();

      ros::shutdown();
      return;
    }

    // Validate fine gain is not greater than the max fine gain supported by the MCA
    if (fine_gain_ > max_fine_gain_)
    {
      ROS_FATAL_STREAM("The fine gain " << fine_gain_ << " must not be greater than the maximum fine gain "
                                        << max_fine_gain_ << " supported.");

      mca_->COMM_CLOSE();

      ros::shutdown();
      return;
    }

    // Validate product of the coarse and fine gains is not greater than 10000000 for the MCA-166
    if (!mca_state527_ && coarse_gain_ * fine_gain_ > 10000000)
    {
      ROS_FATAL_STREAM("The product of the coarse gain " << coarse_gain_ << " and fine gain " << fine_gain_
                                                         << ", which is " << coarse_gain_ * fine_gain_
                                                         << ", must not be greater than 10000000.");

      mca_->COMM_CLOSE();

      ros::shutdown();
      return;
    }

    if (shaping_time_ > 0.0)
    {
      // Validate shaping time is not lower than 0.1
      if (shaping_time_ < 0.1)
      {
        ROS_FATAL_STREAM("The shaping time " << shaping_time_ << "us must not be lower than 0.1μs");

        mca_->COMM_CLOSE();

        ros::shutdown();
        return;
      }
    }

    // Set channels, lower and upper channel
    error_flag = mca_->MMCA_SET_ADC_RES_DISCR(channels_, lld_, uld_);
    if (error_flag != McaComm::ERROR_OK)
    {
      ROS_FATAL_STREAM("Failed to set channels to " << channels_ << ", lower channel to " << lld_
                                                    << " and upper channel " << uld_ << ". Error: " << error_flag);

      mca_->COMM_CLOSE();

      ros::shutdown();
      return;
    }

    mca_data_.Spectrum.resize(mca_state_->Channels);

    // Set last channel to the last channel available in the MCA
    if (last_channel_ < 0)
    {
      last_channel_ = mca_state_->Channels - 1;

      ROS_INFO_STREAM("Last channel set to " << last_channel_ << ", which is the last of all the "
                                             << mca_state_->Channels << " channels available.");
    }

    // Set coarse and fine gains
    error_flag = mca_->MMCA_SET_GAIN(coarse_gain_, fine_gain_);
    if (error_flag != McaComm::ERROR_OK)
    {
      ROS_FATAL_STREAM("Failed to set coarse gain to " << coarse_gain_ << " and fine gain to " << fine_gain_
                                                       << ". Error: " << error_flag);

      mca_->COMM_CLOSE();

      ros::shutdown();
      return;
    }

    // Set shaping time, unless it is negative
    if (shaping_time_ > 0.0)
    {
      // Retrieve low and high shaping times
      const auto low_shaping_time = 0.1 * mca_system_data.LowShapingTime;
      auto high_shaping_time = 0.1 * mca_system_data.HighShapingTime;

      if (high_shaping_time < low_shaping_time)
      {
        high_shaping_time += 256 * 0.1;
      }

      if (std::abs(shaping_time_ - low_shaping_time) < std::numeric_limits<double>::epsilon())
      {
        error_flag = mca_->MMCA_SET_SHAPING_TIME_LOW();
        if (error_flag != McaComm::ERROR_OK)
        {
          ROS_FATAL_STREAM("Failed to set low shaping time " << shaping_time_ << "us. Error: " << error_flag);

          mca_->COMM_CLOSE();

          ros::shutdown();
          return;
        }

        ROS_INFO_STREAM("Low shaping time set to " << shaping_time_ << "us.");
      }
      else if (std::abs(shaping_time_ - high_shaping_time) < std::numeric_limits<double>::epsilon())
      {
        error_flag = mca_->MMCA_SET_SHAPING_TIME_HIGH();
        if (error_flag != McaComm::ERROR_OK)
        {
          ROS_FATAL_STREAM("Failed to set high shaping time " << shaping_time_ << "us. Error: " << error_flag);

          mca_->COMM_CLOSE();

          ros::shutdown();
          return;
        }

        ROS_INFO_STREAM("High shaping time set to " << shaping_time_ << "us.");
      }
      else
      {
        if (!mca_state527_)  // MCA-166
        {
          ROS_FATAL_STREAM("The shaping time " << shaping_time_ << "us must be the low shaping time "
                                               << low_shaping_time << "us or the high shaping time "
                                               << high_shaping_time
                                               << "us, which are the only ones supported by MCA-166.");

          mca_->COMM_CLOSE();

          ros::shutdown();
          return;
        }

        const auto max_shaping_time = mca_state527_->MaxShapingTime;
        if (shaping_time_ > 0.1 * max_shaping_time)
        {
          ROS_FATAL_STREAM("The shaping time " << shaping_time_
                                               << "us must not be higher than the maximum shaping time "
                                               << 0.1 * max_shaping_time << "us.");

          mca_->COMM_CLOSE();

          ros::shutdown();
          return;
        }

        const auto value = static_cast<uint16_t>(std::round(shaping_time_ * 10));
        if (value > (max_shaping_time >> 1))
        {
          error_flag = mca_->MMCA_SET_SHAPING_TIME_PAIR(1, value);
          if (error_flag != McaComm::ERROR_OK)
          {
            ROS_FATAL_STREAM("Failed to set shaping time pair (1, " << value << ") for shaping time " << shaping_time_
                                                                    << "us. Error: " << error_flag);

            mca_->COMM_CLOSE();

            ros::shutdown();
            return;
          }

          error_flag = mca_->MMCA_SET_SHAPING_TIME_HIGH();
          if (error_flag != McaComm::ERROR_OK)
          {
            ROS_FATAL_STREAM("Failed to set high shaping time " << shaping_time_ << "us. Error: " << error_flag);

            mca_->COMM_CLOSE();

            ros::shutdown();
            return;
          }

          ROS_INFO_STREAM("High shaping time set to " << shaping_time_ << "us with pair (1, " << value << ").");
        }
        else
        {
          error_flag = mca_->MMCA_SET_SHAPING_TIME_PAIR(value, value + 1);
          if (error_flag != McaComm::ERROR_OK)
          {
            ROS_FATAL_STREAM("Failed to set shaping time pair (" << value << ", " << value + 1 << ") for shaping time "
                                                                 << shaping_time_ << "us. Error: " << error_flag);

            mca_->COMM_CLOSE();

            ros::shutdown();
            return;
          }

          error_flag = mca_->MMCA_SET_SHAPING_TIME_LOW();
          if (error_flag != McaComm::ERROR_OK)
          {
            ROS_FATAL_STREAM("Failed to set low shaping time " << shaping_time_ << "us. Error: " << error_flag);

            mca_->COMM_CLOSE();

            ros::shutdown();
            return;
          }

          ROS_INFO_STREAM("Low shaping time set to " << shaping_time_ << "us with pair (" << value << ", " << value + 1
                                                     << ").");
        }
      }
    }

    // Query the MCA state again, after setting the configuration
    error_flag = mca_->MMCA_QUERY_STATE(mca_state_);
    if (error_flag != McaComm::ERROR_OK)
    {
      ROS_FATAL_STREAM("Failed to query MCA state. Error: " << error_flag);

      mca_->COMM_CLOSE();

      ros::shutdown();
      return;
    }

    // Query the MCA state again, after setting the configuration
    error_flag = mca_->MMCA_QUERY_SYSTEM_DATA(&mca_system_data);
    if (error_flag != McaComm::ERROR_OK)
    {
      ROS_ERROR_STREAM("Failed to query MCA system data. Error: " << error_flag);

      mca_->COMM_CLOSE();

      ros::shutdown();
      return;
    }

    // Retrieve low and high shaping times
    const auto low_shaping_time = 0.1 * mca_system_data.LowShapingTime;
    auto high_shaping_time = 0.1 * mca_system_data.HighShapingTime;

    if (high_shaping_time < low_shaping_time)
    {
      high_shaping_time += 256 * 0.1;
    }

    // Print MCA state
    ROS_INFO_STREAM("ADC: " << mca_state_->Channels << ", LLD: " << mca_state_->Lld << ", ULD: " << mca_state_->Uld);
    ROS_INFO_STREAM("HV: " << (static_cast<E_Polarity>(mca_state_->DetectorBiasPoly) == POLARITY_POSITIVE ? '+' : '-')
                           << mca_state_->DetectorBias << "V");
    ROS_INFO_STREAM("Coarse gain: " << mca_state_->CoarseGain << ", Fine gain: " << mca_state_->FineGain);
    ROS_INFO_STREAM("Amplifier polarity: "
                    << (static_cast<E_Polarity>(mca_state_->McaInputPol) == POLARITY_POSITIVE ? "pos" : "neg"));
    ROS_INFO_STREAM("PZC: " << mca_state_->PzcValue
                            << ", PUR: " << (static_cast<E_McaPur>(mca_state_->McaPur) == PUR_OFF ? "off" : "on"));

    if (mca_state527_)
    {
      ROS_INFO_STREAM("Flat top time: " << 0.1 * mca_state527_->FlatTopTime << "us");
      const auto trigger_filter =
          mca_state_->Dtc == 1 ? mca_state527_->TriggerFilterLow : mca_state527_->TriggerFilterHigh;
      ROS_INFO_STREAM("Trigger filter: " << trigger_filter << " (" << McaComm::GetTriggerFilterDesc(trigger_filter)
                                         << ")");
      ROS_INFO_STREAM("Threshold: " << 0.1 * mca_state527_->ThresholdTenths);
    }
    else
    {
      ROS_INFO_STREAM("Threshold: " << mca_state_->Threshold);
    }

    ROS_INFO_STREAM("Max high voltage: " << static_cast<int>(max_high_voltage_));
    ROS_INFO_STREAM("Max channels: " << static_cast<int>(max_channels_));
    ROS_INFO_STREAM("Max upper channel: " << static_cast<int>(max_uld_));
    ROS_INFO_STREAM("Max fine gain: " << static_cast<int>(max_fine_gain_));
    ROS_INFO_STREAM("Max flat top time: " << 0.1 * max_flat_top_ << "us");

    ROS_INFO_STREAM("Low shaping time: " << low_shaping_time << "us");
    ROS_INFO_STREAM("High shaping time: " << high_shaping_time << "us");

    if (mca_state527_)
    {
      ROS_INFO_STREAM("Max shaping time: " << 0.1 * mca_state527_->MaxShapingTime << "us");
    }

    // Check that high voltage requested is not higher than the maximum one allowed by the device
    if (high_voltage_ > max_high_voltage_)
    {
      ROS_FATAL_STREAM("High voltage " << high_voltage_ << " must not be greater than the maximum high voltage "
                                       << max_high_voltage_);

      mca_->COMM_CLOSE();

      ros::shutdown();
      return;
    }

    // Set high voltage
    error_flag = mca_->MMCA_SET_HIGH_VOLTAGES(high_voltage_, INHIBIT_OFF);
    if (error_flag != McaComm::ERROR_OK)
    {
      ROS_FATAL_STREAM("Failed to set high voltage to " << high_voltage_ << ". Error: " << error_flag);

      mca_->COMM_CLOSE();

      ros::shutdown();
      return;
    }

    ROS_INFO_STREAM("High voltage set to " << high_voltage_ << "V.");

    // Validate "live" and "real" acquisition mode configuration
    if (acquisition_mode_ != "none")
    {
      // Confirm the acquistion time was set properly
      if (static_cast<uint32_t>(acquisition_time_) != mca_state_->PresetValue)
      {
        ROS_WARN_STREAM("Requested acquisition time of "
                        << acquisition_time_ << "s was not set successfully. Actual acquisition time is "
                        << mca_state_->PresetValue << "s. Using actual acquisition time instead.");
        acquisition_time_ = mca_state_->PresetValue;
      }

      // Warn if we will likely sample while the sensor is still acquiring
      if (sample_period_ < acquisition_time_ && !stop_acquisition_before_sampling_)
      {
        ROS_WARN_STREAM("Sample period of "
                        << sample_period_ << "s is smaller than acquisition time of " << acquisition_time_
                        << "s and the acquisition is not stopped before sampling. This could lead to sampling while "
                           "still acquiring. Consider increasing the sample period.");
      }
    }

    // Publish latched calibration coefficients
    gsniffer_msgs::Calibration msg;
    msg.slope = calibration_slope_;
    msg.offset = calibration_offset_;
    msg.square = calibration_square_;

    calibration_publisher_.publish(msg);

    // Start acquisition
    if (!startAcquisition())
    {
      ROS_FATAL_STREAM("Failed to start first acquisition. Shutting down.");

      mca_->COMM_CLOSE();

      ros::shutdown();
      return;
    }

    // Start the sample timer
    sample_timer_.start();
  }

  /**
   * @brief Destructor
   *
   * Stops the timers and shutdowns the MCA and related handlers cleanly
   */
  ~GSnifferNode()
  {
    // Stop timers
    sample_timer_.stop();
    power_publish_timer_.stop();
    state_publish_timer_.stop();
    system_data_publish_timer_.stop();

    // Shutdown MCA
    if (mca_)
    {
      // Stop acquisition
      auto error_flag = mca_->MMCA_STOP_ACQUIRE();
      if (error_flag != McaComm::ERROR_OK)
      {
        ROS_ERROR_STREAM("Failed to stop acquisition. Error flag: " << error_flag);
      }

      // Disconnect from MCA device
      mca_->COMM_CLOSE();
    }

    // Destroy MCA527 state
    if (mca_state527_)
    {
      delete mca_state527_;
    }

    if (mca_state527_ex_)
    {
      delete mca_state527_ex_;
    }

    // Destroy MCA and MCA state
    if (mca_state_)
    {
      delete mca_state_;
    }
  }

  /**
   * @brief Checkes whether the channels number is valid or not, i.e. it is a power of 2 in the [128, 16284] range
   *
   * @return True if the channels number is valid; false otherwise
   */
  static bool isChannelsValid(const int channels)
  {
    for (int i = 128; i <= 16284; i <<= 1)
    {
      if (i == channels)
      {
        return true;
      }
    }

    return false;
  }

  /**
   * @brief Creates the folder to save the .spe files
   *
   * If save_folder_ is empty, no .spe file will be saved, which is not recommended.
   */
  void createSaveFolder()
  {
    if (save_folder_.empty())
    {
      ROS_WARN("Saving .spe files is disabled.");
      return;
    }

    if (std::filesystem::exists(save_folder_))
    {
      if (!std::filesystem::is_directory(save_folder_))
      {
        ROS_ERROR_STREAM("The save folder '" << save_folder_
                                             << "' already exists but it is not a directory. Saving .spe files "
                                                "will be disabled.");
        save_folder_ = "";
      }
    }
    else
    {
      try
      {
        if (!std::filesystem::create_directories(save_folder_))
        {
          ROS_ERROR_STREAM("Failed to create save folder '" << save_folder_
                                                            << "'. Saving .spe files will be disabled.");
          save_folder_ = "";
        }
      }
      catch (const std::filesystem::filesystem_error& ex)
      {
        ROS_ERROR_STREAM("Failed to create save folder '" << save_folder_ << "'. Error: " << ex.what()
                                                          << ". Saving .spe files will be disabled.");
        save_folder_ = "";
      }
    }
  }

  /**
   * @brief Starts acquisition
   *
   * @return True if the acquisition was started successfully; false otherwise
   */
  bool startAcquisition()
  {
    // Start acquisition
    time_t now;
    time(&now);
    const auto error_flag = mca_->MMCA_START_ACQUIRE(1, McaComm::ToMcaTime(now));
    if (error_flag != McaComm::ERROR_OK)
    {
      ROS_WARN_STREAM("Failed to start acquisition. Error: " << error_flag);

      return false;
    }

    return true;
  }

  /**
   * @brief Checks whether an acquisition is running or not
   *
   * @return True if there is an acquisition running; false otherwise
   */
  bool isAcquisitionRunning()
  {
    // Query system data
    McaComm::QUERY_SYSTEM_DATA mca_system_data;
    const auto error_flag = mca_->MMCA_QUERY_SYSTEM_DATA(&mca_system_data);
    if (error_flag != McaComm::ERROR_OK)
    {
      ROS_ERROR_STREAM(
          "Failed to query MCA system data. Assuming MCA state is not running any acquisition. Error: " << error_flag);

      return false;
    }

    return mca_system_data.McaState == STATE_RUN;
  }

  /**
   * @brief Callback to start a new acquisition to take a new sample of the spectrum.
   *
   * This function gets invoked by the sample timer.
   *
   * If already sampling we also skip sampling and warn the user the sample period should be increased because it is not
   * sufficiently larger than the acquisition time.
   */
  void sample(const ros::TimerEvent&)
  {
    // Stop acquisition before sampling
    if (stop_acquisition_before_sampling_)
    {
      const auto error_flag = mca_->MMCA_STOP_ACQUIRE();
      if (error_flag != McaComm::ERROR_OK)
      {
        ROS_WARN_STREAM_THROTTLE(30.0, "Failed to stop acquisition before sampling. Error: " << error_flag);
      }
    }

    // Query spectrum sample
    if (!querySpectrum())
    {
      ROS_WARN_STREAM_THROTTLE(30.0, "Failed to query spectrum sample.");
    }

    // Re-start acquisition if there is no acquisition running
    if (!isAcquisitionRunning())
    {
      if (!startAcquisition())
      {
        ROS_ERROR_THROTTLE(30.0, "Failed to start new acquisition.");

        return;
      }
    }
  }

  /**
   * @brief Queries the spectrum and publish it.
   *
   * @return True if the spectrum was successfully queried; false otherwise
   */
  bool querySpectrum()
  {
    // Query system data
    McaComm::QUERY_SYSTEM_DATA mca_system_data;
    auto error_flag = mca_->MMCA_QUERY_SYSTEM_DATA(&mca_system_data);
    if (error_flag != McaComm::ERROR_OK)
    {
      ROS_ERROR_STREAM_THROTTLE(30.0, "Failed to query MCA system data. Error: " << error_flag);

      return false;
    }

    // Query spectrum
    error_flag = mca_->MMCA_QUERY_COMPLETE_SPECTRUM(first_channel_, last_channel_, 1, mca_data_.Spectrum.data());
    if (error_flag != McaComm::ERROR_OK)
    {
      ROS_WARN_STREAM_THROTTLE(30.0, "Failed to query complete spectrum. Error: " << error_flag);

      return false;
    }

    // Query state
    error_flag = mca_->MMCA_QUERY_STATE(mca_state_);
    if (error_flag != McaComm::ERROR_OK)
    {
      ROS_WARN_STREAM_THROTTLE(30.0, "Failed to query MCA state to fill in spectrum message. Error: " << error_flag);

      return false;
    }

    if (mca_state_->FirmwareVersion == 0xFFFF)  // MCA-527
    {
      error_flag = mca_->MMCA_QUERY_STATE527(mca_state527_);
      if (error_flag != McaComm::ERROR_OK)
      {
        ROS_WARN_STREAM_THROTTLE(30.0,
                                 "Failed to query MCA-527 state to fill in spectrum message. Error: " << error_flag);

        return false;
      }
    }

    // Query power
    McaComm::QUERY_POWER mca_power;
    error_flag = mca_->MMCA_QUERY_POWER(&mca_power);
    if (error_flag != McaComm::ERROR_OK)
    {
      ROS_WARN_STREAM_THROTTLE(30.0,
                               "Failed to query MCA power state to fill in spectrum message. Error: " << error_flag);

      return false;
    }

    // Fill in the metadata
    mca_data_.FillFromMca(mca_state_, mca_state527_, &mca_power, &mca_system_data);

    // Publish spectrum data
    gsniffer_msgs::Spectrum msg;
    // NOTE For the spectrum we could optimize it out by allocating its memory before, likely having a member msg
    // variable
    //
    // NOTE The stamp should be set based on the time the acquisition starts, which is in the message, but it could be
    // there is an offset between that and ros::Time::now()
    const auto stamp = ros::Time::now();
    msg.header.stamp = stamp;
    msg.first_channel = first_channel_;
    msg.last_channel = last_channel_;
    msg.channels = mca_data_.Spectrum;
    msg.life_time.sec = mca_data_.LifeTime;
    msg.real_time.sec = mca_data_.RealTime;
    msg.flat_top_time.sec = mca_data_.FlatTop / 10000000;
    msg.flat_top_time.nsec = 100 * mca_data_.FlatTop;
    msg.high_voltage = mca_data_.Uhvs;
    msg.counts = mca_data_.Counts;
    msg.peak_detector_counts = mca_data_.PdCounts;

    const auto& integral = mca_data_.Integral;  // summation of channels from first_channel to last_channel
    double sample_period = static_cast<double>(msg.life_time.sec);  // sample period if the acquisition has finished
                                                                    // when we take a sample, which is always the case
                                                                    // when we stop the acquisition before sampling
    if (stop_acquisition_before_sampling_)
    {
      msg.integral = integral;
    }
    else
    {
      msg.integral = integral - previous_integral_;
      previous_integral_ = integral;

      if (acquisition_mode_ == "none")
      {
        sample_period = sample_period_;  // acquisition is never stopped, so the sample period cannot be the life time
      }
      else
      {
        if (isAcquisitionRunning())
        {
          sample_period = sample_period_;  // acquisition still running, so the sample period cannot be the life time
        }
        else
        {
          sample_period = std::fmod(sample_period, sample_period_);  // remainder of the life time and the sample period
        }
      }
    }

    msg.cps = msg.integral / sample_period;
    msg.detector_temperature = mca_data_.DetectorTemperature;
    msg.dead_time.sec = mca_data_.DeadTime / 1000;
    msg.dead_time.nsec = 1000000 * (mca_data_.DeadTime % 1000);
    msg.busy_time.sec = mca_data_.BusyTime / 1000;
    msg.busy_time.nsec = 1000000 * (mca_data_.BusyTime % 1000);
    msg.mca_temperature = mca_data_.McaTemperature;

    spectrum_publisher_.publish(msg);

    // Save .spe file
    if (!save_folder_.empty())
    {
      // Generate filename based on time stamp
      std::ostringstream oss;
      oss << boost::asio::ip::host_name() << '-' << stamp.sec << '.' << std::setw(9) << std::setfill('0')
          << stamp.nsec % 1000000000;

      const auto stamp_string = oss.str();
      const auto filename = (std::filesystem::path(save_folder_) / (stamp_string + ".spe")).string();

      mca_data_.SpecRem = private_node_handle_.getNamespace() + " sample acquired at " + stamp_string;

      // Save .spe file
      SpeFile::GBS_MCA_SpeFile spe_output;
      if (!spe_output.WriteSpectrum(filename, &mca_data_))
      {
        ROS_WARN_STREAM("Failed to save spectrum sample to file '" << filename << "'.");
      }
      else
      {
        ROS_INFO_STREAM("Spectrum sample successfully saved to file '" << filename << "'.");
      }
    }

    return true;
  }

  /**
   * @brief Callback to query and lazy publish the power state.
   *
   * This function gets invoked by the power publish timer.
   */
  void publishPower(const ros::TimerEvent&)
  {
    // Skip if no subscriber is connected to the topic
    if (power_publisher_.getNumSubscribers() == 0)
    {
      return;
    }

    // Query power state
    McaComm::QUERY_POWER mca_power;
    const auto error_flag = mca_->MMCA_QUERY_POWER(&mca_power);
    if (error_flag != McaComm::ERROR_OK)
    {
      ROS_WARN("Failed to query MCA power state.");
      return;
    }

    // Publish power message
    gsniffer_msgs::Power msg;

    msg.header.stamp = ros::Time::now();
    msg.high_voltage = 1.2 * mca_power.HighVoltage;

    power_publisher_.publish(msg);
  }

  /**
   * @brief Callback to query and lazy publish the state.
   *
   * This function gets invoked by the state publish timer.
   */
  void publishState(const ros::TimerEvent&)
  {
    // Skip if no subscriber is connected to the topic
    if (state_publisher_.getNumSubscribers() == 0)
    {
      return;
    }

    // Query state
    auto error_flag = mca_->MMCA_QUERY_STATE(mca_state_);
    if (error_flag != McaComm::ERROR_OK)
    {
      ROS_WARN_STREAM("Failed to query MCA state. Error: " << error_flag);
      return;
    }

    // Query MCA-527 state
    McaComm::QUERY_STATE527 mca_state527;
    if (mca_state_->FirmwareVersion == 0xFFFF)  // MCA-527
    {
      error_flag = mca_->MMCA_QUERY_STATE527(&mca_state527);
      if (error_flag != McaComm::ERROR_OK)
      {
        ROS_ERROR_STREAM("Failed to query MCA-527 sate. Error: " << error_flag);

        // TODO Go into error processing state
        return;
      }
    }

    // Publish state message
    gsniffer_msgs::State msg;

    msg.header.stamp = ros::Time::now();
    if (mca_state_->FirmwareVersion == 0xFFFF)  // MCA-527
    {
      // TODO Consider having a msg attribute or a class encapsulating a state sampler, so the constant fields are only
      // queried and/or set once
      //
      // It could also handle the MCA-527 state internally
      switch (mca_state527.HwModification)
      {
        case 0:
          msg.device_type = gsniffer_msgs::State::DEVICE_TYPE_MCA527_FULL;
          break;
        case 1:
          msg.device_type = gsniffer_msgs::State::DEVICE_TYPE_MCA527_LITE;
          break;
        case 2:
          msg.device_type = gsniffer_msgs::State::DEVICE_TYPE_MCA527_OEM;
          break;
        case 3:
          msg.device_type = gsniffer_msgs::State::DEVICE_TYPE_MCA527_MICRO;
          break;
        case 4:
          msg.device_type = gsniffer_msgs::State::DEVICE_TYPE_MCA527_NANO;
          break;
        default:
          msg.device_type = gsniffer_msgs::State::DEVICE_TYPE_MCA166;
      }
      msg.hardware_version = mca_state527.HwVersion;
      msg.firmware_version = mca_state527.FwVersion;
      msg.threshold = 0.1f * mca_state527.ThresholdTenths;
    }
    else
    {
      msg.device_type = gsniffer_msgs::State::DEVICE_TYPE_MCA166;
      msg.hardware_version = mca_state_->HardwareVersion;
      msg.firmware_version = mca_state_->FirmwareVersion;
      msg.threshold = mca_state_->Threshold;
    }
    msg.serial_number = mca_state_->McaNumber;
    msg.preset = mca_state_->Presets;
    msg.preset_time = mca_state_->PresetValue;
    msg.lld = mca_state_->Lld;
    msg.uld = mca_state_->Uld;
    msg.coarse_gain = mca_state_->CoarseGain;
    msg.fine_gain = mca_state_->FineGain;
    msg.detector_bias = mca_state_->DetectorBias;
    if (mca_state_->DetectorBiasPoly != POLARITY_POSITIVE)
    {
      msg.detector_bias *= -1;
    }
    msg.pzc = mca_state_->PzcValue;

    state_publisher_.publish(msg);
  }

  /**
   * @brief Callback to query and lazy publish the system data.
   *
   * This function gets invoked by the system data publish timer.
   */
  void publishSystemData(const ros::TimerEvent&)
  {
    // Skip if no subscriber is connected to the topic
    if (system_data_publisher_.getNumSubscribers() == 0)
    {
      return;
    }

    // Query system data
    McaComm::QUERY_SYSTEM_DATA mca_system_data;
    const auto error_flag = mca_->MMCA_QUERY_SYSTEM_DATA(&mca_system_data);
    if (error_flag != McaComm::ERROR_OK)
    {
      ROS_WARN_STREAM("Failed to query MCA system data. Error: " << error_flag);
      return;
    }

    // Publish system data message
    gsniffer_msgs::SystemData msg;

    msg.header.stamp = ros::Time::now();
    msg.state = mca_system_data.McaState;

    system_data_publisher_.publish(msg);
  }

private:
  ros::NodeHandle private_node_handle_{ "~" };  // Private node handle

  ros::Publisher calibration_publisher_;  // Calibration latched publisher
  ros::Publisher spectrum_publisher_;     // Spectrum publisher
  ros::Publisher power_publisher_;        // Power lazy publisher
  ros::Publisher state_publisher_;        // State lazy publisher
  ros::Publisher system_data_publisher_;  // System data lazy publisher

  ros::Timer sample_timer_;               // Sample timer
  ros::Timer power_publish_timer_;        // Timer to publish the power
  ros::Timer state_publish_timer_;        // Timer to publish the state
  ros::Timer system_data_publish_timer_;  // Timet to publish the system data

  std::shared_ptr<McaComm::GBS_MCA_Comm> mca_;   // MCA
  McaComm::QUERY_STATE* mca_state_;              // MCA state
  McaComm::QUERY_STATE527* mca_state527_;        // MCA-527 state
  McaComm::QUERY_STATE527_EX* mca_state527_ex_;  // MCA-527 extended state

  McaData::GBS_MCA_Data mca_data_;  // MCA data

  uint16_t max_high_voltage_;  // MCA maximum high voltage; device-specific parameter read on startup
  uint16_t max_channels_;      // MCA maximum channels; device-specific parameter read on startup
  uint16_t max_uld_;           // MCA maximum upper channel; device-specific parameter read on startup
  uint16_t max_fine_gain_;     // MCA maximum fine gain; device-specific parameter read on startup
  uint16_t max_flat_top_;      // MCA maximum flat top time * 0.1 μs; device-specific parameter read on startup

  double timeout_{ 1.0 };  // MCA initialization timeout in seconds
  int attempts_{ 8 };      // MCA initialization attempts
  int baud_rate_{ 0 };     // MCA initialization baud rate; 0 means the highest possible baud rate is set

  double sample_period_{ 15.0 };            // Sample period in seconds
  std::string acquisition_mode_{ "none" };  // Acquisition mode: {"none", "live", "real"}
  int acquisition_time_{ 10 };  // Acquisition time in seconds; only applies to live and real acquisition modes
  bool stop_acquisition_before_sampling_{ true };  // Whether to stop the acquisition before sampling or not
  int high_voltage_{ 1500 };                       // MCA high voltage in volts
  int first_channel_{ 0 };                         // First channel to sample from the spectrum
  int last_channel_{ -1 };    // Last channel to sample from the spectrum. If lower than 0, it is set to the number of
                              // channels available in the MCA minus 1, i.e. the last channel of all the zero-indexed
                              // channels available
  int channels_{ 1024 };      // Number of channels for the spectrum. It must be a power of 2 from 128 to 16284, and not
                              // higher than the maximum number of channels reported by the sensor at runtime
  int lld_{ 0 };              // Lower channel. It must be lower than the upper channel uld
  int uld_{ channels_ - 1 };  // Upper channel. It must be smaller than the maximum uld reported by the sensor at
                              // runtime
  int coarse_gain_{ 10 };     // Coarse gain. It must be 2, 5, 10, 20, 50, 100, 200, 500 or 1000
  int fine_gain_{ 11252 };  // Fine gain. It must be >= 5000 and <= the max fine gain reported by the sensor at runtime.
                            // For MCA-166 the product of the coarse and fine gains must be <= 10000000
  double shaping_time_{ -1.0 };  // Shaping time in microseconds (us). If negative, it is not set
  std::string save_folder_;  // Save folder to save .spe files for each acquisition sample. If empty, no .spe file is
                             // saved
  bool sampling_{ false };   // Flag to indicate the spectrum is being acquired, to avoid re-starting the query spectrum
                            // timer before we have queried it from the previous acquisition started; otherwise we would
                            // never query any spectrum because the timer would not be able to trigger

  uint64_t previous_integral_{ 0 };  // The previous integral, used to compute the sample integral when the acquisition
                                     // is not stopped before sampling because in that case the spectrum channels do not
                                     // reset the counts after taking a sample, so we must compute the difference
                                     // between the current integral and the previous one

  double power_publish_period_{ 1.0 };        // Power state publish period in seconds
  double state_publish_period_{ 1.0 };        // State publish period in seconds
  double system_data_publish_period_{ 1.0 };  // System data publish period in seconds

  // Linear calibration: channel = (energy - offset) / slope
  // Quadratic calibration: channel = (sqrt(slope * slope - 4 * square * (offset - energy)) - slope) / (2 * square)
  float calibration_slope_{ 1.0 };   // Calibration slope
  float calibration_offset_{ 0.0 };  // Calibration offset
  float calibration_square_{ 0.0 };  // Calibration square
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "gsniffer");

  GSnifferNode node;

  ros::spin();

  return EXIT_SUCCESS;
}
