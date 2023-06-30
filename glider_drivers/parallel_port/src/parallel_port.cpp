//=================================================================================================
// Copyright (c) 2016, Medusa Team, Instituto Superior Tecnico
// All rights reserved.
//=================================================================================================

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2017, DSOR, IST
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the DSOR/IST nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <ros/ros.h>
#include <string.h>
#include <std_msgs/UInt8.h>
//#include <nmea_msgs/Sentence.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <farol_gimmicks_library/FarolGimmicks.h>
#include <boost/format.hpp>

#include <sys/io.h>

/*
 * Parallel port I/O addresses.
 */
#define DATA 0x378
#define STATUS (DATA + 1)

class PP
{
public:
  PP()
  {
    ros::NodeHandle node, node_("~");

    if (ioperm(DATA, 2, 1))
    {
      ROS_ERROR("You don't have premissions, please root!!");
      ros::shutdown();
      exit(EXIT_FAILURE);
    }

    //-----------------------------------------------------------
    // Publishing
    pub_data = node.advertise<std_msgs::UInt8>(FarolGimmicks::getParameters<std::string>(node_, "topics/publishers/data"), 10);
    pub_diag = node.advertise<diagnostic_msgs::DiagnosticArray>(FarolGimmicks::getParameters<std::string>(node_, "topics/publishers/diagnostics"), 100);

    // Subscribing
    sub_conf = node.subscribe(FarolGimmicks::getParameters<std::string>(node_, "topics/subscribers/conf"), 1, &PP::confCallback, this);

    //-----------------------------------------------------------
    // Setup timer to handle the parallel port
    t_read = node_.createTimer(ros::Duration(1.0), &PP::treadCallback, this);

  }

  ~PP()
  {
  }

  /*
   * Function to handle the parallel port
   */
  void treadCallback(const ros::TimerEvent& event)
  {
    // Read parallel port
    std_msgs::UInt8 sts_msg;
    unsigned char status = inb(STATUS);
    sts_msg.data = status;
    pub_data.publish(sts_msg);

    // Report
    reporting(sts_msg);
  }

  /*
   * Function to output port
   */
  void confCallback(const std_msgs::UInt8& msg)
  {
    // setting the output as requested
    outb(msg.data, DATA);
  }

  /*
   * Reporting the sensor state
   */
  void reporting(const std_msgs::UInt8 msg)
  {
    // Instantiate diagnostic message
    diagnostic_msgs::DiagnosticArray diag_msg;
    diag_msg.header.stamp = ros::Time::now();
    diag_msg.status.resize(1);
    diag_msg.status[0].name = "/sensors" + ros::this_node::getName();
    diag_msg.status[0].hardware_id = "CPU";
    diag_msg.status[0].message = "";
    diag_msg.status[0].level = diagnostic_msgs::DiagnosticStatus::OK;

    // Generate key value
    diagnostic_msgs::KeyValue key;
    key.key = ros::this_node::getName() + "/data";
    key.value = boost::str(boost::format("%d") % ((int) msg.data));
    diag_msg.status[0].values.insert(diag_msg.status[0].values.end(), key);

    pub_diag.publish(diag_msg);
  }

private:
  ros::Publisher pub_data;
  ros::Publisher pub_diag;

  ros::Subscriber sub_conf;

  ros::Timer t_read;

  int default_output;
};

/*
 *  Main
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "parallel_port");

  PP parallel_port;

  ros::spin();

  return EXIT_SUCCESS;
}
