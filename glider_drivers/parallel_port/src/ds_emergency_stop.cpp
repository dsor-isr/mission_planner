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
#include <std_msgs/Bool.h>

class ES
{
public:
  ES()
  {
    ros::NodeHandle node_("~"), nh_;

    if (!node_.getParamCached("address", emergency_addr))
    {
      ROS_ERROR("No Parameter: address");
      ros::shutdown();
    }

    //-----------------------------------------------------------
    // Publishing
    pub_emergency = nh_.advertise<std_msgs::Bool>("/emergency_stop", 10);

    // Subscribing
    sub_parallel_port = nh_.subscribe("parallel_port/data", 3, &ES::ppCallback, this);
  }

  ~ES() {}

  /*
   * Function to check for emergency stop
   */
  void ppCallback(const std_msgs::UInt8& msg)
  {
    std_msgs::Bool emergency_msg;

    emergency_msg.data = true; // thrusters cannot rotate
    if ((msg.data & emergency_addr) == emergency_addr)
    {
      emergency_msg.data = false;
    }
    pub_emergency.publish(emergency_msg);
  }

private:
  ros::Publisher pub_emergency;

  ros::Subscriber sub_parallel_port;

  int emergency_addr;
};

/*
 *  Main
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "emergency_stop");

  ES emergency_stop_obj;

  ros::spin();

  return EXIT_SUCCESS;
}
