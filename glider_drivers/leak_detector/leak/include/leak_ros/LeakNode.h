/*
Developers: DSOR Team -> @irt.ist.pt Instituto Superior Tecnico */
#ifndef CATKIN_WS_LEAKDSNODE_H
#define CATKIN_WS_LEAKDSNODE_H

// some generically useful stuff to include...
#include <boost/format.hpp>
#include <ros/ros.h> //ALWAYS need to include this
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt8.h>
#ifdef __aarch64__
#include <sys/uio.h>
#else
#include <sys/io.h>
#endif
#include <glider_diagnostics_library/GliderDiagnostics.h>
#include <farol_gimmicks_library/FarolGimmicks.h>

class LeakNode {
public:
  // #############################
  // @.@ Constructor
  // #############################
  LeakNode(ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private);

  // #############################
  // @.@ Destructor
  // #############################
  ~LeakNode();

  // #############################
  // @.@ Public methods
  // #############################
  double nodeFrequency();

private:
  ros::NodeHandle nh_;

  ros::NodeHandle nh_p_;

  // #####################
  // @.@ Subsctibers
  // #####################
  ros::Subscriber parallel_port_sub_;

  // #####################
  // @.@ Publishers
  // #####################
  ros::Publisher leak1_pub_;
  ros::Publisher leak2_pub_;
  ros::Publisher stop_pub_;
  ros::Publisher diag_pub_;

  // ####################################################################################################################
  // member variable: better than using globals; convenient way to pass data
  // from a subscriber to other member functions member variables will retain
  // their values even as callbacks come and go
  // ####################################################################################################################

  std_msgs::Int8 int_stop_;

  // +.+ Parameters from Yaml
  // always p_paraName -> Example: double p_ku;
  std::vector<std::string> p_leak_name_;
  std::vector<int> p_leak_addr_;

  // #######################################################################################
  // @.@ Encapsulation the gory details of initializing subscribers, publishers
  // and services
  // #######################################################################################
  void initializeSubscribers();
  void initializePublishers();
  void initializeTimer();
  void loadParams();

  // #######################################################################################
  // @.@ Callbacks declaration
  // #######################################################################################
  void dropoutCallback(const ros::TimerEvent &event);
  void ppCallback(const std_msgs::UInt8 &msg);

  // @.@ Other functions declaration
  void reporting(const int status1, const int status2);
  bool leak(const unsigned int addr, const unsigned int data);
};
#endif // CATKIN_WS_LEAKDSNODE_H
