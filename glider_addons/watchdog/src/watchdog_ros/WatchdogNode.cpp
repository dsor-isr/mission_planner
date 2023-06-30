/* 
 * Developers: DSOR Team -> @tecnico.ulisboa.pt Instituto Superior Tecnico 
 */
#include "WatchdogNode.h"

// @.@ Constructor
WatchdogNode::WatchdogNode(ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private):nh_(*nodehandle), nh_private_(*nodehandle_private) {

  loadParams();
  initializeSubscribers();
  initializePublishers();
  initializeServices();
  initializeTimer();

}

// @.@ Destructor
WatchdogNode::~WatchdogNode() {

  // +.+ shutdown subscribers
  nav_state_sub_.shutdown();
  acoustic_range_sub_.shutdown();
  shore_connection_vehicle_id_sub_.shutdown();
  acoustic_watchdog_abort_sub_.shutdown();


  // +.+ shutdown publishers
  watchdog_safety_thrusters_pub_.shutdown();
  //stop_thrusters_pub_.shutdown();
  farol_flag_pub_.shutdown();
  acoustic_watchdog_abort_pub_.shutdown();
  

  // +.+ stop timer
  timer_main_.stop();

  // +.+ shutdown node
  nh_.shutdown();
  nh_private_.shutdown();
}

// @.@ Member helper to load parameters from parameter server
void WatchdogNode::loadParams() {
  ROS_INFO("Load the WatchdogNode parameters");

  p_rate_main_ = FarolGimmicks::getParameters<double>(nh_private_, "node_frequency", 10.0);
  p_rate_shore_ = FarolGimmicks::getParameters<double>(nh_private_, "shore_frequency", 2.0);
  p_rate_acoustic_ = FarolGimmicks::getParameters<double>(nh_private_, "acoustic_frequency", 0.2);
  p_rate_safety_ = FarolGimmicks::getParameters<double>(nh_private_, "safety_frequency", 2.0);
  p_depth_transition_ = FarolGimmicks::getParameters<double>(nh_private_, "depth_transition", 0.3);
  p_shore_vehicle_id_ = FarolGimmicks::getParameters<int>(nh_private_,"shore_vehicle_id",2);
  p_relay_vehicle_ = FarolGimmicks::getParameters<bool>(nh_private_,"relay_vehicle",false);
 
  // +.+ Register last communications time
  last_acoustic_ping_ = ros::Time::now();
  last_wifi_ping_ = ros::Time::now();

}


// @.@ Member helper function to set up subscribers
void WatchdogNode::initializeSubscribers() {
  ROS_INFO("Initializing Subscribers for WatchdogNode");

  nav_state_sub_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/state"),1, &WatchdogNode::updateDepthCallback, this);

  acoustic_range_sub_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/usbl_fix"),10, &WatchdogNode::rangeAcousticCallback, this);

  shore_connection_vehicle_id_sub_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/shore_vehicle"),10, &WatchdogNode::shoreVehicleIdCallback, this);

  acoustic_watchdog_abort_sub_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/acoustic_safety_abort"),1, &WatchdogNode::acousticAbortCallback, this);

  wifi_client_connection_sub_ = nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/subscribers/wifi_client"), 1, &WatchdogNode::wifiConnectionCallback, this);
}


// @.@ Member helper function to set up publishers
void WatchdogNode::initializePublishers() {
  ROS_INFO("Initializing Publishers for WatchdogNode");

  watchdog_safety_thrusters_pub_ = nh_.advertise<std_msgs::Int8>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/safety_feature"),10);

  farol_flag_pub_ = nh_.advertise<std_msgs::Int8>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/flag"),10);

  acoustic_watchdog_abort_pub_ = nh_.advertise<std_msgs::Int8>(FarolGimmicks::getParameters<std::string>(nh_private_, "topics/publishers/safety_abort"),10);

}


// @.@ Member helper function to set up services
void WatchdogNode::initializeServices() {
  ROS_INFO("Initializing Services for WatchdogNode");

  force_wifi_srv_ = nh_.advertiseService(FarolGimmicks::getParameters<std::string>(nh_private_, "services/force_wifi", "force_wifi"), &WatchdogNode::forceWifiService, this);
}


// @.@ Member helper function to set up the timer
void WatchdogNode::initializeTimer() {
  timer_main_ = nh_.createTimer(ros::Duration(1.0/p_rate_main_), &WatchdogNode::timerIterCallback, this);
  timer_shore_ = nh_.createTimer(ros::Duration(1.0/p_rate_shore_), &WatchdogNode::timerShoreCallback, this);
  timer_shore_.stop();
  timer_acoustic_ = nh_.createTimer(ros::Duration(1.0/p_rate_acoustic_), &WatchdogNode::timerAcousticCallback, this);
  timer_acoustic_.stop();
  timer_pub_safety_ = nh_.createTimer(ros::Duration(1.0/p_rate_safety_), &WatchdogNode::timerSafetyFeatureCallback, this);
  timer_pub_safety_.stop();
  timer_pub_acoustic_abort_ = nh_.createTimer(ros::Duration(1.0/p_rate_safety_), &WatchdogNode::timerAcousticAbortSendCallback, this);
  timer_pub_acoustic_abort_.start();

}


// @.@ Where the magic should happen.
void WatchdogNode::timerIterCallback(const ros::TimerEvent &event) {

/*
 * Only uses wifi at surface. Activate shore Timer and stop acoustic timer. Note that the start and stop can happen multiples times:
 * - Does nothing if the timer is already started: start().
 * - Does nothing if the timer is already stopped: stop().
 * Else, the vehicle is underwater. Deactivate shore timer and start 
 * acoustic timer.
 */

  if (depth_ > p_depth_transition_ && force_wifi_ == false){
    timer_shore_.stop(); 
    timer_acoustic_.start();
  }
  else {
    timer_acoustic_.stop();
    timer_shore_.start();
  }
}


// @.@ Timer for handling wifi comms
void WatchdogNode::timerShoreCallback(const ros::TimerEvent &event) {

  /*
   * Verifies if the server socket already has a client, if not will try
   * to add one. 
   *
   * In the presence of a client, it will check if the connection is ok.
   */

  if ((ros::Time::now()- last_wifi_ping_).toSec() > (1/p_rate_shore_)){
    timer_pub_safety_.stop();

    if(p_relay_vehicle_){
      acoustic_abort_flag_ = 1;
    }
    // +.+ Note: Flag not changed here, so as soon communication is estaliblished the vehicle returns to the mission
  }
  else{
    timer_pub_safety_.start();
    acoustic_abort_flag_ = 0;
  }

}


// @.@ Timer for handling acoustic comms
void WatchdogNode::timerAcousticCallback(const ros::TimerEvent &event){
  /*
   * Verifies if the last received acoustic range from the 
   * surface vehicle is inside an interval of time.  
   *
   * Starts safety feature publisher: acoustic range inside 
   * desired interval time.   
   *
   * Stops safety_feature publisher and publish farol flag 
   * to abort mission: acoustic range outside interval of time.
   *
   */
  if (((ros::Time::now()- last_acoustic_ping_).toSec() > (1/p_rate_acoustic_)) || recv_acoustic_abort_){
    timer_pub_safety_.stop();
    // +.+ Stop underwater vehicle mission
    FarolGimmicks::publishValue<std_msgs::Int8,const int>(farol_flag_pub_,0);
  }
  else{
    timer_pub_safety_.start();
  }

}


// @.@ Timer to publish Safety Feature to thrusters
void WatchdogNode::timerSafetyFeatureCallback(const ros::TimerEvent &event){
  FarolGimmicks::publishValue<std_msgs::Int8,const int>(watchdog_safety_thrusters_pub_,0);
}


// @.@ Timer to publish acoustic abort from surface to underwater vehicles
void WatchdogNode::timerAcousticAbortSendCallback(const ros::TimerEvent &event){
  FarolGimmicks::publishValue<std_msgs::Int8,const int>(acoustic_watchdog_abort_pub_,acoustic_abort_flag_);
}

// @.@ Update the depth of vehicle
void WatchdogNode::updateDepthCallback(const auv_msgs::NavigationStatus &msg){
  depth_ = msg.position.depth;
}

// @.@ Receive an acoustic range
void WatchdogNode::rangeAcousticCallback(const farol_msgs::mUSBLFix &msg){
  // Check if an acoustic range is received. Also check if the message comes from the shore/relay vehicle.
  if (msg.type == msg.RANGE_ONLY || msg.source_id == p_shore_vehicle_id_)
    last_acoustic_ping_ = ros::Time::now();
}

// @.@ Defines which vehicle is at the surface
void WatchdogNode::shoreVehicleIdCallback(const std_msgs::Int8 &msg){
  p_shore_vehicle_id_ = msg.data;

}

// @.@ Receives an acoustic abort from the surface vehicle
void WatchdogNode::acousticAbortCallback(const std_msgs::Int8 &msg){
    recv_acoustic_abort_ = msg.data;
}

// @.@ Receives a communication from a wifi client
void WatchdogNode::wifiConnectionCallback(const std_msgs::UInt64 &msg){
    ROS_INFO("Receiving from wifi client");
    last_wifi_ping_ = ros::Time::now();
}

bool WatchdogNode::forceWifiService(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){

  if (req.data){
    force_wifi_ = true;
    res.success = true;
    res.message = "Force wifi activated, you can dive with buoy.";
  }
  else{
    force_wifi_ = false;
    res.success = false;
    res.message = " Careful wifi over buoy won't work when underwater.";
  }

  return true;
}


/*
   @.@ Main
   */
int main(int argc, char** argv)
{
  // +.+ ROS set-ups:
  ros::init(argc, argv, "watchdog_node"); //node name

  // +.+ node handle
  ros::NodeHandle nh;

  // +.+ private node handle
  ros::NodeHandle nh_private("~");

  ROS_INFO("main: instantiating an object of type WatchdogNode");

  // +.+ instantiate an WatchdogNode class object and pass in pointers to nodehandle public and private for constructor to use
  WatchdogNode watchdog(&nh,&nh_private);

  // +.+  Going into spin; let the callbacks do all the magic
  ros::spin();

  return 0;
}
