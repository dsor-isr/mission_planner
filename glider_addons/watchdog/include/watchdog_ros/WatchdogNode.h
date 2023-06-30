/*
Developers: #DSORTeam -> @tecnico.ulisboa.pt Instituto Superior Tecnico 
*/
 #ifndef CATKIN_WS_WATCHDOGNODE_H
 #define CATKIN_WS_WATCHDOGNODE_H

 //some generically useful stuff to include...
 #include <std_msgs/String.h>
 #include <vector>
 #include <ros/ros.h> 

 #include <farol_gimmicks_library/FarolGimmicks.h>
 #include <auv_msgs/NavigationStatus.h>
 #include <std_msgs/Int8.h>
 #include <std_msgs/UInt64.h>
 #include <farol_msgs/mUSBLFix.h>
 #include <std_srvs/SetBool.h>

/* -------------------------------------------------------------------------*/
/**
 * @brief  Node to verify if there is an active connection between surface vehicle 
 * and a console room. Also when a vehicle is underwater checks an acoustic connection
 * between the relay vehicle responsible to guarantee a connection to the console room.
 * If the connection between the console room and any vehicle at the surface is lost, 
 * the vehicles stop the sending safety to the thrusters. In case of a vehicle being the 
 * relay one ir also sends an acoustic abort to all underwater vehicles. The underwater 
 * vehicles will also stop publishing the safety to the thrusters and will publish the
 * farol Flag wuth 0 to abort all 
 */
/* -------------------------------------------------------------------------*/
 class WatchdogNode {
 public:
   
   /* -------------------------------------------------------------------------*/
   /**
    * @brief Constructor
    *
    * @Param nodehandle
    * @Param nodehandle_private
    */
   /* -------------------------------------------------------------------------*/
 	WatchdogNode(ros::NodeHandle* nodehandle, ros::NodeHandle *nodehandle_private);

  /* -------------------------------------------------------------------------*/
  /**
   * @brief  Destructor
   */
  /* -------------------------------------------------------------------------*/
 	~WatchdogNode();

  

  // @.@ Public methods



 private:
 	ros::NodeHandle nh_;          ///< ROS nodehandler
 	ros::NodeHandle nh_private_;  ///< ROS private nodehandler

 	// @.@ Subsctibers
	ros::Subscriber nav_state_sub_;       ///< Nav filter state subscriber
	ros::Subscriber acoustic_range_sub_;  ///< Acoustic range sunscriber
	ros::Subscriber shore_connection_vehicle_id_sub_; ///< To change vehicle ID during mission
	ros::Subscriber acoustic_watchdog_abort_sub_; ///< Receive Abort mission from relay vehicle
	ros::Subscriber wifi_client_connection_sub_; ///< Receive wifi communication from shore

 	// @.@ Publishers
	ros::Publisher watchdog_safety_thrusters_pub_; ///< Publishes thruster_safety 
	ros::Publisher farol_flag_pub_;               ///< Publishes farol stack flag mode
	ros::Publisher acoustic_watchdog_abort_pub_;  ///< Publishes an acoustic abort if the vehicle is the relay one.

  // @.@ Services
  ros::ServiceServer force_wifi_srv_;         ///< Service to force wifi over buoy even when underwater


 	// @.@ Timer
 	ros::Timer timer_main_;                 ///< ROS Timer
  ros::Timer timer_shore_;                ///< ROS Timer to manage wifi watchdog
  ros::Timer timer_acoustic_;             ///> ROS Timer to manage acoustic watchdog
  ros::Timer timer_pub_safety_;           ///> ROS Timer to publish safety feature
  ros::Timer timer_pub_acoustic_abort_;   ///> ROS Timer to publish acoustic abort

  // @.@ Parameters from Yaml
  double p_rate_main_;              ///< Rate for main loop  
  double p_rate_shore_;             ///< Rate for wifi check comms loop
  double p_rate_acoustic_;          ///< Rate for acoustics check comms loop
  double p_rate_safety_;            ///< Rate for different timer_acoustic_
  double p_depth_transition_{0.3};  ///< Depth transition between wifi and acoustic
  bool p_relay_vehicle_{false};     ///< Relay vehicle true or false
  int p_shore_vehicle_id_{0};       ///< Relay vehicle id
 	
  // @.@ Problem variables
  double depth_{0.0};               ///< Depth of the vehicle
  bool send_acoustic_abort_;        ///< Bool to send acoustic abort or not
  int recv_acoustic_abort_{0};      ///< Receive acoustic abort
  int acoustic_abort_flag_{0};      ///< Acoustic abort flag

  bool force_wifi_{false};
  ros::Time last_acoustic_ping_;    ///< Time when receive acoustic range
  ros::Time last_wifi_ping_;        ///< Time when receive wifi transmission
  // +.+ Socket variables

  int id_sck_;        ///< Port open or not from WatchdogSocket::makeServer 
  int sck_client_;    ///< When accepting new client from WatchSocket::acceptClient

	bool client_active_{false};       ///< Bool to check if socket client is active or not

 	

  // @.@ Encapsulation the gory details of initializing subscribers, publishers and services
 	


  /* -------------------------------------------------------------------------*/
  /**
   * @brief Load parameters from parameter server 
   */
  /* -------------------------------------------------------------------------*/
 	void loadParams();


  /* -------------------------------------------------------------------------*/
  /**
   * @brief Initialize ROS node Subscribers
   */
  /* -------------------------------------------------------------------------*/
  void initializeSubscribers();


  /* -------------------------------------------------------------------------*/
  /**
   * @brief  Initialize ROS node Publishers
   */
  /* -------------------------------------------------------------------------*/
 	void initializePublishers();

 
  /* -------------------------------------------------------------------------*/
  /**
   * @brief  Initialize ROS node Services
   */
  /* -------------------------------------------------------------------------*/
 	void initializeServices();


  /* -------------------------------------------------------------------------*/
  /**
   * @brief Initialize ROS node Timer  
   */
  /* -------------------------------------------------------------------------*/
 	void initializeTimer();


	/* -------------------------------------------------------------------------*/
	/**
	 * @brief Initialize Socket 
	 */
	/* -------------------------------------------------------------------------*/
  void initializeSocket();
 	
  // @.@ Callbacks declaration
  

  /* -------------------------------------------------------------------------*/
  /**
   * @brief Get depth from filter  
   *
   * @Param msg
   */
  /* -------------------------------------------------------------------------*/
  void updateDepthCallback(const auv_msgs::NavigationStatus &msg); 
  
  
  /* -------------------------------------------------------------------------*/
  /**
   * @brief Check if acoustic message received
   *
   * @Param msg
   */
  /* -------------------------------------------------------------------------*/
  void rangeAcousticCallback(const farol_msgs::mUSBLFix &msg);


  /* -------------------------------------------------------------------------*/
  /**
   * @brief Redefine vehicle with wifi connection to shore
   *
   * @Param msg
   */
  /* -------------------------------------------------------------------------*/
  void shoreVehicleIdCallback(const std_msgs::Int8 &msg);

  
  /* -------------------------------------------------------------------------*/
  /**
   * @brief Check if the shore vehicle is sending an abort via acoustic. Needed 
   * when underwater.
   *
   * @Param msg
   */
  /* -------------------------------------------------------------------------*/
  void acousticAbortCallback(const std_msgs::Int8 &msg);
  

  /* -------------------------------------------------------------------------*/
  /**
   * @brief Register the time of a wifi connection 
   *
   * @param msg
   */
  /* -------------------------------------------------------------------------*/
  void wifiConnectionCallback(const std_msgs::UInt64 &msg);
  

  /* -------------------------------------------------------------------------*/
  /**
   * @brief  Timer iteration callback
   * Only uses wifi at surface. Activate shore Timer and stop acoustic timer. 
   * Note that the start and stop can happen multiples times:
   * - Does nothing if the timer is already started: start().
   * - Does nothing if the timer is already stopped: stop().
   * Else, the vehicle is underwater. Deactivate shore timer and start 
   * acoustic timer.
   *
   * @Param event
   */
  /* -------------------------------------------------------------------------*/
  void timerIterCallback(const ros::TimerEvent& event);
  

  /* -------------------------------------------------------------------------*/
  /**
   * @brief Iteration to check shore connectioni
   *
   * Verifies if the server socket already has a client, if not will try
   * to add one. 
   * In the presence of a client, it will check if the connection is ok.
   *
   * @Param event
   */
  /* -------------------------------------------------------------------------*/
  void timerShoreCallback(const ros::TimerEvent& event);
  
  /* -------------------------------------------------------------------------*/
  /**
   * @brief Iteration to check underwater connection
   *
   * Verifies if the last received acoustic range from the 
   * surface vehicle is inside an interval of time.  
   *
   * Starts safety feature publisher: acoustic range inside 
   * desired interval time.   
   *
   * Stops safety_feature publisher and publish farol flag 
   * to abort mission: acoustic range outside interval of time.
   * 
   * @Param event
   */
  /* -------------------------------------------------------------------------*/
  void timerAcousticCallback(const ros::TimerEvent& event);
  
  /* -------------------------------------------------------------------------*/
  /**
   * @brief Iteration to publish safety feature to the thrusters 
   *
   * @Param event
   */
  /* -------------------------------------------------------------------------*/
  void timerSafetyFeatureCallback(const ros::TimerEvent& event);
  
  /* -------------------------------------------------------------------------*/
  /**
   * @brief Iteration to publish acoustic abort from relay vehicle 
   *
   * @Param event
   */
  /* -------------------------------------------------------------------------*/
  void timerAcousticAbortSendCallback(const ros::TimerEvent& event);



  // @.@ Services declaration

   
  /* -------------------------------------------------------------------------*/
  /**
   * @brief Force wifi connection over buoy even when vehicle is dive 
   *
   * @param req
   * @param std_srvs::SetBool::Response
   */
  /* -------------------------------------------------------------------------*/
  bool forceWifiService(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  
  // @.@ Member helper functions
  	

};
#endif //CATKIN_WS_CONTROLNODE_H
