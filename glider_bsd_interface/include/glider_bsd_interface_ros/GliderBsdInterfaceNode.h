/*
Developers: #DSORTeam -> @tecnico.ulisboa.pt Instituto Superior Tecnico 
*/
 #ifndef CATKIN_WS_GLIDERBSDINTERFACENODE_H
 #define CATKIN_WS_GLIDERBSDINTERFACENODE_H

 //some generically useful stuff to include...
 #include <std_msgs/String.h>
 #include <std_msgs/Float64.h>
 #include <std_msgs/Float32.h>
 #include <dsor_msgs/Measurement.h>
 #include <vector>
 #include <ros/ros.h> 

 #include <farol_gimmicks_library/FarolGimmicks.h>

 #include <GeographicLib/UTMUPS.hpp>

 #include "GliderBsdInterfaceAlgorithm.h"

/* -------------------------------------------------------------------------*/
/**
 * @brief  ADD HERE A SMALL DESCRIPTION OF THE NODE'S OBJECTIVE
 */
/* -------------------------------------------------------------------------*/
 class GliderBsdInterfaceNode {
 public:
   
   /* -------------------------------------------------------------------------*/
   /**
    * @brief Constructor
    *
    * @Param nodehandle
    * @Param nodehandle_private
    */
   /* -------------------------------------------------------------------------*/
 	GliderBsdInterfaceNode(ros::NodeHandle* nodehandle, ros::NodeHandle *nodehandle_private);

  /* -------------------------------------------------------------------------*/
  /**
   * @brief  Destructor
   */
  /* -------------------------------------------------------------------------*/
 	~GliderBsdInterfaceNode();

  

  // @.@ Public methods



 private:
 	ros::NodeHandle nh_;          ///< ROS nodehandler
 	ros::NodeHandle nh_private_;  ///< ROS private nodehandler

 	// @.@ Subscribers
  
  // data measured by the glider (m_*)
  ros::Subscriber m_gps_lat_sub_;
  ros::Subscriber m_gps_lon_sub_;
  ros::Subscriber m_est_lat_sub_;
  ros::Subscriber m_est_lon_sub_;
  ros::Subscriber m_depth_sub_;
  ros::Subscriber m_pitch_sub_;

  // control references
  ros::Subscriber yaw_ref_sub_;
  ros::Subscriber surge_ref_sub_;

 	// @.@ Publishers
  ros::Publisher position_pub_;

 	// @.@ Timer
 	ros::Timer timer_;           ///< ROS Timer

  // @.@ Parameters from Yaml
  double p_node_frequency_;   ///< node frequency
  std::string vehicle_name_;

 	// @.@ Problem variables

  // data measured by the glider (m_*)
  double m_gps_lat_ = 0.0;
  double m_gps_lon_ = 0.0;
  double m_est_lat_ = 0.0;
  double m_est_lon_ = 0.0;
  double m_depth_;
  double m_pitch_;

  // control references
  double yaw_ref_;
  double surge_ref_;

  // make sure position is only published when both lat and lon are known
  bool est_lat_received_ = false;
  bool est_lon_received_ = false;
  bool gps_lat_received_ = false;
  bool gps_lon_received_ = false;

  // algorithm instance
  std::unique_ptr<GliderBsdInterfaceAlgorithm> glider_bsd_interface_alg_;
 	

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


 	
  // @.@ Callbacks declaration

  void mGpsLatCallback(const std_msgs::Float64 &msg);
  void mGpsLonCallback(const std_msgs::Float64 &msg);
  void mEstLatCallback(const std_msgs::Float64 &msg);
  void mEstLonCallback(const std_msgs::Float64 &msg);
  void mDepthCallback(const std_msgs::Float32 &msg);
  void mPitchCallback(const std_msgs::Float32 &msg);
  void yawRefCallback(const std_msgs::Float64 &msg);
  void surgeRefCallback(const std_msgs::Float64 &msg);
  

  /* -------------------------------------------------------------------------*/
  /**
   * @brief  Timer iteration callback
   *
   * @Param event
   */
  /* -------------------------------------------------------------------------*/
  void timerIterCallback(const ros::TimerEvent& event);



  // @.@ Services declaration



  // @.@ Member helper functions


};
#endif //CATKIN_WS_CONTROLNODE_H
