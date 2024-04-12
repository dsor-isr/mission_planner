/*
Developers: #DSORTeam -> @tecnico.ulisboa.pt Instituto Superior Tecnico 
*/
 #ifndef CATKIN_WS_MISSIONPLANNERNODE_H
 #define CATKIN_WS_MISSIONPLANNERNODE_H

 //some generically useful stuff to include...
 #include <std_msgs/String.h>
 #include <std_msgs/Bool.h>
 #include <std_msgs/Empty.h>
 #include <std_msgs/Int8.h>
 #include <vector>
 #include <ros/ros.h> 

 #include <farol_gimmicks_library/FarolGimmicks.h>

 #include "MissionPlannerAlgorithm.h"
 
 #include "mission_planner/mInterestZone.h" // message
 #include "mission_planner/mNewIZMission.h" // message
 #include "mission_planner/mMissionStartedAck.h" // message
 #include "mission_planner/InterestZone.h" // service
 #include "mission_planner/Configs.h" // service
 #include "auv_msgs/NavigationStatus.h"

/* -------------------------------------------------------------------------*/
/**
 * @brief  ADD HERE A SMALL DESCRIPTION OF THE NODE'S OBJECTIVE
 */
/* -------------------------------------------------------------------------*/
 class MissionPlannerNode {
 public:
   
   /* -------------------------------------------------------------------------*/
   /**
    * @brief Constructor
    *
    * @Param nodehandle
    * @Param nodehandle_private
    */
   /* -------------------------------------------------------------------------*/
 	MissionPlannerNode(ros::NodeHandle* nodehandle, ros::NodeHandle *nodehandle_private);

  /* -------------------------------------------------------------------------*/
  /**
   * @brief  Destructor
   */
  /* -------------------------------------------------------------------------*/
 	~MissionPlannerNode();

  

  // @.@ Public methods



 private:
 	ros::NodeHandle nh_;          ///< ROS nodehandler
 	ros::NodeHandle nh_private_;  ///< ROS private nodehandler

  // algorithm instance
  std::unique_ptr<MissionPlannerAlgorithm> mission_planner_alg_;

 	// @.@ Subscribers
  ros::Subscriber state_sub_;
  ros::Subscriber interest_zone_sub_;
  ros::Subscriber new_iz_mission_acomms_sub_;
  ros::Subscriber being_scanned_acomms_sub_;
  ros::Subscriber vehicle_ready_acomms_sub_;
  ros::Subscriber mission_started_ack_acomms_sub_;
  ros::Subscriber stop_pf_sub_;


 	// @.@ Publishers
  ros::Publisher mission_string_pub_;
  ros::Publisher mission_started_ack_pub_;
  ros::Publisher new_iz_mission_pub_;
  ros::Publisher ready_for_mission_pub_;
  ros::Publisher stop_all_pf_pub_;
  ros::Publisher status_flag_pub_;

 	// @.@ Timer
 	ros::Timer timer_;           ///< ROS Timer

  // @.@ Parameters from Yaml
  double p_node_frequency_;   ///< node frequency
  int path_orientation_;
  std::string path_type_;
  double min_turning_radius_;
  double path_speed_;
  double resolution_;
  double dist_inter_vehicles_;
  int vehicle_id_; // id for paths with multiple vehicles (normally in a CPF situation)
  double timeout_ack_;

 	// @.@ Problem variables
  std::vector<double> veh_pos_ = {0.0, 0.0};
  std::set<int> participating_veh_;
  std::set<int> mission_started_veh_; // set of vehicle IDs for which the mission has stated successfully
  mission_planner::mNewIZMission last_IZ_mission_;
  bool waiting_for_mission_started_ack = false;
  double waiting_time_start = 0;
  double time = 0;
 	

  // @.@ Encapsulation the gory details of initializing subscribers, publishers and services
  ros::ServiceServer interest_zone_srv_;
  ros::ServiceServer change_configs_srv_;
 	

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

  void stateCallback(const auv_msgs::NavigationStatus &msg);
  void interestZoneCallback(const mission_planner::mInterestZone &msg);
  void newIZMissionZoneAcommsCallback(const mission_planner::mNewIZMission &msg);
  void beingScannedAcommsCallback(const std_msgs::Empty &msg);
  void vehicleReadyAcommsCallback(const std_msgs::Int8 &msg);
  void stopPFAcomms(const std_msgs::Empty &msg);
  void missionStartedAckAcommsCallback(const mission_planner::mMissionStartedAck &msg);
  

  /* -------------------------------------------------------------------------*/
  /**
   * @brief  Timer iteration callback
   *
   * @Param event
   */
  /* -------------------------------------------------------------------------*/
  void timerIterCallback(const ros::TimerEvent& event);



  // @.@ Services declaration
  bool changeConfigsService(mission_planner::Configs::Request &req,
                            mission_planner::Configs::Response &res);
  bool interestZoneService(mission_planner::InterestZone::Request &req,
                           mission_planner::InterestZone::Response &res);


  // @.@ Member helper functions
  void stopParticipatingVehiclesPF();


};
#endif //CATKIN_WS_CONTROLNODE_H
