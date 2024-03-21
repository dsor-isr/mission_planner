/*
Developers: #DSORTeam -> @tecnico.ulisboa.pt Instituto Superior Tecnico 
*/
 #ifndef CATKIN_WS_GLIDERMISSIONPLANNERNODE_H
 #define CATKIN_WS_GLIDERMISSIONPLANNERNODE_H

 //some generically useful stuff to include...
 #include <std_msgs/String.h>
 #include <vector>
 #include <ros/ros.h> 

 #include <farol_gimmicks_library/FarolGimmicks.h>

 #include "GliderMissionPlannerAlgorithm.h"

 #include "glider_mission_planner/InterestZone.h"
 #include "glider_mission_planner/Configs.h"
 #include "auv_msgs/NavigationStatus.h"

/* -------------------------------------------------------------------------*/
/**
 * @brief  ADD HERE A SMALL DESCRIPTION OF THE NODE'S OBJECTIVE
 */
/* -------------------------------------------------------------------------*/
 class GliderMissionPlannerNode {
 public:
   
   /* -------------------------------------------------------------------------*/
   /**
    * @brief Constructor
    *
    * @Param nodehandle
    * @Param nodehandle_private
    */
   /* -------------------------------------------------------------------------*/
 	GliderMissionPlannerNode(ros::NodeHandle* nodehandle, ros::NodeHandle *nodehandle_private);

  /* -------------------------------------------------------------------------*/
  /**
   * @brief  Destructor
   */
  /* -------------------------------------------------------------------------*/
 	~GliderMissionPlannerNode();

  

  // @.@ Public methods



 private:
 	ros::NodeHandle nh_;          ///< ROS nodehandler
 	ros::NodeHandle nh_private_;  ///< ROS private nodehandler

  // algorithm instance
  std::unique_ptr<GliderMissionPlannerAlgorithm> glider_mission_planner_alg_;

 	// @.@ Subscribers
  ros::Subscriber interest_zone_sub_;
  ros::Subscriber state_sub_;
  ros::Subscriber change_configs_sub_;


 	// @.@ Publishers


 	// @.@ Timer
 	ros::Timer timer_;           ///< ROS Timer

  // @.@ Parameters from Yaml
  double p_node_frequency_;   ///< node frequency
  int path_orientation_;
  std::string path_type_;
  double min_turning_radius_;
  double path_speed_;
  double resolution_;

 	// @.@ Problem variables
  std::vector<double> veh_pos_ = {0.0, 0.0};
 	

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
  

  /* -------------------------------------------------------------------------*/
  /**
   * @brief  Timer iteration callback
   *
   * @Param event
   */
  /* -------------------------------------------------------------------------*/
  void timerIterCallback(const ros::TimerEvent& event);



  // @.@ Services declaration
  bool changeConfigsService(glider_mission_planner::Configs::Request &req,
                            glider_mission_planner::Configs::Response &res);
  bool interestZoneService(glider_mission_planner::InterestZone::Request &req,
                           glider_mission_planner::InterestZone::Response &res);


  // @.@ Member helper functions


};
#endif //CATKIN_WS_CONTROLNODE_H
