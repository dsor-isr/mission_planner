/* 
 * Developers: DSOR Team -> @tecnico.ulisboa.pt Instituto Superior Tecnico 
 */
#include "GliderMissionPlannerNode.h"
#include "GliderMissionPlannerAlgorithm.h"

// @.@ Constructor
GliderMissionPlannerNode::GliderMissionPlannerNode(ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private):nh_(*nodehandle), nh_private_(*nodehandle_private) {

  loadParams();
  initializeSubscribers();
  initializePublishers();
  initializeServices();
  initializeTimer();

  glider_mission_planner_alg_ = std::make_unique<GliderMissionPlannerAlgorithm>();

}

// @.@ Destructor
GliderMissionPlannerNode::~GliderMissionPlannerNode() {

  // +.+ shutdown publishers
  mission_string_pub_.shutdown();

  // +.+ shutdown subscribers
  state_sub_.shutdown();
  interest_zone_acomms_sub_.shutdown();

  // +.+ stop timer
  timer_.stop();

  // +.+ shutdown node
  nh_.shutdown();
  nh_private_.shutdown();
}

// @.@ Member helper to load parameters from parameter server
void GliderMissionPlannerNode::loadParams() {
  ROS_INFO("Load the GliderMissionPlannerNode parameters");

  p_node_frequency_ = FarolGimmicks::getParameters<double>(nh_private_, "node_frequency", 5);
  path_orientation_ = FarolGimmicks::getParameters<int>(nh_private_, "path_orientation", 1);
  path_type_ = FarolGimmicks::getParameters<std::string>(nh_private_, "path_type", "lawnmower_normal");
  min_turning_radius_ = FarolGimmicks::getParameters<double>(nh_private_, "min_turning_radius", 50);
  path_speed_ = FarolGimmicks::getParameters<double>(nh_private_, "path_speed", 0.7);
  resolution_ = FarolGimmicks::getParameters<double>(nh_private_, "resolution", 10);

}

bool GliderMissionPlannerNode::interestZoneService(glider_mission_planner::InterestZone::Request &req,
                                                   glider_mission_planner::InterestZone::Response &res) {
  // check if max min values are correct
  if (req.northing_min > req.northing_max || req.easting_min > req.easting_max) {
    res.success = false;
    res.message = "Invalid interest zone.";
    return false;
  }

  // start new mission according to zone of interest published
  glider_mission_planner_alg_->startNewMission(req.northing_min, req.northing_max, req.easting_min, req.easting_max, 
                                               path_orientation_, veh_pos_, min_turning_radius_, resolution_,
                                               path_type_, path_speed_, mission_string_pub_);
  res.success = true;
  res.message = "Started new PF Mission on interest zone.";
  return true;
}

void GliderMissionPlannerNode::stateCallback(const auv_msgs::NavigationStatus &msg) {
  // update vehicle position
  veh_pos_[0] = msg.position.east;
  veh_pos_[1] = msg.position.north;
}

void GliderMissionPlannerNode::interestZoneAcommsCallback(const glider_mission_planner::mInterestZone &msg) {
  // ack msg
  std_msgs::Bool ack_msg;
  
  // check if max min values are correct
  if (msg.northing_min > msg.northing_max || msg.easting_min > msg.easting_max) { // not good
    // send acoustic message back saying PF HAS NOT started
    // send FALSE
    ack_msg.data = false;

  } else { // everything ok, let's start PF
    // start new mission according to zone of interest published
    glider_mission_planner_alg_->startNewMission(msg.northing_min, msg.northing_max, msg.easting_min, msg.easting_max, 
                                                path_orientation_, veh_pos_, min_turning_radius_, resolution_,
                                                path_type_, path_speed_, mission_string_pub_);
    
    // send acoustic message back saying PF HAS started
    // send TRUE
    ack_msg.data = true;
  }

  mission_started_ack_pub_.publish(ack_msg);
}

bool GliderMissionPlannerNode::changeConfigsService(glider_mission_planner::Configs::Request &req,
                                                    glider_mission_planner::Configs::Response &res) {
  // check if new configs are OK (within acceptable values)
  if ((req.path_orientation != 0 && req.path_orientation != 1) ||
      (req.path_type != "lawnmower_normal" && req.path_type != "lawnmower_encircling") ||
      (req.min_turning_radius < 0) ||
      (req.path_speed <= 0) ||
      (req.resolution < 0)) {
    ROS_WARN_STREAM("Incorrect configurations for Mission Planner.");
    res.success = false;
    res.message = "Incorrect configurations for Mission Planner.";
    return false;
  }
  
  // update configs
  path_orientation_ = req.path_orientation;
  path_type_ = req.path_type;
  min_turning_radius_ = req.min_turning_radius;
  path_speed_ = req.path_speed;
  resolution_ = req.resolution;
  ROS_INFO("Updated configurations for Mission Planner.");
  res.success = true;
  res.message = "Updated configurations for Mission Planner.";
  return true;
}


// @.@ Member helper function to set up subscribers
void GliderMissionPlannerNode::initializeSubscribers() {
  ROS_INFO("Initializing Subscribers for GliderMissionPlannerNode");

  // subscribe to the vehicle state to update veh_pos_
  state_sub_ = nh_private_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, 
    "topics/subscribers/state", "dummy"),
    10, &GliderMissionPlannerNode::stateCallback, this);

  // subscribe to the interest zone that comes via acoustic comms
  interest_zone_acomms_sub_ = nh_private_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, 
    "topics/subscribers/interest_zone_acomms", "dummy"),
    10, &GliderMissionPlannerNode::interestZoneAcommsCallback, this);
}


// @.@ Member helper function to set up publishers
void GliderMissionPlannerNode::initializePublishers() {
  ROS_INFO("Initializing Publishers for GliderMissionPlannerNode");

  // publisher for the new mission string
  mission_string_pub_ = nh_private_.advertise<std_msgs::String>(
      FarolGimmicks::getParameters<std::string>(
          nh_private_, "topics/publishers/Mission_String", "dummy"), 1);

  mission_started_ack_pub_ = nh_private_.advertise<std_msgs::Bool>(
      FarolGimmicks::getParameters<std::string>(
          nh_private_, "topics/publishers/mission_started_ack", "dummy"), 1);
}


// @.@ Member helper function to set up services
void GliderMissionPlannerNode::initializeServices() {
  ROS_INFO("Initializing Services for GliderMissionPlannerNode");

  // servers
  interest_zone_srv_ = nh_.advertiseService(FarolGimmicks::getParameters<std::string>(nh_private_, 
    "topics/services/interest_zone", "dummy"), &GliderMissionPlannerNode::interestZoneService, this);
  
  change_configs_srv_ = nh_.advertiseService(FarolGimmicks::getParameters<std::string>(nh_private_, 
    "topics/services/change_configs", "dummy"), &GliderMissionPlannerNode::changeConfigsService, this);

}


// @.@ Member helper function to set up the timer
void GliderMissionPlannerNode::initializeTimer() {
  timer_ =nh_.createTimer(ros::Duration(1.0/p_node_frequency_), &GliderMissionPlannerNode::timerIterCallback, this);
}


// @.@ Where the magic should happen.
void GliderMissionPlannerNode::timerIterCallback(const ros::TimerEvent &event) {

}


/*
  @.@ Main
*/
int main(int argc, char** argv)
{
  // +.+ ROS set-ups:
  ros::init(argc, argv, "glider_mission_planner_node"); //node name
  
  // +.+ node handle
  ros::NodeHandle nh;

  // +.+ private node handle
  ros::NodeHandle nh_private("~");

  ROS_INFO("main: instantiating an object of type GliderMissionPlannerNode");

  // +.+ instantiate an GliderMissionPlannerNode class object and pass in pointers to nodehandle public and private for constructor to use
  GliderMissionPlannerNode glider_mission_planner(&nh,&nh_private);

  // +.+  Going into spin; let the callbacks do all the magic
  ros::spin();

  return 0;
}
