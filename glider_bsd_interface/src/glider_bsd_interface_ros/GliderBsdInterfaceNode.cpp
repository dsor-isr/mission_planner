/* 
 * Developers: DSOR Team -> @tecnico.ulisboa.pt Instituto Superior Tecnico 
 */
#include "GliderBsdInterfaceNode.h"
#include "GliderBsdInterfaceAlgorithm.h"

// @.@ Constructor
GliderBsdInterfaceNode::GliderBsdInterfaceNode(ros::NodeHandle *nodehandle, ros::NodeHandle *nodehandle_private):nh_(*nodehandle), nh_private_(*nodehandle_private) {

  loadParams();
  initializeSubscribers();
  initializePublishers();
  initializeServices();
  initializeTimer();

  glider_bsd_interface_alg_ = std::make_unique<GliderBsdInterfaceAlgorithm>();
}

// @.@ Destructor
GliderBsdInterfaceNode::~GliderBsdInterfaceNode() {

  // +.+ shutdown publishers
  position_pub_.shutdown();

  // +.+ shutdown subscribers
  m_est_lat_sub_.shutdown();  
  m_est_lon_sub_.shutdown();
  m_gps_lat_sub_.shutdown();  
  m_gps_lon_sub_.shutdown();  
  m_depth_sub_.shutdown();
  m_pitch_sub_.shutdown();
  yaw_ref_sub_.shutdown();
  surge_ref_sub_.shutdown();

  // +.+ stop timer
  timer_.stop();

  // +.+ shutdown node
  nh_.shutdown();
  nh_private_.shutdown();
}

// @.@ Member helper to load parameters from parameter server
void GliderBsdInterfaceNode::loadParams() {
  ROS_INFO("Load the GliderBsdInterfaceNode parameters");

  p_node_frequency_ = FarolGimmicks::getParameters<double>(nh_private_, "node_frequency", 5.0);
  vehicle_name_ = FarolGimmicks::getParameters<std::string>(nh_private_, "vehicle_name", "dummy");

}

void GliderBsdInterfaceNode::mGpsLatCallback(const std_msgs::Float64 &msg) {
  m_gps_lat_ = msg.data / 100;
  gps_lat_received_ = true;
}

void GliderBsdInterfaceNode::mGpsLonCallback(const std_msgs::Float64 &msg) {
  m_gps_lon_ = msg.data / 100;
  gps_lon_received_ = true;
}

void GliderBsdInterfaceNode::mEstLatCallback(const std_msgs::Float64 &msg) {
  m_est_lat_ = msg.data / 100;
  est_lat_received_ = true;
}

void GliderBsdInterfaceNode::mEstLonCallback(const std_msgs::Float64 &msg) {
  m_est_lon_ = msg.data / 100;
  est_lon_received_ = true;
}

void GliderBsdInterfaceNode::mDepthCallback(const std_msgs::Float32 &msg) {
  m_depth_ = msg.data;
  glider_bsd_interface_alg_->publishDepth(m_depth_, position_pub_, vehicle_name_);
}

void GliderBsdInterfaceNode::mPitchCallback(const std_msgs::Float32 &msg) {
  m_pitch_ = msg.data;
}

void GliderBsdInterfaceNode::yawRefCallback(const std_msgs::Float64 &msg) {
  yaw_ref_ = msg.data;
}

void GliderBsdInterfaceNode::surgeRefCallback(const std_msgs::Float64 &msg) {
  surge_ref_ = msg.data;
}

// @.@ Member helper function to set up subscribers
void GliderBsdInterfaceNode::initializeSubscribers() {
  ROS_INFO("Initializing Subscribers for GliderBsdInterfaceNode");

  // data measured by the glider (m_*)

  m_est_lat_sub_ = nh_private_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, 
    "topics/subscribers/m_est_lat", "dummy"),
    10, &GliderBsdInterfaceNode::mEstLatCallback, this);
  
  m_est_lon_sub_ = nh_private_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, 
    "topics/subscribers/m_est_lon", "dummy"),
    10, &GliderBsdInterfaceNode::mEstLonCallback, this);

  m_gps_lat_sub_ = nh_private_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, 
    "topics/subscribers/m_gps_lat", "dummy"),
    10, &GliderBsdInterfaceNode::mGpsLatCallback, this);
  
  m_gps_lon_sub_ = nh_private_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, 
    "topics/subscribers/m_gps_lon", "dummy"),
    10, &GliderBsdInterfaceNode::mGpsLonCallback, this);
  
  m_depth_sub_ = nh_private_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, 
    "topics/subscribers/m_depth", "dummy"),
    10, &GliderBsdInterfaceNode::mDepthCallback, this);

  m_pitch_sub_ = nh_private_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, 
    "topics/subscribers/m_pitch", "dummy"),
    10, &GliderBsdInterfaceNode::mPitchCallback, this);

  // control references

  yaw_ref_sub_ = nh_private_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, 
    "topics/subscribers/yaw_ref", "dummy"),
    10, &GliderBsdInterfaceNode::yawRefCallback, this);

  surge_ref_sub_ = nh_private_.subscribe(FarolGimmicks::getParameters<std::string>(nh_private_, 
    "topics/subscribers/surge_ref", "dummy"),
    10, &GliderBsdInterfaceNode::surgeRefCallback, this);
}


// @.@ Member helper function to set up publishers
void GliderBsdInterfaceNode::initializePublishers() {
  ROS_INFO("Initializing Publishers for GliderBsdInterfaceNode");

  // measurement position for navigation filter
  position_pub_ = nh_private_.advertise<dsor_msgs::Measurement>(
      FarolGimmicks::getParameters<std::string>(
          nh_private_, "topics/publishers/position", "dummy"), 1);

}


// @.@ Member helper function to set up services
void GliderBsdInterfaceNode::initializeServices() {
  ROS_INFO("Initializing Services for GliderBsdInterfaceNode");

}


// @.@ Member helper function to set up the timer
void GliderBsdInterfaceNode::initializeTimer() {
  timer_ =nh_.createTimer(ros::Duration(1.0/p_node_frequency_), &GliderBsdInterfaceNode::timerIterCallback, this);
}

// @.@ Where the magic should happen.
void GliderBsdInterfaceNode::timerIterCallback(const ros::TimerEvent &event) {

  // if both latitude and longitude have been received, publish a position measurement
  if (est_lat_received_ && est_lon_received_) {
    glider_bsd_interface_alg_->publishPosition(m_est_lat_, m_est_lon_, position_pub_, vehicle_name_);
    
    est_lat_received_ = false;
    est_lon_received_ = false;
  }

}

/*
  @.@ Main
*/
int main(int argc, char** argv)
{
  // +.+ ROS set-ups:
  ros::init(argc, argv, "glider_bsd_interface_node"); //node name
  
  // +.+ node handle
  ros::NodeHandle nh;

  // +.+ private node handle
  ros::NodeHandle nh_private("~");

  ROS_INFO("main: instantiating an object of type GliderBsdInterfaceNode");

  // +.+ instantiate an GliderBsdInterfaceNode class object and pass in pointers to nodehandle public and private for constructor to use
  GliderBsdInterfaceNode glider_bsd_interface(&nh,&nh_private);

  // +.+  Going into spin; let the callbacks do all the magic
  ros::spin();

  return 0;
}
