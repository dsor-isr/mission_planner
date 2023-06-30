/*
Developers: DSOR Team -> @isr.ist.pt Instituto Superior Tecnico
Description: Please check the documentation of this package for more info.
*/
// this header incorporates all the necessary #include files and defines the
// class "LeakDsNode"
#include "LeakNode.h"

/*
 * Parallel port pins and bits of the water leak detectors.
 *
 * DB25    Leaks   Parallel port
 * ----------------------------------
 * 9       VCC     Output data bit 7
 * 13      Hull    Input status bit 4
 * 18-25   Ground  Ground
 */

/*
#######################################################################################################################
@.@ CONSTRUCTOR: put all dirty work of initializations here
Note the odd syntax: have to pass nodehandle pointer into constructor for
constructor to build subscribers, etc
#######################################################################################################################
*/
LeakNode::LeakNode(ros::NodeHandle *nodehandle,
                   ros::NodeHandle *nodehandle_private)
    : nh_(*nodehandle), nh_p_(*nodehandle_private) {

  ROS_INFO("in class constructor of LeakNode");
  initializeSubscribers();
  loadParams();
  initializePublishers();

  // Change first letter to uppercase
  p_leak_name_[0][0] = toupper(p_leak_name_[0][0]);
  p_leak_name_[1][0] = toupper(p_leak_name_[1][0]);

  // Set variable to stop thrusters
  int_stop_.data = 1;
}

/*
#######################################################################################################################
@.@ Destructor
#######################################################################################################################
*/
LeakNode::~LeakNode() {

  // +.+ shutdown publishers
  leak1_pub_.shutdown();
  leak2_pub_.shutdown();
  stop_pub_.shutdown();
  diag_pub_.shutdown();

  // +.+ shutdown node
  nh_.shutdown();
  nh_p_.shutdown();
}

/*
#######################################################################################################################
@.@ Member Helper function to set up subscribers;
note odd syntax: &LeakDsNode::subscriberCallback is a pointer to a member
function of LeakDsNode "this" keyword is required, to refer to the current
instance of LeakDsNode
#######################################################################################################################
*/
void LeakNode::initializeSubscribers() {

  ROS_INFO("Initializing Subscribers for LeakNode");
  // ---> add subscribers here
  parallel_port_sub_ =
      nh_.subscribe(FarolGimmicks::getParameters<std::string>(nh_p_, "topics/subscribers/data", "/parallel_port/data"), 1, &LeakNode::ppCallback, this);
}

/*
#######################################################################################################################
@.@ Member helper function to set up publishers;
#######################################################################################################################
*/
void LeakNode::initializePublishers() {

  ROS_INFO("Initializing Publishers for LeakNode"); // ---> add publishers here
  // Example: uref_pub = nh_.advertise<std_msgs::Float64>("URef", 10); //Surge
  // Reference
  leak1_pub_ = nh_p_.advertise<std_msgs::Bool>(p_leak_name_[0], 10);
  leak2_pub_ = nh_p_.advertise<std_msgs::Bool>(p_leak_name_[1], 10);
  stop_pub_ = nh_.advertise<std_msgs::Int8>(FarolGimmicks::getParameters<std::string>(nh_p_, "topics/publishers/thruster_stop", "/Thruster_Stop"), 10);
  diag_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>(FarolGimmicks::getParameters<std::string>(nh_p_, "topics/publishers/diagnostics", "/diagnostics"), 100);
}

/*
#######################################################################################################################
@.@ Set frequency of the node default is 2
#######################################################################################################################
*/
double LeakNode::nodeFrequency() {
  double node_frequency;
  nh_.param("node_frequency", node_frequency, 2.0);
  ROS_INFO("Node will run at : %lf [hz]", node_frequency);
  return node_frequency;
}

/*
#######################################################################################################################
@.@ Load the parameters
#######################################################################################################################
*/
void LeakNode::loadParams() {

  ROS_INFO("Load the LeakDsNode parameters");
  //---> params here, always p_paramName

  // Get parameters
  p_leak_name_ =
      FarolGimmicks::getParameters<std::vector<std::string>>(nh_p_, "names");
  p_leak_addr_ =
      FarolGimmicks::getParameters<std::vector<int>>(nh_p_, "address");

  if (p_leak_name_.size() != p_leak_addr_.size()) {
    ROS_ERROR("Number of names is different from number of addresses");
    ros::shutdown();
  }
}

/*
#######################################################################################################################
@.@ Report the sensor state.
#######################################################################################################################
*/
void LeakNode::reporting(const int status1, const int status2) {

  diagnostic_msgs::DiagnosticArray diag_msg;
  diag_msg.header.stamp = ros::Time::now();
  diag_msg.status.push_back(GliderDiagnostics::setDiagnosisMsg(
      diagnostic_msgs::DiagnosticStatus::OK, "/Sensors/Leaks", "Leaks Good.",
      "DSOR Leaks"));

  if (status1)
    GliderDiagnostics::errorLevel(
        &diag_msg, "Leaks " + p_leak_name_[0] + ": Leak Detected in sensor", 0);

  if (status2)
    GliderDiagnostics::errorLevel(
        &diag_msg, "Leaks " + p_leak_name_[1] + ": Leak Detected in sensor", 0);

  // Generate key values for both leaks
  GliderDiagnostics::addKeyValue(&diag_msg, p_leak_name_[0],
                                 boost::str(boost::format("%1%") % (status1)),
                                 0);
  GliderDiagnostics::addKeyValue(&diag_msg, p_leak_name_[1],
                                 boost::str(boost::format("%1%") % (status2)),
                                 0);

  diag_pub_.publish(diag_msg);
}

/*
#######################################################################################################################
 @.@ Check if there is a leak.
 #######################################################################################################################
 */
bool LeakNode::leak(const unsigned int addr, const unsigned int data) {
  if (addr & data)
    return true;

  return false;
}

/*
#######################################################################################################################
@.@ Callbacks Section / Methods
#######################################################################################################################
*/

/*
#######################################################################################################################
@.@ Function to handle parallel port data.
#######################################################################################################################
*/
void LeakNode::ppCallback(const std_msgs::UInt8 &msg) {
  std_msgs::Bool aux;

  // Leak 1
  bool status_leak1 = leak(p_leak_addr_[0], msg.data);
  if (status_leak1) {
    ROS_ERROR_STREAM(p_leak_name_[0] + " Leak Detected - Stopping Thrusters");
    stop_pub_.publish(int_stop_);
  }
  aux.data = status_leak1;
  leak1_pub_.publish(aux);

  // Leak 2
  bool status_leak2 = leak(p_leak_addr_[1], msg.data);
  if (status_leak2) {
    ROS_ERROR_STREAM(p_leak_name_[1] + " Leak Detected - Stopping Thrusters");
    stop_pub_.publish(int_stop_);
  }
  aux.data = status_leak2;
  leak2_pub_.publish(aux);

  // Reporting
  reporting(status_leak1, status_leak2);
}

/*
#######################################################################################################################
@.@ Main
#######################################################################################################################
*/
int main(int argc, char **argv) {
  // +.+ ROS set-ups:
  ros::init(argc, argv, "leak_node"); // node name
  // +.+ create a node handle; need to pass this to the class constructor
  ros::NodeHandle nh;
  ros::NodeHandle nh_p("~");

  ROS_INFO("main: instantiating an object of type LeakNode");

  // +.+ instantiate an LeakDsNode class object and pass in pointer to
  // nodehandle for constructor to use
  LeakNode leak_(&nh, &nh_p);

  // +.+ Added to work with timer -> going into spin; let the callbacks do all
  // the work
  ros::spin();

  return 0;
}
