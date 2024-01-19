#include "GliderBsdInterfaceAlgorithm.h"

// @.@ Constructor
GliderBsdInterfaceAlgorithm::GliderBsdInterfaceAlgorithm() {

}

// @.@ Destructor
GliderBsdInterfaceAlgorithm::~GliderBsdInterfaceAlgorithm() {

}

// publish position after converting lat/lon to UTM
void GliderBsdInterfaceAlgorithm::publishPosition(double m_lat, double m_lon, ros::Publisher position_pub, std::string vehicle_name) {
  // fill header
  measurement_position_.header.stamp = ros::Time::now();
  measurement_position_.header.frame_id = vehicle_name + "_gnss";

  // compute earthing andf easting from lat and lon
  bool northp;
  int zone;
  double northing, easting, gamma, k;

  // COPIED FROM Gnss2Utm.cpp (adapted)
  try {
    GeographicLib::UTMUPS::Forward(m_lat, m_lon, zone, northp, easting, northing, gamma, k);
  }
  catch (const GeographicLib::GeographicErr::exception &ex)
  {
    ROS_WARN("Gnss2Utm caught exception: %s", ex.what());
    return;
  }

  measurement_position_.value = {northing, easting};

  // set noise to 0
  measurement_position_.noise = {0.0, 0.0};

  // publish
  position_pub.publish(measurement_position_);
}

// publish depth
void GliderBsdInterfaceAlgorithm::publishDepth(double m_depth, ros::Publisher position_pub, std::string vehicle_name) {
  // fill header
  measurement_depth_.header.stamp = ros::Time::now();
  measurement_depth_.header.frame_id = vehicle_name + "_depth";

  measurement_depth_.value = {m_depth};

  // set noise to 0
  measurement_depth_.noise = {0.0};

  // publish
  position_pub.publish(measurement_depth_);
}

// publish depth
void GliderBsdInterfaceAlgorithm::publishOrientation(double m_roll, double m_pitch, double m_heading, ros::Publisher orientation_pub, std::string vehicle_name) {
  // fill header
  measurement_orientation_.header.stamp = ros::Time::now();
  measurement_orientation_.header.frame_id = vehicle_name + "_ahrs";

  measurement_orientation_.value = {m_roll, m_pitch, m_heading,
                                    0.0, 0.0, 0.0}; // should be NAN, but filter does not like it

  // set noise to 0.001
  measurement_orientation_.noise = {0.001, 0.001, 0.001,
                                    0.001, 0.001, 0.001};

  // publish
  orientation_pub.publish(measurement_orientation_);
}

// call service to change heading reference
void GliderBsdInterfaceAlgorithm::callHeadingService(double yaw_ref, ros::ServiceClient set_heading_ref_client) {
  
  /* Call the service to set heading reference */
  slocum_glider_msgs::SetFloat32 srv;
  srv.request.data = yaw_ref* FarolGimmicks::PI / 180;
  set_heading_ref_client.call(srv);
}