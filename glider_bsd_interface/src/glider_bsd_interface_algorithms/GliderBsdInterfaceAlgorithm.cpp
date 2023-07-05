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
  measurement_.header.stamp = ros::Time::now();
  measurement_.header.frame_id = vehicle_name + "_gnss";

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

  measurement_.value = {northing, easting};

  // set noise to 0
  measurement_.noise = {0.0, 0.0};

  // publish
  position_pub.publish(measurement_);
}