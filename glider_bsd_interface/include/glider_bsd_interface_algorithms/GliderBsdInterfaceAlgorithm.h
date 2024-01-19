/* 
 * Developers: DSOR Team -> @tecnico.ulisboa.pt Instituto Superior Tecnico 
 */
#ifndef CATKIN_WS_GLIDERBSDINTERFACEALGORITHM_H
#define CATKIN_WS_GLIDERBSDINTERFACEALGORITHM_H

#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <dsor_msgs/Measurement.h>
#include <ros/ros.h>

#include <GeographicLib/UTMUPS.hpp>
#include <farol_gimmicks_library/FarolGimmicks.h>

/* Required to call the path services*/
#include "slocum_glider_msgs/SetFloat32.h"


class GliderBsdInterfaceAlgorithm {
  public:
   
    /* -------------------------------------------------------------------------*/
    /**
    * @brief Constructor
    *
    * @Param nodehandle
    * @Param nodehandle_private
    */
    /* -------------------------------------------------------------------------*/
    GliderBsdInterfaceAlgorithm();

    /* -------------------------------------------------------------------------*/
    /**
    * @brief  Destructor
    */
    /* -------------------------------------------------------------------------*/
    ~GliderBsdInterfaceAlgorithm();

    void publishPosition(double m_lat, double m_lon, ros::Publisher position_pub, std::string vehicle_name);

    void publishDepth(double m_depth, ros::Publisher position_pub, std::string vehicle_name);

    void publishOrientation(double m_roll, double m_pitch, double m_heading, ros::Publisher orientation_pub, std::string vehicle_name);

    void callHeadingService(double yaw_ref, ros::ServiceClient set_heading_ref_client);

  private:
    
    // position message
    dsor_msgs::Measurement measurement_position_;

    // depth message
    dsor_msgs::Measurement measurement_depth_;

    // orientation message
    dsor_msgs::Measurement measurement_orientation_;
  

};
#endif
