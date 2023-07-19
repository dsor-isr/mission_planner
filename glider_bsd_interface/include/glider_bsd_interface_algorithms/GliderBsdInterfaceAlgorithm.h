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

  private:
    
    // position message
    dsor_msgs::Measurement measurement_position_;

    // depth message
    dsor_msgs::Measurement measurement_depth_;
  

};
#endif
