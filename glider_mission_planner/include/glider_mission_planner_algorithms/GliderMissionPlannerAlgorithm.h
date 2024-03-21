/* 
 * Developers: DSOR Team -> @tecnico.ulisboa.pt Instituto Superior Tecnico 
 */
#ifndef CATKIN_WS_GLIDERMISSIONPLANNERALGORITHM_H
#define CATKIN_WS_GLIDERMISSIONPLANNERALGORITHM_H

#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <ros/ros.h>

class GliderMissionPlannerAlgorithm {
  public:
   
    /* -------------------------------------------------------------------------*/
    /**
    * @brief Constructor
    *
    * @Param nodehandle
    * @Param nodehandle_private
    */
    /* -------------------------------------------------------------------------*/
    GliderMissionPlannerAlgorithm();

    /* -------------------------------------------------------------------------*/
    /**
    * @brief  Destructor
    */
    /* -------------------------------------------------------------------------*/
    ~GliderMissionPlannerAlgorithm();

    void startNewMission(double north_min, double north_max, double east_min, double east_max,
                         int path_orientation, std::vector<double> vehicle_pos,
                         double min_turn_radius, double resolution, 
                         std::string path_type, double velocity);

  private:

    int getInitialArcDirection(std::vector<double> initial_path_pos, double north_min, 
                               double north_max, double east_min, double east_max,
                               int path_orientation);
    
    int getProgressionSign(std::vector<double> initial_path_pos, double north_min, 
                           double north_max, double east_min, double east_max,
                           int path_orientation);

    double updateCurrentRadius(int initial_line_direction, int line_direction, double normal_radius, 
                               double big_radius, std::string path_type);

    int getInitialLineDirection(std::vector<double> initial_path_pos, double north_min, 
                                double north_max, double east_min, double east_max,
                                int path_orientation);

    bool insideInterestZone(std::vector<double> path_pos, double north_min, double north_max, 
                            double east_min, double east_max);

    double updateLineLength(double north_min, double north_max, double east_min, double east_max, 
                            int initial_line_direction, int line_direction, 
                            double normal_radius, double big_radius,
                            bool is_first_line, bool is_last_line, int path_orientation);

    std::string getPathSections(double north_min, double north_max, double east_min, double east_max,
                                int path_orientation, std::vector<double> vehicle_pos,
                                double min_turn_radius, double resolution, 
                                std::string path_type, double velocity);

    std::string getNewMissionString(double north_min, double north_max, double east_min, double east_max,
                                    int path_orientation, std::vector<double> vehicle_pos,
                                    double min_turn_radius, double resolution,
                                    std::string path_type, double velocity);

    std::string mission_start_;
    std::string mission_string_;

};
#endif
