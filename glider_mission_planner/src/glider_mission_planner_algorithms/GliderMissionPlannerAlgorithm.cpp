#include "GliderMissionPlannerAlgorithm.h"

// @.@ Constructor
GliderMissionPlannerAlgorithm::GliderMissionPlannerAlgorithm() {
  mission_start_ = "3\n";

}

// @.@ Destructor
GliderMissionPlannerAlgorithm::~GliderMissionPlannerAlgorithm() {

}

bool GliderMissionPlannerAlgorithm::insideInterestZone(std::vector<double> path_pos, double north_min, double north_max, 
                                                       double east_min, double east_max) {
  if (path_pos[0] >= east_min && path_pos[0] <= east_max && path_pos[1] >= north_min && path_pos[1] <= north_max) {
    return true;
  }

  return false;
}

int GliderMissionPlannerAlgorithm::getInitialArcDirection(std::vector<double> initial_path_pos, double north_min, 
                                                          double north_max, double east_min, double east_max,
                                                          int path_orientation) {
  // determine what's the arc direction for the first turn

  // if initial position is the bottom left or top right corner
  if ((initial_path_pos[0] == east_min && initial_path_pos[1] == north_min) || 
      (initial_path_pos[0] == east_max && initial_path_pos[1] == north_max)) {
    return (path_orientation == 0) ? 1 : -1;
  }
  // else: top left or bottom right corner
  return (path_orientation == 0) ? -1 : 1;
}

int GliderMissionPlannerAlgorithm::getInitialLineDirection(std::vector<double> initial_path_pos, double north_min, 
                                                           double north_max, double east_min, double east_max,
                                                           int path_orientation) {
  // determine what's the line direction initially
  if ((initial_path_pos[0] == east_min && initial_path_pos[1] == north_min) ||
      ((initial_path_pos[0] == east_min && initial_path_pos[1] == north_max) && path_orientation == 0) ||
      ((initial_path_pos[0] == east_max && initial_path_pos[1] == north_min) && path_orientation == 1)) {
    return 1;
  } else {
    return -1;
  }
}

int GliderMissionPlannerAlgorithm::getProgressionSign(std::vector<double> initial_path_pos, double north_min, 
                                                      double north_max, double east_min, double east_max,
                                                      int path_orientation) {
  // determine the sign of progression along the zone of intereset
  if (path_orientation == 0) { // E-W
    return (initial_path_pos[1] == north_min) ? 1 : -1;
  } else { // N-S
    return (initial_path_pos[0] == east_min) ? 1 : -1;
  }
}

double GliderMissionPlannerAlgorithm::updateCurrentRadius(int initial_line_direction, int line_direction, double normal_radius, 
                                                          double big_radius, std::string path_type) {
  if (path_type == "lawnmower_normal") { // radius is always the min_turn_radius
    return normal_radius;
  } else if (path_type == "lawnmower_encircling") { // big radius in one leg direction, normal radius in reverse direction
    return (line_direction == initial_line_direction) ? big_radius : normal_radius;
  } else { // default
    return normal_radius;
  }
}

double GliderMissionPlannerAlgorithm::updateLineLength(double north_min, double north_max, double east_min, double east_max, 
                                                       int initial_line_direction, int line_direction, 
                                                       double normal_radius, double big_radius,
                                                       bool is_first_line, bool is_last_line, int path_orientation) {
  double line_length = (path_orientation == 0) ? (east_max - east_min) // E-W
                                               : (north_max - north_min); // N-S
  
  if (is_first_line) { // FIRST LINE
    return line_length - big_radius;
  } else if (is_last_line) { // LAST LINE
    return (line_direction == initial_line_direction) ? line_length - normal_radius : line_length - big_radius;
  } else { // all other lines
    return line_length - big_radius - normal_radius;
  }
}

std::string GliderMissionPlannerAlgorithm::getPathSections(double north_min, double north_max, double east_min, double east_max,
                                                           int path_orientation, std::vector<double> vehicle_pos,
                                                           double min_turn_radius, double resolution,
                                                           std::string path_type, double velocity) {
  // path sections string
  std::string path_sections_string = "";

  // current type of section
  std::string curr_section = "LINE";

  // is first line?
  bool is_first_line = true;

  // is last line?
  bool is_last_line = false;
  
  // current intersection, get closest corner to vehicle
  std::vector<double> path_pos = {0.0, 0.0};
  path_pos[0] = (abs(vehicle_pos[0] - east_min) < abs(vehicle_pos[0] - east_max)) ? east_min : east_max;
  path_pos[1] = (abs(vehicle_pos[1] - north_min) < abs(vehicle_pos[1] - north_max)) ? north_min : north_max;

  // sign of progression along the zone of interest (positive or negative, across the easting or northing axis)
  int progression_sign = getProgressionSign(path_pos, north_min, north_max, east_min, east_max, path_orientation);

  // positive or negative direction for the line
  int initial_line_direction = getInitialLineDirection(path_pos, north_min, north_max, east_min, east_max, path_orientation);
  int line_direction = initial_line_direction;

  // current arc direction (left = 1 or right = -1)
  int current_adirection = getInitialArcDirection(path_pos, north_min, north_max, east_min, east_max, path_orientation);

  // compute normal and big radii
  double normal_radius = min_turn_radius;
  double big_radius = (path_type == "lawnmower_encircling") ? normal_radius + resolution/2 : normal_radius;

  // current radius and line_length
  double curr_radius = 0;
  double line_length = 0;

  while (insideInterestZone(path_pos, north_min, north_max, east_min, east_max) || curr_section == "LINE") {
    if (curr_section == "LINE") {
      // add line -> # LINE xInit yInit xEnd yEnd velocity <nVehicle> <gamma> <user data>

      curr_radius = updateCurrentRadius(initial_line_direction, line_direction, normal_radius, big_radius, path_type);

      line_length = updateLineLength(north_min, north_max, east_min, east_max, 
                                     initial_line_direction, line_direction, 
                                     normal_radius, big_radius,
                                     is_first_line, is_last_line, path_orientation);

      if (path_orientation == 0) { // E-W lawnmower
        path_sections_string += "LINE " + std::to_string(path_pos[0]) + " " + std::to_string(path_pos[1]) + " " 
                                        + std::to_string(path_pos[0] + line_direction * line_length) + " " + std::to_string(path_pos[1]) + " "
                                        + std::to_string(velocity) + " -1\n";

        // update current path position
        path_pos[0] += line_direction * line_length;

      } else { // N-S lawnmower
        path_sections_string += "LINE " + std::to_string(path_pos[0]) + " " + std::to_string(path_pos[1]) + " " 
                                        + std::to_string(path_pos[0]) + " " + std::to_string(path_pos[1] + line_direction * line_length) + " "
                                        + std::to_string(velocity) + " -1\n";

        // update current path position
        path_pos[1] += line_direction * line_length;
      }

      // we are no longer in the first line
      is_first_line = false;

      // update line direction
      line_direction = (line_direction == 1) ? -1 : 1;

    } else {
      // add arc -> # ARC xInit yInit xCenter yCenter xEnd yEnd velocity adirection radius <nVehicle> <gamma> <user data>
      if (path_orientation == 0) { // E-W lawnmower
        path_sections_string += "ARC " + std::to_string(path_pos[0]) + " " + std::to_string(path_pos[1]) + " "
                                       + std::to_string(path_pos[0]) + " " + std::to_string(path_pos[1] + progression_sign * curr_radius) + " "
                                       + std::to_string(path_pos[0]) + " " + std::to_string(path_pos[1] + progression_sign * 2 * curr_radius) + " "
                                       + std::to_string(velocity) + " " + std::to_string(current_adirection) + " "
                                       + std::to_string(curr_radius) + " -1\n";

        // update current path position
        path_pos[1] += progression_sign * 2 * curr_radius;

      } else { // N-S lawnmower
        path_sections_string += "ARC " + std::to_string(path_pos[0]) + " " + std::to_string(path_pos[1]) + " "
                                       + std::to_string(path_pos[0] + progression_sign * curr_radius) + " " + std::to_string(path_pos[1]) + " "
                                       + std::to_string(path_pos[0] + progression_sign * 2 * curr_radius) + " " + std::to_string(path_pos[1]) + " "
                                       + std::to_string(velocity) + " " + std::to_string(current_adirection) + " "
                                       + std::to_string(curr_radius) + " -1\n";

        // update current path position
        path_pos[0] += progression_sign * 2 * curr_radius;

      }

      // update next arc direction if last section was an arc and path type is lawnmower_normal
      if (path_type == "lawnmower_normal") {
        current_adirection *= -1;
      } else if (path_type == "lawnmower_encircling") { // don't change adirection, but change progression sign
        progression_sign *= -1;
      }

      // if after this arc the curent path position is outside the interest zone, then the next line is the last one
      if (!insideInterestZone(path_pos, north_min, north_max, east_min, east_max)) {
        is_last_line = true;
      }

    }
    
    // update current section for next iteration
    curr_section = (curr_section == "LINE") ? "ARC" : "LINE";
  }

  return path_sections_string;
}

std::string GliderMissionPlannerAlgorithm::getNewMissionString(double north_min, double north_max, double east_min, double east_max,
                                                               int path_orientation, std::vector<double> vehicle_pos,
                                                               double min_turn_radius, double resolution, 
                                                               std::string path_type, double velocity) {
  // create new string
  std::string mission = mission_start_;

  // add mission reference point (bottom left corner of the zone of interest)
  mission += std::to_string(east_min) + " " + std::to_string(north_min) + "\n";

  // move vehicle position to zone of interest's frame
  vehicle_pos[0] -= east_min;
  vehicle_pos[1] -= north_min;

  // add path sections
  mission += getPathSections(0, north_max - north_min, 0, east_max - east_min,
                             path_orientation, vehicle_pos,
                             min_turn_radius, resolution, path_type, velocity);


  return mission;
}

void GliderMissionPlannerAlgorithm::startNewMission(double north_min, double north_max, double east_min, double east_max,
                                                    int path_orientation, std::vector<double> vehicle_pos,
                                                    double min_turn_radius, double resolution,
                                                    std::string path_type, double velocity) {
  // get mission string
  mission_string_ = getNewMissionString(north_min, north_max, east_min, east_max,
                                        path_orientation, vehicle_pos,
                                        min_turn_radius, resolution, 
                                        path_type, velocity);

  // call service to start new PF
  ROS_INFO("\nMISSION FILE:\n");
  std::cout << mission_string_;
}