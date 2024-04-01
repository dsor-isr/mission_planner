# Glider Mission Planner Node

## Diagram
<!-- ![glider_mission_planner Diagram](img/data_serializer.png) -->

## Subscribers

- /#vehicle#/nav/filter/state [auv_msgs/NavigationStatus]
- "/#vehicle#/acomms/interest_zone" [glider_mission_planner/mInterestZone]

## Publishers

- "/#vehicle#/addons/Mission_String"
- "/#vehicle#/mission_planner/mission_start_ack"

## Services

- /#vehicle#/mission_planner/change_configurations
- /#vehicle#/mission_planner/interest_zone

## Parameters

- min_turning_radius
- node_frequency
- path_orientation
- path_speed
- path_type
- resolution