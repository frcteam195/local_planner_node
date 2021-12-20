
#include "ros/ros.h"
#include <teb_local_planner/teb_local_planner_ros.h>

class LocalPlanner{
public:
    LocalPlanner(ros::NodeHandle* n);
    void step();

    ros::NodeHandle* node;
    teb_local_planner::TebConfig config;
    teb_local_planner::RobotFootprintModelPtr robot_model;
    std::vector<teb_local_planner::ObstaclePtr> obst_vector;
    teb_local_planner::TebVisualizationPtr visual;
    teb_local_planner::ViaPointContainer via_points;
    teb_local_planner::TebOptimalPlanner planner;
};
