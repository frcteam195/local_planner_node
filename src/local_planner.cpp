#include "local_planner.hpp"
#include <iostream>


LocalPlanner::LocalPlanner( ros::NodeHandle* n )
{
    node = n;
    config.loadRosParamFromNodeHandle(*node);
    robot_model = teb_local_planner::TebLocalPlannerROS::getRobotFootprintFromParamServer(*node);

    visual = teb_local_planner::TebVisualizationPtr(
        new teb_local_planner::TebVisualization(*node, config));

    planner = teb_local_planner::TebOptimalPlanner(config,
                                                   &obst_vector,
                                                   robot_model,
                                                   visual,
                                                   &via_points);


}

void LocalPlanner::step()
{
    planner.plan(teb_local_planner::PoseSE2(0,0,0),
                  teb_local_planner::PoseSE2(4,0,0));

    planner.visualize();
    visual->publishObstacles(obst_vector);
    visual->publishViaPoints(via_points);
    visual->publishFeedbackMessage(planner,
                                   obst_vector);

}
