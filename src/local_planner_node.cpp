#include "ros/ros.h"
#include "std_msgs/String.h"

#include <thread>
#include <string>
#include <mutex>

#include "teb_local_planner/FeedbackMsg.h"

ros::NodeHandle* node;
#define RATE (10)

int main(int argc, char **argv)
{
	ros::init(argc, argv, "local_planner_node");

	ros::NodeHandle n;
	node = &n;
    ros::Rate rate(RATE);

    ros::Publisher output_pub =
        node->advertise<teb_local_planner::FeedbackMsg>("/teb_feedback", 1);


    while( ros::ok() )
    {

        teb_local_planner::FeedbackMsg output_traj;

        teb_local_planner::TrajectoryMsg traj;
        teb_local_planner::TrajectoryPointMsg traj_point;

        traj_point.velocity.linear.x = 10;
        traj_point.time_from_start = ros::Duration(1.0);
        traj.trajectory.push_back( traj_point );

        traj_point.velocity.linear.x = 10;
        traj_point.velocity.linear.y = 10;
        traj_point.time_from_start = ros::Duration(2.0);
        traj.trajectory.push_back( traj_point );

        output_traj.trajectories.push_back( traj );

        output_pub.publish(output_traj);

        ros::spinOnce();
        rate.sleep();
    }


	return 0;
}
