#include "ros/ros.h"
#include "std_msgs/String.h"

#include <thread>
#include <string>
#include <mutex>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <teb_local_planner/teb_local_planner_ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#pragma GCC diagnostic pop

#include "local_planner_node/TrajectoryFollowCue.h"
#include "local_planner_node/PlanReq.h"

#define RATE (1)

using namespace teb_local_planner;

ros::NodeHandle* node;
TebLocalPlannerROS * planner;
tf2_ros::Buffer * tfBuffer;
tf2_ros::TransformListener * tfListener;
costmap_2d::Costmap2DROS * costmap;

bool plan_active = false;

void send_initial_plan()
{
    geometry_msgs::PoseStamped target;
    std::vector<geometry_msgs::PoseStamped> plan;
    tf2::Quaternion Up;
    Up.setRPY(0,0,0);
    target.pose.orientation.w = Up.getW();
    target.pose.orientation.x = Up.getX();
    target.pose.orientation.y = Up.getY();
    target.pose.orientation.z = Up.getZ();
    target.pose.position.x = 1.5;
    target.pose.position.y = 1.5;
    target.pose.position.z = 0;

    target.header.stamp = ros::Time::now();
    target.header.frame_id = "odom";

    plan.push_back(target);
    planner->setPlan(plan);
    plan_active = true;
}

bool add_plan_request( local_planner_node::PlanReq::Request& req,
                       local_planner_node::PlanReq::Response& resp)
{
    (void)req;
    (void)resp;

    std::cout << "Got request : " << req << "\n";

    std_msgs::Header HEADER;
    HEADER.frame_id = "base_link";
    HEADER.stamp = ros::Time::now();

    for( int i = 0; (size_t)i < req.plan.size(); i++ )
    {
        req.plan[i].header = HEADER;
    }

    planner->setPlan(req.plan);
    plan_active = true;

    std::cout << "\n\nPLANE REQUIEST\n";
    return true;
}

void planner_loop(void)
{
    while(ros::ok())
    {
        local_planner_node::TrajectoryFollowCue active_trajectory;
        geometry_msgs::Twist result;
        if(plan_active)
        {
            planner->computeVelocityCommands(result);
            if(planner->isGoalReached())
            {
                ROS_INFO("Goal Reached");
                plan_active = false;
            }
            active_trajectory.velocity.linear.x = result.linear.x;
            active_trajectory.velocity.linear.y = result.linear.y;
            active_trajectory.velocity.linear.z = result.linear.z;

            active_trajectory.velocity.angular.x = result.angular.x;
            active_trajectory.velocity.angular.y = result.angular.y;
            active_trajectory.velocity.angular.z = result.angular.z;

            active_trajectory.traj_follow_active = true;
        }
        else
        {
            active_trajectory.traj_follow_active = false;
        }

        static ros::Publisher active_traj_publisher = node->advertise<local_planner_node::TrajectoryFollowCue>("/active_trajectory", 1);
        active_traj_publisher.publish(active_trajectory);
    }
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "local_planner_node");

	ros::NodeHandle n;
	node = &n;

    ros::ServiceServer service_plan_req = node->advertiseService("local_plan_request", add_plan_request);

    tfBuffer = new tf2_ros::Buffer();

    planner = new TebLocalPlannerROS();

    tfListener = new tf2_ros::TransformListener(*tfBuffer);
    std::string err;
    tfBuffer->canTransform("base_link", "map", ros::Time(0), ros::Duration(3.0), &err );

    costmap = new costmap_2d::Costmap2DROS("costmap", *tfBuffer);

    planner->initialize("", tfBuffer, costmap);
    std::thread planner_thread(planner_loop);
    ros::spin();
    planner_thread.join();

	return 0;
}
