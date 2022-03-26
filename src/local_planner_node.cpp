#include "ros/ros.h"
#include "std_msgs/String.h"

#include <thread>
#include <string>
#include <mutex>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <teb_local_planner/teb_local_planner_ros.h>
#include <vector>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#pragma GCC diagnostic pop

#include "local_planner_node/TrajectoryFollowCue.h"
#include "local_planner_node/PlanReq.h"
#include "local_planner_node/LocalPlannerDiagnostics.h"

#define RATE (1)

using namespace teb_local_planner;

ros::NodeHandle* node;
TebLocalPlannerROS * planner;
tf2_ros::Buffer * tfBuffer;
tf2_ros::TransformListener * tfListener;
costmap_2d::Costmap2DROS * costmap;

std::vector<std::string> points;

static bool plan_active = false;

bool add_plan_request( local_planner_node::PlanReq::Request& req,
                       local_planner_node::PlanReq::Response& resp)
{
    static ros::Publisher via_pub = node->advertise<nav_msgs::Path>("/local_planner_node/via_points", 1);
    
    (void)req;
    (void)resp;

    for (size_t i = 0; i < req.plan.size(); i++)
    {
        points.push_back (req.plan[i].header.frame_id);
    }

    std::vector<geometry_msgs::PoseStamped> prevPlan = req.plan;
    std::vector<geometry_msgs::PoseStamped> lastPoint;
    if (prevPlan.size() > 0)
    {
        lastPoint.push_back(req.plan[req.plan.size() - 1]);
        prevPlan.pop_back();
    }

    if (prevPlan.size() > 0)
    {
        nav_msgs::Path viaPoints;
        viaPoints.header.frame_id = "odom";
        for (geometry_msgs::PoseStamped& p : prevPlan)
        {
            tf2::Stamped<tf2::Transform> point_to_first_point;
            tf2::convert(tfBuffer->lookupTransform("odom", p.header.frame_id, ros::Time(0)), point_to_first_point);
            ROS_INFO("Pose frame: %s", p.header.frame_id.c_str());

            p.header.frame_id = "odom";
            p.pose.position.x = point_to_first_point.getOrigin().getX();
            p.pose.position.y = point_to_first_point.getOrigin().getY();
            p.pose.position.z = point_to_first_point.getOrigin().getZ();

            //TODO: Fix orientation
            viaPoints.poses.push_back(p);
        }
        via_pub.publish(viaPoints);
    }
    planner->setPlan(lastPoint);
    plan_active = true;

    return true;
}

void publish_local_planner_diagnostics ()
{
    static ros::Publisher diagnostics_publisher = node->advertise<local_planner_node::LocalPlannerDiagnostics>("/LocalPlannerNodeDiagnostics", 1);
    local_planner_node::LocalPlannerDiagnostics diagnostics;
    for(std::vector<std::string>::iterator i = points.begin(); i != points.end(); i++)
    {
        diagnostics.points.push_back(*i);
    }
    diagnostics.traj_active = plan_active;
    diagnostics_publisher.publish(diagnostics);
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

        publish_local_planner_diagnostics();

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
