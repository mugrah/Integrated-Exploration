/************************************
*           TODO LIST
*
*  - set_new_goal
*    - adapt information function
*    - adapt distance function
*
*  - define detection procedure
*
*  - define map exchange method
*
************************************/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <nav_msgs/GetMap.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Quaternion.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/console.h>
#include <fstream>
#include <iostream>
#include <cstdlib>
#include <string>
#include <vector>
#include <stdio.h>

#include "pioneer3at/OccMap.h"
#include "utils.h"

#include "save_map.cc"

#include "utils.cc"

#include "goal_tools.cc"


#define NEW_MAP 0
#define SET_NEW_GOAL 1
#define GOAL_SET 2



ros::Publisher goal_pub;
ros::Publisher cmd_vel_pub;

ros::Subscriber map_sub;
ros::Subscriber goal_status_sub;
ros::Subscriber pose_sub;
ros::Subscriber cmd_vel_sub;
ros::Subscriber laser_sub;

std::string robot_topic;
std::string goal_topic;
std::string map_topic;
std::string occ_map_topic;
std::string pose_topic;
// std::string cmd_vel_topic;
std::string base_link_topic;
// std::string goal_status_topic;
// std::string laser_topic;

double alpha;
double beta;
double gama;
double a_beta;
double b_beta;

pioneer3at::OccMap r_map;
nav_msgs::Odometry r_pose;
mapPose m_pose;
odomPose o_pose;
mapPose m_goal;
geometry_msgs::PoseStamped r_goal;

int flowStatus;

int map_pose_count = 0;

/*****************************************
*                                        *
*               CALLBACKS                *
*                                        *
*****************************************/


void ros_pose_CallBack(nav_msgs::Odometry pose)
{
    if(robot_topic.compare("/robot_0") == 0){
        tf::StampedTransform transform;
        tf::TransformListener listener;

        try{
            listener.waitForTransform(map_topic, base_link_topic, ros::Time(0), ros::Duration(5.0));
            listener.lookupTransform(map_topic, base_link_topic, ros::Time(0), transform);
        }catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
        }

        r_pose = pose;

        m_pose = odom2map(&r_pose, &r_map, &transform, robot_topic);

        // saveMapPose(m_pose, robot_topic);
        // saveOdomPose(o_pose, robot_topic);

        if(flowStatus == SET_NEW_GOAL){
            // ROS_ERROR_STREAM("PUBLISH NEW GOAL");

            save_map_pose(r_map, m_pose, robot_topic, map_pose_count++);
            r_goal = setNewGoal(&r_map, pose, m_pose, &transform, a_beta, b_beta, alpha, beta, gama);
            
            ROS_ERROR_STREAM("before publishing - " << r_goal.header.frame_id);

            goal_pub.publish(r_goal);

            m_goal = goal2map(&r_goal, &r_map, &transform, robot_topic);

            save_map_goal(r_map, m_pose, m_goal, robot_topic, map_pose_count);

            flowStatus++;
        }
    }

}

void ros_map_Callback(pioneer3at::OccMap map)
{
    r_map = map;
    
    if(flowStatus == NEW_MAP){
        save_map_simple(map, robot_topic);
        flowStatus++;
    } else if (flowStatus == GOAL_SET) {
        // if((!verify_if_goal_is_frontier(r_map, m_goal)) || (verify_if_goal_is_near(r_pose, r_goal))){
        if(verify_if_goal_is_near(r_pose, r_goal)){
            ROS_ERROR_STREAM("NOT FRONTIER");
            flowStatus = NEW_MAP;
        }

    }

}


/*****************************************
*                                        *
*                  MAIN                  *
*                                        *
*****************************************/


int main( int argc, char* argv[] )
{
    // Initialize ROS
    ros::init(argc, argv, "explorer");
    ros::NodeHandle n;
    ros::NodeHandle n_("~");

    n_.getParam("map_topic", map_topic);
    n_.getParam("occ_map_topic", occ_map_topic);
    n_.getParam("goal_topic", goal_topic);
    n_.getParam("pose_topic", pose_topic);
    // n_.getParam("cmd_vel_topic", cmd_vel_topic);
    // n_.getParam("laser_topic", laser_topic);
    // n_.getParam("goal_status_topic", goal_status_topic);
    n_.getParam("robot_topic", robot_topic);
    n_.getParam("base_link_topic", base_link_topic);
    n_.getParam("a_beta", a_beta);
    n_.getParam("b_beta", b_beta);
    n_.getParam("alpha", alpha);
    n_.getParam("beta", beta);
    n_.getParam("gama", gama);

    flowStatus = NEW_MAP;
    
    goal_pub = n.advertise<geometry_msgs::PoseStamped>(goal_topic, 1);

    map_sub = n.subscribe(occ_map_topic, 1, ros_map_Callback);
    pose_sub = n.subscribe(pose_topic, 1, ros_pose_CallBack);

    ros::spin();

}
