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

#include "gmapping/occMap.h"

#include "save_map.cc"


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
std::string cmd_vel_topic;
std::string base_link_topic;
std::string goal_status_topic;
std::string laser_topic;

gmapping::occMap r_map;
nav_msgs::Odometry r_pose;
geometry_msgs::PoseStamped r_goal;



/*****************************************
*                                        *
*               CALLBACKS                *
*                                        *
*****************************************/


void ros_pose_CallBack(nav_msgs::Odometry pose)
{
    double y = pose.pose.pose.position.y;
    double x = pose.pose.pose.position.x;
    double z = pose.pose.pose.position.z;
    double qx = pose.pose.pose.orientation.x;
    double qy = pose.pose.pose.orientation.y;
    double qz = pose.pose.pose.orientation.z;
    double qw = pose.pose.pose.orientation.w;

    double roll, pitch, yaw;
    tf::Quaternion qm(qx, qy, qz, qw);
    tf::Matrix3x3 mm(qm);
    mm.getRPY(roll, pitch, yaw);

    std::ofstream myfile;
    std::string filename = "/home/rcolares/catkin_ws/src/ros-pioneer3at/maps/" + robot_topic +"_pose.txt";
    myfile.open (filename.c_str(),  std::ios::out | std::ios::app );
    myfile << x << " " << y << " " << z << " " << roll << " " << pitch << " " << yaw << "\n";
    myfile.close();

    r_pose = pose;
    //set_new_goal(pose);

}

void ros_map_Callback(gmapping::occMap map)
{
    r_map = map;
    save_map_simple(map, robot_topic);

    if(!verify_if_goal_is_frontier(r_map, r_goal)){
        set_new_goal(r_pose);
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
    n_.getParam("cmd_vel_topic", cmd_vel_topic);
    n_.getParam("laser_topic", laser_topic);
    n_.getParam("goal_status_topic", goal_status_topic);
    n_.getParam("robot_topic", robot_topic);
    n_.getParam("base_link_topic", base_link_topic);


    std::ofstream myfile;
    std::string filename = "/home/rcolares/catkin_ws/src/ros-pioneer3at/maps/time_"+robot_topic+".txt";
    myfile.open (filename.c_str(),  std::ios::out | std::ios::app );
    myfile << ros::Time::now()<<"\n";
    myfile.close();

    goal_pub = n.advertise<geometry_msgs::PoseStamped>(goal_topic, 1);
//    cmd_vel_pub = n.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);

    map_sub = n.subscribe(occ_map_topic, 1, ros_map_Callback);
//    laser_sub = n.subscribe(laser_topic, 1, ros_laser_Callback);
//    goal_status_sub = n.subscribe(goal_status_topic, 1, ros_goal_status_Callback);
    pose_sub = n.subscribe(pose_topic, 1, ros_pose_CallBack);
//    cmd_vel_sub = n.subscribe(cmd_vel_topic, 1, ros_set_cmd_vel_CallBack);

    ros::spin();

}
