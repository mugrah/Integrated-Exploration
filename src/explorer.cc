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






void ros_set_goal_CallBack(nav_msgs::Odometry odometry)
{

    double y = odometry.pose.pose.position.y;
    double x = odometry.pose.pose.position.x;
    double z = odometry.pose.pose.position.z;
    double qx = odometry.pose.pose.orientation.x;
    double qy = odometry.pose.pose.orientation.y;
    double qz = odometry.pose.pose.orientation.z;
    double qw = odometry.pose.pose.orientation.w;
    geometry_msgs::PoseStamped goal;

    tf::StampedTransform transform;
    tf::TransformListener listener;

    try{
        listener.waitForTransform(map_topic, base_link_topic, ros::Time(0), ros::Duration(5.0));
        listener.lookupTransform(map_topic, base_link_topic, ros::Time(0), transform);
    }catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }


    goal.header.frame_id = map_topic;
    goal.header.stamp = ros::Time::now();

    goal.pose.position.x = x+1;
    goal.pose.position.y = y;
    goal.pose.position.z = z;
    goal.pose.orientation.x = qx;
    goal.pose.orientation.y = qy;
    goal.pose.orientation.z = qz;
    goal.pose.orientation.w = qw;

    goal_pub.publish(goal);


}


//void ros_laser_Callback(sensor_msgs::LaserScan scan){

//    int min;
//    double aux = 99.0;

//    geometry_msgs::Twist cmd_vel;

//    cmd_vel.linear.x = 1.0;
//    cmd_vel.linear.y = 0.0;
//    cmd_vel.linear.z = 0.0;
//    cmd_vel.angular.x = 0.0;
//    cmd_vel.angular.y = 0.0;
//    cmd_vel.angular.z = 0.0;

//    for(int i = 0; i < scan.ranges.size(); i++){
//        if(scan.ranges[i] < aux){
//            min=i;
//            aux=scan.ranges[i];
//        }
//    }

//    if(aux<1.0){
//        if(min <= scan.ranges.size()/2){
//            cmd_vel.linear.x = 0.2;
//            cmd_vel.angular.z = 1.0;
//        }else{
//            cmd_vel.linear.x = 0.2;
//            cmd_vel.angular.z = -1.0;
//        }
//    }
//    cmd_vel_pub.publish(cmd_vel);
//}


void ros_map_Callback(gmapping::occMap map)
{
    save_map_simple(map);
}



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
    pose_sub = n.subscribe(pose_topic, 1, ros_set_goal_CallBack);
//    cmd_vel_sub = n.subscribe(cmd_vel_topic, 1, ros_set_cmd_vel_CallBack);

    ros::spin();

}
