// /************************************
// *           TODO LIST
// *
// *  - set_new_goal
// *    - adapt information function
// *    - adapt distance function
// *
// *  - define detection procedure
// *
// *  - define map exchange method
// *
// ************************************/

// #include <ros/ros.h>
// #include <geometry_msgs/Twist.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <sensor_msgs/LaserScan.h>
// #include <nav_msgs/OccupancyGrid.h>
// #include <nav_msgs/Odometry.h>
// #include <nav_msgs/Path.h>
// #include <std_msgs/String.h>
// #include <actionlib/client/simple_action_client.h>
// #include <actionlib_msgs/GoalStatusArray.h>
// #include <move_base_msgs/MoveBaseActionGoal.h>
// #include <move_base_msgs/MoveBaseActionResult.h>
// #include <nav_msgs/GetMap.h>
// #include <nav_msgs/GetPlan.h>
// #include <tf/transform_listener.h>
// #include <tf/LinearMath/Matrix3x3.h>
// #include <geometry_msgs/Quaternion.h>
// #include <move_base_msgs/MoveBaseAction.h>
// #include <ros/console.h>
// #include <fstream>
// #include <iostream>
// #include <cstdlib>
// #include <string>
// #include <vector>
// #include <stdio.h>

// #include "pioneer3at/OccMap.h"
// #include "utils.h"

// #include "save_map.cc"

// #include "utils.cc"

// #include "goal_tools.cc"


// #define NEW_MAP 0
// #define SET_NEW_GOAL 1
// #define GOAL_SET 2



// ros::Publisher goal_pub;
// ros::Publisher cmd_vel_pub;

// ros::Subscriber map_sub;
// ros::Subscriber goal_result_sub;
// ros::Subscriber pose_sub;
// ros::Subscriber cmd_vel_sub;
// ros::Subscriber laser_sub;

// ros::ServiceClient path_srv;

// std::string robot_topic;
// std::string goal_topic;
// std::string map_topic;
// std::string occ_map_topic;
// std::string pose_topic;
// // std::string cmd_vel_topic;
// std::string base_link_topic;
// // std::string goal_status_topic;
// // std::string laser_topic;

// double alpha;
// double beta;
// double gama;
// double a_beta;
// double b_beta;

// pioneer3at::OccMap r_map;
// nav_msgs::Odometry r_pose;
// mapPose m_pose;
// odomPose o_pose;
// mapPose m_goal;
// geometry_msgs::PoseStamped r_goal;

// int flowStatus;
// int goal_result = 0;
// int map_pose_count = 0;
// bool initialized = false;
// /*****************************************
// *                                        *
// *               CALLBACKS                *
// *                                        *
// *****************************************/


// void ros_pose_CallBack(nav_msgs::Odometry pose)
// {   // wait for the first map
//     if(robot_topic.compare("/robot_0") == 0 && r_map.map.info.resolution > 0){
//         tf::StampedTransform transform;
//         tf::TransformListener listener;

//         try{
//             listener.waitForTransform(map_topic, base_link_topic, ros::Time(0), ros::Duration(5.0));
//             listener.lookupTransform(map_topic, base_link_topic, ros::Time(0), transform);
//         }catch (tf::TransformException ex){
//             ROS_ERROR("%s",ex.what());
//         }

//         r_pose = pose;

//         m_pose = odom2map(&r_pose, &r_map, &transform, robot_topic);

//         // saveMapPose(m_pose, robot_topic);
//         // saveOdomPose(o_pose, robot_topic);

//         if(flowStatus == SET_NEW_GOAL){
//             goal_result = 0;
//             // ROS_ERROR_STREAM("PUBLISH NEW GOAL");
//             save_map_pose(r_map, m_pose, robot_topic, map_pose_count++);
//             r_goal = setNewGoal(&r_map, pose, m_pose, &transform, a_beta, b_beta, alpha, beta, gama, path_srv);
            
//             if(r_goal.pose.position.z == -1){
//                 std::ofstream myfile;
//                 std::string package = ros::package::getPath("pioneer3at");
//                 std::string filename = package + "/maps/" + robot_topic +"_time.txt";
//                 myfile.open (filename.c_str(),  std::ios::out | std::ios::app );
//                 myfile << ros::Time::now();
//                 myfile.close();
//                 std::string cmd = "rosnode kill Explorer";
//                 system(cmd.c_str());
//             }

//             goal_pub.publish(r_goal);
//             ROS_INFO("OOO Goal set");
//             m_goal = goal2map(&r_goal, &r_map, &transform, robot_topic);

//             save_map_goal(r_map, m_pose, m_goal, robot_topic, map_pose_count);

//             flowStatus = GOAL_SET;
//         }
//     }

// }

// void ros_map_Callback(pioneer3at::OccMap map)
// {
//     r_map = map;
//     if(flowStatus == GOAL_SET){
//         flowStatus = NEW_MAP;
//         ROS_INFO("OOO New map");
//     }
// }

// void goal_result_CallBack(actionlib_msgs::GoalStatusArray result)
// {
//     if(flowStatus == NEW_MAP && result.status_list.size() > 0 && result.status_list[result.status_list.size()-1].status >= 2){
//         ROS_INFO("OOO Goal reached");
//         save_map_simple(r_map, robot_topic);
//         flowStatus = SET_NEW_GOAL;
//     }
// }

// /*****************************************
// *                                        *
// *                  MAIN                  *
// *                                        *
// *****************************************/


// int main( int argc, char* argv[] )
// {
//     // Initialize ROS
//     ros::init(argc, argv, "explorer");
//     ros::NodeHandle n;
//     ros::NodeHandle n_("~");

//     n_.getParam("map_topic", map_topic);
//     n_.getParam("occ_map_topic", occ_map_topic);
//     n_.getParam("goal_topic", goal_topic);
//     n_.getParam("pose_topic", pose_topic);
//     // n_.getParam("cmd_vel_topic", cmd_vel_topic);
//     // n_.getParam("laser_topic", laser_topic);
//     // n_.getParam("goal_status_topic", goal_status_topic);
//     n_.getParam("robot_topic", robot_topic);
//     n_.getParam("base_link_topic", base_link_topic);
//     n_.getParam("a_beta", a_beta);
//     n_.getParam("b_beta", b_beta);
//     n_.getParam("alpha", alpha);
//     n_.getParam("beta", beta);
//     n_.getParam("gama", gama);

//     std::ofstream myfile;
//     std::string package = ros::package::getPath("pioneer3at");
//     std::string filename = package + "/maps/" + robot_topic +"_time.txt";
//     myfile.open (filename.c_str(),  std::ios::out | std::ios::app );
//     myfile << ros::Time::now()<<"\n";
//     myfile.close();
    
//     goal_pub = n.advertise<geometry_msgs::PoseStamped>(goal_topic, 1);

//     map_sub = n.subscribe(occ_map_topic, 1, ros_map_Callback);
//     pose_sub = n.subscribe(pose_topic, 1, ros_pose_CallBack);
//     goal_result_sub = n.subscribe("move_base/status", 1, goal_result_CallBack);

//     path_srv = n.serviceClient<nav_msgs::GetPlan>("move_base/make_plan");

//     // flowStatus = NEW_MAP;
//     flowStatus = SET_NEW_GOAL;
//     ros::spin();
// }








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
#include <move_base_msgs/MoveBaseActionResult.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/GetPlan.h>
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


ros::Publisher goal_pub;
ros::Publisher cmd_vel_pub;

ros::Subscriber map_sub;
ros::Subscriber goal_result_sub;
ros::Subscriber pose_sub;
ros::Subscriber cmd_vel_sub;
ros::Subscriber laser_sub;

ros::ServiceClient path_srv;

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
double robot_radius;

pioneer3at::OccMap r_map;
nav_msgs::Odometry r_pose;
mapPose m_pose;
odomPose o_pose;
mapPose m_goal;
geometry_msgs::PoseStamped r_goal;

int goal_status = 7;
int map_pose_count = 0;
/*****************************************
*                                        *
*               CALLBACKS                *
*                                        *
*****************************************/


void ros_pose_CallBack(nav_msgs::Odometry pose){   // wait for the first map
    bool goal_published;
    if(robot_topic.compare("/robot_0") == 0 && r_map.map.info.resolution > 0){
        tf::StampedTransform transform;
        tf::TransformListener listener;

        try{
            listener.waitForTransform(map_topic, base_link_topic, ros::Time(0), ros::Duration(5.0));
            listener.lookupTransform(map_topic, base_link_topic, ros::Time(0), transform);
        }catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
        }

        m_pose = odom2map(&pose, &r_map, &transform, robot_topic);

        // std::string file_path = ros::package::getPath("pioneer3at") + "/maps/test_map"; // save map
        // std::string cmd = "rosrun map_server map_saver -f test_map";// + file_path;
        // system(cmd.c_str());

        // saveMapPose(m_pose, robot_topic);
        // saveOdomPose(o_pose, robot_topic);
        if(goal_status > 2){
            goal_status = 0;
            // ROS_ERROR_STREAM("PUBLISH NEW GOAL");
            // save_map_pose(r_map, m_pose, robot_topic, map_pose_count++);
            goal_published = setNewGoal(&r_map, pose, m_pose, &transform, a_beta, b_beta, alpha, beta, gama, path_srv, goal_pub, robot_radius);
            if(goal_published == false){
                std::ofstream myfile;
                std::string package = ros::package::getPath("pioneer3at");
                std::string filename = package + "/maps/" + robot_topic +"_time.txt";
                myfile.open (filename.c_str(),  std::ios::out | std::ios::app );
                myfile << ros::Time::now();
                myfile.close();
                std::string cmd = "rosnode kill Explorer"; // kill node
                system(cmd.c_str());
            }
            m_goal = goal2map(&r_goal, &r_map, &transform, robot_topic);
            // save_map_goal(r_map, m_pose, m_goal, robot_topic, map_pose_count);
        }
    }

}

void ros_map_Callback(pioneer3at::OccMap map){
    r_map = map;
}

void goal_result_CallBack(move_base_msgs::MoveBaseActionResult result){
    goal_status = result.status.status;
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
    n_.getParam("robot_radius", robot_radius);

    std::ofstream myfile;
    std::string package = ros::package::getPath("pioneer3at");
    std::string filename = package + "/maps/" + robot_topic +"_time.txt";
    myfile.open (filename.c_str(),  std::ios::out | std::ios::app );
    myfile << ros::Time::now()<<"\n";
    myfile.close();
    
    goal_pub = n.advertise<geometry_msgs::PoseStamped>(goal_topic, 1);

    map_sub = n.subscribe(occ_map_topic, 1, ros_map_Callback);
    pose_sub = n.subscribe(pose_topic, 1, ros_pose_CallBack);
    goal_result_sub = n.subscribe("move_base/result", 1, goal_result_CallBack);

    path_srv = n.serviceClient<nav_msgs::GetPlan>("move_base/make_plan");

    ros::spin();
}
