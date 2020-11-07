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

#include <stage_ros/fiducials.h>

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
ros::Subscriber action_sub;
ros::Subscriber fiducial_sub;

// ros::ServiceClient path_srv;

std::string robot_topic;
std::string goal_topic;
std::string map_topic;
std::string occ_map_topic;
std::string pose_topic;
std::string fiducial_topic;
// std::string cmd_vel_topic;
std::string base_link_topic;
// std::string goal_status_topic;
// std::string laser_topic;

double alpha;
double beta;
double gama1;
double gama2;
double gama3;
double robot_radius;
double sigma;

int n_size;

pioneer3at::OccMap r_map;
nav_msgs::Odometry r_pose;
mapPose m_pose;
odomPose o_pose;
mapPose m_goal;
geometry_msgs::PoseStamped r_goal;
std::vector<nav_msgs::Odometry> other_poses;

int goal_status = 7;
int map_pose_count = 0;
int abort_run = 0;
int fiducial_status = 3;

double f_x, f_y;
double f_dist = 0.0;
/*****************************************
*                                        *
*               CALLBACKS                *
*                                        *
*****************************************/

void end_run(){
    std::ofstream myfile;
    std::string package = ros::package::getPath("pioneer3at");
    std::string filename = package + "/maps/" + robot_topic +"_time.txt";
    myfile.open (filename.c_str(),  std::ios::out | std::ios::app );
    myfile << ros::Time::now();
    // myfile << alpha << "," << beta << "," << ros::Time::now()<<"\n";
    myfile.close();
    std::string cmd = "rosnode kill Explorer"; // kill node
    system(cmd.c_str());
}

void calc_dist(){
    double x_new = r_pose.pose.pose.position.x;
    double y_new = r_pose.pose.pose.position.y;
    
    f_dist += sqrt((x_new-f_x) * (x_new-f_x) + (y_new-f_y) * (y_new-f_y));
    f_x = x_new;
    f_y = y_new;
    if (f_dist > 5.0){
        fiducial_status = 2;
    }

}

void ros_fiducial_Callback(stage_ros::fiducials fiducials){
    
    if (fiducial_status == 3){
        f_x = r_pose.pose.pose.position.x;
        f_y = r_pose.pose.pose.position.y;
        fiducial_status = 2;
    }
    
    if (goal_status > 2) {
        calc_dist();
        if (fiducial_status == 2){
            other_poses.clear();
            for (int i=0; i < fiducials.observations.size(); i++){

                std::string topic = "/robot_" + std::to_string(fiducials.observations[i].id - 1) + "/odom";
                boost::shared_ptr<nav_msgs::Odometry const> sharedaux;
                nav_msgs::Odometry aux;
                sharedaux = ros::topic::waitForMessage<nav_msgs::Odometry>(topic,ros::Duration(5));
                if(sharedaux != NULL){
                    aux = *sharedaux;
                }
                other_poses.push_back(aux);
            }
            f_dist = 0.0;
            fiducial_status = 1;
        }
    }

}

void ros_pose_CallBack(nav_msgs::Odometry pose){   // wait for the first map
    bool goal_published;
    r_pose = pose;
    // if(robot_topic.compare("robot_0") == 0 && r_map.map.info.resolution > 0){
    if(r_map.map.info.resolution > 0){
        tf::StampedTransform transform;
        tf::TransformListener listener;
        try{
            listener.waitForTransform(map_topic, base_link_topic, ros::Time(0), ros::Duration(5.0));
            listener.lookupTransform(map_topic, base_link_topic, ros::Time(0), transform);
        }catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
        }

        m_pose = odom2map(&pose, &r_map);

        // saveMapPose(m_pose, robot_topic);
        // saveOdomPose(o_pose, robot_topic);
        if(goal_status > 2 || fiducial_status == 1){
            // goal_status = 99;
            goal_status = 0;
            // // save_map_pose(r_map, m_pose, robot_topic, map_pose_count++);
            if (fiducial_status == 1){
                fiducial_status = 0;
            } else {
                other_poses.clear();
            }
            goal_published = setNewGoal(&r_map, pose, m_pose, other_poses, alpha, beta, gama1, gama2, gama3, "move_base/make_plan", goal_pub, robot_radius, n_size, sigma);
            save_map_pose(r_map, m_pose, robot_topic, map_pose_count++);
            // goal_published = false;
            if(goal_published == false){
                end_run();
            }
            // m_goal = goal2map(&r_goal, &r_map, &transform, robot_topic);
            // save_map_goal(r_map, m_pose, m_goal, robot_topic, map_pose_count);
        }
    }

}

void ros_map_Callback(pioneer3at::OccMap map){
    r_map = map;
}

void goal_result_CallBack(move_base_msgs::MoveBaseActionResult result){
    goal_status = result.status.status;
    if (goal_status == 4){
        abort_run++;
    }
  
    if(abort_run == 3)
        end_run();
        
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
    n_.getParam("fiducial_topic", fiducial_topic);
    // n_.getParam("cmd_vel_topic", cmd_vel_topic);
    // n_.getParam("laser_topic", laser_topic);
    // n_.getParam("goal_status_topic", goal_status_topic);
    n_.getParam("robot_topic", robot_topic);
    n_.getParam("base_link_topic", base_link_topic);
    n_.getParam("alpha", alpha);
    n_.getParam("beta", beta);
    n_.getParam("gama1", gama1);
    n_.getParam("gama2", gama2);
    n_.getParam("gama3", gama3);
    n_.getParam("robot_radius", robot_radius);
    n_.getParam("n_size", n_size);
    n_.getParam("sigma", sigma);

    // std::ofstream myfile;
    // std::string package = ros::package::getPath("pioneer3at");
    // std::string filename = package + "/maps/" + robot_topic +"_time.txt";
    // myfile.open (filename.c_str(),  std::ios::out | std::ios::app );
    // myfile << alpha << " " << beta << " " << ros::Time::now()<<"\n";
    // myfile.close();
    
    goal_pub = n.advertise<geometry_msgs::PoseStamped>(goal_topic, 1);

    map_sub = n.subscribe(occ_map_topic, 1, ros_map_Callback);
    pose_sub = n.subscribe(pose_topic, 1, ros_pose_CallBack);
    fiducial_sub = n.subscribe(fiducial_topic, 1, ros_fiducial_Callback);
    goal_result_sub = n.subscribe("move_base/result", 1, goal_result_CallBack);


    // path_srv = n.serviceClient<nav_msgs::GetPlan>("move_base/make_plan");

    ros::spin();
}
