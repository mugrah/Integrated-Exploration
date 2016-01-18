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


#define PI 3.14
#define RAD2GR 180/PI
#define MAX_SIZE 3000 3000
#define ALPHA 1.0
#define BETA 1.0
#define MAX_DIST 999999
#define UNCSIZE 40
#define STD 500.0
#define MU 0.0
#define NUM_ROBOT 2


#include "frontier.cc"
#include "gmapping/occMap.h"
#include "pioneer3at/poses.h"
#include "pioneer3at/signal.h"

ros::Publisher frontier_cmd_vel;
ros::Publisher frontier_map_pose;
ros::Publisher frontier_robots_poses;
ros::Subscriber frontier_map;
ros::Subscriber frontier_occ_map;
ros::Subscriber frontier_goal_status;
ros::Subscriber frontier_pose;
ros::Subscriber frontier_other_robot_pose;
ros::Subscriber map_node_signal_rcv;
nav_msgs::Path plan_path;

// geometry_msgs::PoseStamped goal;
move_base_msgs::MoveBaseActionGoal goal;

nav_msgs::Odometry actual_pose;
std::string robot_topic;
std::string goal_topic;
std::string map_topic;
std::string occ_map_topic;
std::string pose_topic;
std::string base_link_topic;
std::string goal_status_topic;
std::string other_robot_pose_topic;
std::string robots_poses_topic;
std::string signal_topic;

pioneer3at::poses posesWave;

int **real_map;
int **mapa;

int first=0;
int id =0;
int goal_plan=-1;
int chose_goal=-1;
double map_x,map_y, map_height, map_width;
double map_cell;
int abortcont=0;
int cont_pose=0;
int timer = 0;
double start_timer;
int last_status =-1;




void mapSaver(nav_msgs::OccupancyGrid map, const std::string& mapname_, int id){
  ROS_INFO("Received a %d X %d map @ %.3f m/pix",
               map.info.width,
               map.info.height,
               map.info.resolution);
  char sysCall[512];
  char number[512]; 

  sprintf ( number, "%d", id );
  
  
  std::string mapdatafile = mapname_ + robot_topic + "_" + number;
  sprintf(sysCall, "%s.ppm", mapdatafile.c_str());
  FILE* printFile = fopen(sysCall, "w");
  fprintf(printFile, "P6\n # particles.ppm \n %d %d\n", map.info.width, map.info.height);
  fprintf(printFile, "255\n");

  for(unsigned int y = 0; y < map.info.height; y++) {
    for(unsigned int x = 0; x < map.info.width; x++) {
      unsigned int i = x + (map.info.height - y - 1) * map.info.width;
      if (map.data[i] == 0) { //occ [0,0.1)
          fprintf(printFile, "%c%c%c", 255, 255, 255);
      } else if (map.data[i] == +100) { //occ (0.65,1]
          fprintf(printFile, "%c%c%c", 0, 0, 0);
      } else { //occ [0.1,0.65]
          fprintf(printFile, "%c%c%c", 125, 125, 125);
      }
    }
  }
  fclose(printFile);
  sprintf(sysCall, "convert %s.ppm %s.png", mapdatafile.c_str(), mapdatafile.c_str());
  system(sysCall);
  sprintf(sysCall, "chmod 666 %s.ppm", mapdatafile.c_str());
  system(sysCall);
  sprintf(sysCall, "chmod 666 %s.png", mapdatafile.c_str());
  system(sysCall);
  sprintf(sysCall, "rm %s.ppm", mapdatafile.c_str());
  system(sysCall);
  
  std::string mapmetadatafile = mapname_ + robot_topic + "_" +  number + ".yaml";
  ROS_INFO("Writing map occupancy data to %s", mapmetadatafile.c_str());
  FILE* yaml = fopen(mapmetadatafile.c_str(), "w");

  geometry_msgs::Quaternion orientation = map.info.origin.orientation;
  tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
  double yaw, pitch, roll;
  mat.getEulerYPR(yaw, pitch, roll);

  fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
          mapdatafile.c_str(), map.info.resolution, map.info.origin.position.x, map.info.origin.position.y, yaw);

  fclose(yaml);

  ROS_INFO("Done\n");
  
  std::ofstream myfile;
  std::string filename = "/home/rafael/catkin_ws/src/ros-pioneer3at/maps/plan_time_"+robot_topic+".txt";
  myfile.open (filename.c_str(),  std::ios::out | std::ios::app );
  myfile << ros::Time::now()<<"\n";
  myfile.close();
}

void ros_signal_Callback(pioneer3at::signal sig){
  if(sig.sig == 1){
    goal_plan = 0;
  }
}

void ros_goal_status_Callback(actionlib_msgs::GoalStatusArray goals){
  if(goal_plan==4){
    if(goals.status_list[0].status == 3 || goals.status_list[0].status == 4){
      abortcont++;
      goal_plan=0;
     
    }
  }
}

int chooseGoal(nav_msgs::Odometry odometry){
    
  int y = map_height - ((odometry.pose.pose.position.y - map_y)/map_cell);
  int x = (odometry.pose.pose.position.x - map_x)/map_cell;
  int k;
  double dist, err, f;
  double best = 0.0;
  int max_dist = MAX_DIST;
  for(int i = 0; i<frontier_vector.size(); i++){
    dist = sqrt((frontier_vector[i].y_mean - y)*(frontier_vector[i].y_mean - y) + (frontier_vector[i].x_mean - x)*(frontier_vector[i].x_mean - x));
    if(dist<max_dist && dist > 30.0){
      max_dist=dist;
      k=i;
    } 
  }
  return k;
}

void ros_other_robot_pose_Callback(nav_msgs::Odometry odometry){
    int y_other = map_height - ((odometry.pose.pose.position.y - map_y)/map_cell);
    int x_other = (odometry.pose.pose.position.x - map_x)/map_cell;
    int y_mine = map_height - ((actual_pose.pose.pose.position.y - map_y)/map_cell);
    int x_mine = (actual_pose.pose.pose.position.x - map_x)/map_cell;

    double dist = sqrt((y_mine - y_other)*(y_mine - y_other) + (x_mine - x_other)*(x_mine - x_other));
 
    if(dist>=20 && dist <= 150.0){
      if(!timer){      
        timer=1;
        posesWave.dist = dist;
        posesWave.x_other = x_other;
        posesWave.y_other = y_other;
        posesWave.x = x_mine;
        posesWave.y = y_mine;
        std::ofstream myfile;
        std::string filename = "/home/rafael/catkin_ws/src/ros-pioneer3at/maps/encounter_"+robot_topic+".txt";
        myfile.open (filename.c_str(),  std::ios::out | std::ios::app );
        myfile << x_mine <<"   "<< y_mine <<" ---- "<<x_other<<"   "<<y_other<<"  "<<id<<"\n";
        myfile.close();
      }
      
    }
    
    if(sqrt(pow((x_mine - posesWave.x),2) - pow((y_mine - posesWave.y),2)) > 100){
      timer=0;
      posesWave.dist = dist;
      posesWave.x_other = -1;
      posesWave.y_other = -1;
      //posesWave.x = x_mine;
      //posesWave.y = y_mine;
    }
    
    
    pioneer3at::poses aux;
    aux.dist = dist;
    aux.x_other = x_other;
    aux.y_other = y_other;
    aux.x = x_mine;
    aux.y = y_mine;
    frontier_robots_poses.publish(aux);
}
  

void ros_set_goal_CallBack(nav_msgs::Odometry odometry)
{
  if(goal_plan==1){
    
    actual_pose = odometry;
    goal_plan=2; //TODO trocar para 2
  }else if(goal_plan==3)
  {
    
    double y = odometry.pose.pose.position.y;
    double x = odometry.pose.pose.position.x;
    
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
    
    goal.goal.target_pose.header.frame_id = map_topic;
    goal.goal.target_pose.header.stamp = ros::Time::now();
    
    goal.goal.target_pose.pose.position.x = x + ((frontier_vector[chose_goal].x_mean*map_cell)+map_x) - transform.getOrigin().x();
    goal.goal.target_pose.pose.position.y = y + (((map_height-frontier_vector[chose_goal].y_mean)*map_cell)+map_y) - transform.getOrigin().y();
    

    goal.goal.target_pose.pose.position.z = odometry.pose.pose.position.z;
    goal.goal.target_pose.pose.orientation.x = 0.0;
    goal.goal.target_pose.pose.orientation.y = 0.0;
    goal.goal.target_pose.pose.orientation.z = 1.0;
    goal.goal.target_pose.pose.orientation.w = 1.0;
    frontier_cmd_vel.publish(goal); 
    goal_plan = 4;
    
  }else if(goal_plan==4){
    
    double y = odometry.pose.pose.position.y;
    double x = odometry.pose.pose.position.x;
    double xd = goal.goal.target_pose.pose.position.x;
    double yd = goal.goal.target_pose.pose.position.y;
    
    if( sqrt((x-xd)*(x-xd) + (y-yd)*(y-yd)) < 1.0){
      
      goal_plan = 0;
    }
      
  } 
}

void ros_save_map_Callback(nav_msgs::OccupancyGrid map)
{
  int width = map.info.width;
  int height = map.info.height;
  map_x = map.info.origin.position.x;
  map_y = map.info.origin.position.y;
  map_cell = map.info.resolution;
  map_height = height;
  map_width = width;
  

  
  if(goal_plan==0){
    real_map = (int**) malloc(height*sizeof(int*));
    mapa = (int**) malloc(height*sizeof(int*));
    for(int i=0; i<height; i++){
      real_map[i] = (int*) malloc(width*sizeof(int*));
      mapa[i] = (int*) malloc(width*sizeof(int*));
    }  
    mapTransform(map, width, height, real_map, mapa);
     
    createFrontiers(width, height, real_map, mapa);
     
    if(frontier_vector.size()==0 || abortcont == 3){
      std::ofstream myfile;
      std::string filename = "/home/rafael/catkin_ws/src/ros-pioneer3at/maps/time_"+robot_topic+".txt";
      myfile.open (filename.c_str(),  std::ios::out | std::ios::app );
      myfile << ros::Time::now();
      myfile.close();
      std::string cmd = "rosnode kill Pioneer3AT_window_explorer";
      system(cmd.c_str());
    }
    goal_plan=1;
  }else if(goal_plan==2){
     
      chose_goal = chooseGoal(actual_pose); 
       
      mapSaver(map, "/home/rafael/catkin_ws/src/ros-pioneer3at/maps/", id++);
    
      for(int i=0;i<height;i++){
        free(real_map[i]);
        free(mapa[i]);
      }
      free(mapa);
      free(real_map);
      
      goal_plan = 3;
  } 
}



int main( int argc, char* argv[] )
{
  // Initialize ROS
  ros::init(argc, argv, "Frontier_explorer");
  ros::NodeHandle n;
  ros::NodeHandle n_("~");
  
  n_.getParam("robot_topic", robot_topic);
  n_.getParam("goal_topic", goal_topic);
  n_.getParam("map_topic", map_topic);
  n_.getParam("pose_topic", pose_topic);
  n_.getParam("base_link_topic", base_link_topic);
  n_.getParam("occ_map_topic", occ_map_topic);
  n_.getParam("goal_status_topic", goal_status_topic);
  n_.getParam("robots_poses_topic", robots_poses_topic);
  n_.getParam("other_robot_pose_topic", other_robot_pose_topic);
  n_.getParam("signal_topic", signal_topic);
  
    
  std::ofstream myfile;
  std::string filename = "/home/rafael/catkin_ws/src/ros-pioneer3at/maps/time_"+robot_topic+".txt";
  myfile.open (filename.c_str(),  std::ios::out | std::ios::app );
  myfile << ros::Time::now()<<"\n";
  myfile.close();
  
  if(goal_plan==-1){
    posesWave.x_other = -1;
    posesWave.y_other = -1;
    goal_plan = 0;
  }
 
  frontier_cmd_vel = n.advertise<move_base_msgs::MoveBaseActionGoal>(goal_topic, 1);
  frontier_robots_poses = n.advertise<pioneer3at::poses>(robots_poses_topic,1);
//   frontier_map_pose = n.advertise<nav_msgs::OccupancyGrid>(map_merge_topic,1);
  frontier_other_robot_pose = n.subscribe(other_robot_pose_topic, 1, ros_other_robot_pose_Callback);
  frontier_goal_status = n.subscribe(goal_status_topic, 1, ros_goal_status_Callback);
  frontier_map = n.subscribe(map_topic, 1, ros_save_map_Callback);
  frontier_pose = n.subscribe(pose_topic, 1, ros_set_goal_CallBack);
  map_node_signal_rcv = n.subscribe(signal_topic, 1 , ros_signal_Callback);
  ros::spin();
  
}
