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


#include "frontier_window.cc"
#include "gmapping/occMap.h"
#include "wavefront.cc"
#include "utility_function.cc"


int id =0;
int goal_plan=0;
int chose_goal=-1;
double map_x,map_y, map_height, map_width;
double map_cell;

ros::Publisher frontier_cmd_vel;
ros::Publisher frontier_map_pose;
ros::Subscriber frontier_map;
ros::Subscriber frontier_occ_map;
ros::Subscriber frontier_goal_status;
ros::Subscriber frontier_pose;
nav_msgs::Path plan_path;

// geometry_msgs::PoseStamped goal;
move_base_msgs::MoveBaseActionGoal goal;

nav_msgs::Odometry actual_pose;
nav_msgs::OccupancyGrid map_copy;
std::string robot_topic;
std::string goal_topic;
std::string map_topic;
std::string occ_map_topic;
std::string pose_topic;
std::string base_link_topic;
std::string goal_status_topic;

int abortcont=0;

int **real_map;
double **mapa;
double **occ_real_map;
double **wave_map;
double **ww_map;

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

//       std::string mapdatafile = mapname_ + number + ".pgm";
//       ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());
//       FILE* out = fopen(mapdatafile.c_str(), "w");
//       if (!out)
//       {
//         ROS_ERROR("Couldn't save map file to %s", mapdatafile.c_str());
//         return;
//       }
      
      
//       fprintf(out, "P5\n# CREATOR: Map_generator.cpp %.3f m/pix\n%d %d\n255\n",
//               map.info.resolution, map.info.width, map.info.height);

  
  for(unsigned int y = 0; y < map.info.height; y++) {
    for(unsigned int x = 0; x < map.info.width; x++) {
      unsigned int i = x + (map.info.height - y - 1) * map.info.width;
      //if(mapa[y][x] < 233.75){
//             fputc(150,out);
          //fprintf(printFile, "%c%c%c", 255, 55, 55);
      /*} else*/ if (map.data[i] == 0) { //occ [0,0.1)
//             fputc(254, out);
          fprintf(printFile, "%c%c%c", 255, 255, 255);
      } else if (map.data[i] == +100) { //occ (0.65,1]
//             fputc(000, out);
          fprintf(printFile, "%c%c%c", 0, 0, 0);
      } else { //occ [0.1,0.65]
//             fputc(205, out);
          fprintf(printFile, "%c%c%c", 125, 125, 125);
      }
    }
  }
//       fclose(out);
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
}

void ros_goal_status_Callback(actionlib_msgs::GoalStatusArray goals){
  if(goal_plan==4){
    if(goals.status_list[0].status == 3 || goals.status_list[0].status == 4){
      abortcont++;
      goal_plan=0;
    }
  }
}


void ros_set_goal_CallBack(nav_msgs::Odometry odometry)
{
  if(goal_plan==1){
    
    actual_pose = odometry;
    goal_plan=2;
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
    
//     std::string num_id = id++;
//     goal.goal_id.id = num_id;
    goal.goal.target_pose.header.frame_id = map_topic;
    goal.goal.target_pose.header.stamp = ros::Time::now();
    
    goal.goal.target_pose.pose.position.x = x + ((goal_cell.x*map_cell)+map_x) - transform.getOrigin().x();
    goal.goal.target_pose.pose.position.y = y + (((map_height-goal_cell.y)*map_cell)+map_y) - transform.getOrigin().y();
//     goal.pose.position.x = x + 2;
//     goal.pose.position.y = y;
    goal.goal.target_pose.pose.position.z = odometry.pose.pose.position.z;
    goal.goal.target_pose.pose.orientation.x = 0.0;
    goal.goal.target_pose.pose.orientation.y = 0.0;
    goal.goal.target_pose.pose.orientation.z = 1.0;
    goal.goal.target_pose.pose.orientation.w = 1.0;
    //frontier_cmd_vel.publish(goal); 
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

void ros_save_occ_map_Callback(gmapping::occMap occ_map){
  int width = occ_map.map.info.width;
  int height = occ_map.map.info.height;
  map_x = occ_map.map.info.origin.position.x;
  map_y = occ_map.map.info.origin.position.y;
  map_cell = occ_map.map.info.resolution;
  map_height = height;
  map_width = width;
  
  if(goal_plan==0){
    
    real_map = (int**) malloc(height*sizeof(int*));
    mapa = (double**) malloc(height*sizeof(double*));
    occ_real_map = (double**) malloc(height*sizeof(double*));
    wave_map = (double**) malloc(height*sizeof(double*));
    ww_map = (double**) malloc(height*sizeof(double*));
    for(int i=0; i<height; i++){
      real_map[i] = (int*) malloc(width*sizeof(int*));
      mapa[i] = (double*) malloc(width*sizeof(double*));
      occ_real_map[i] = (double*) malloc(width*sizeof(double*));
      wave_map[i] = (double*) malloc(width*sizeof(double*));
      ww_map[i] = (double*) malloc(width*sizeof(double*));
    }  
   
    mapTransform(occ_map, width, height, real_map, mapa, occ_real_map);
    
    createFrontiers(width, height, real_map, mapa, occ_real_map, "0");
    
    
    
   
    goal_plan=1;
  }else if(goal_plan==2){
    
      
      int y = map_height - ((actual_pose.pose.pose.position.y - map_y)/map_cell);
      int x = (actual_pose.pose.pose.position.x - map_x)/map_cell;
      wavefront(width, height, real_map, wave_map, mapa, x, y,"0");
      
      chooseGoal(mapa, wave_map, ww_map, width, height, robot_topic);
      
      if(!goal_cell.valid || abortcont == 3){
        std::ofstream myfile;
        myfile.open ("/home/rafael/catkin_ws/src/ros-pioneer3at/maps/time.txt",  std::ios::out | std::ios::app );
        myfile << ros::Time::now();
        myfile.close();
        system("rosnode kill Pioneer3AT_frontier_window_explorer");
      }
      
      mapSaver(occ_map.map, "/home/rafael/catkin_ws/src/ros-pioneer3at/maps/", id++);
    
      for(int i=0;i<height;i++){
        free(real_map[i]);
        free(mapa[i]);
        free(occ_real_map[i]);
        free(wave_map[i]);
        free(ww_map[i]);
      }
      free(mapa);
      free(real_map);
      free(occ_real_map);
      free(wave_map);
      free(ww_map);
      goal_plan = 3;
  } 
  
}






int main( int argc, char* argv[] )
{
  // Initialize ROS
  ros::init(argc, argv, "Frontier_window_explorer");
  ros::NodeHandle n;
  ros::NodeHandle n_("~");
  
  n_.getParam("robot_topic", robot_topic);
  n_.getParam("goal_topic", goal_topic);
  n_.getParam("map_topic", map_topic);
  n_.getParam("pose_topic", pose_topic);
  n_.getParam("base_link_topic", base_link_topic);
  n_.getParam("occ_map_topic", occ_map_topic);
  n_.getParam("goal_status_topic", goal_status_topic);
  
  
  std::ofstream myfile;
  myfile.open ("/home/rafael/catkin_ws/src/ros-pioneer3at/maps/time.txt",  std::ios::out | std::ios::app );
  myfile << ros::Time::now()<<"\n";
  myfile.close();
  
//   frontier_cmd_vel = n.advertise<geometry_msgs::PoseStamped>(goal_topic, 1);
  frontier_cmd_vel = n.advertise<move_base_msgs::MoveBaseActionGoal>(goal_topic, 1);
  frontier_occ_map = n.subscribe(occ_map_topic, 1, ros_save_occ_map_Callback);
  frontier_goal_status = n.subscribe(goal_status_topic, 1, ros_goal_status_Callback);
  frontier_pose = n.subscribe(pose_topic, 1, ros_set_goal_CallBack);
  ros::spin();
  
}
