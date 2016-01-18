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
//#include "movebasemsgs/MoveBaseAction.h"
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

int first=0;
int id =0;
int goal_plan=0;
int chose_goal=-1;
double map_x,map_y, map_height, map_width;
double map_cell;

ros::Publisher frontier_cmd_vel;
ros::Publisher frontier_map_pose;
ros::Subscriber frontier_laser_scan;
ros::Subscriber frontier_map;
ros::Subscriber frontier_pose;
ros::Subscriber frontier_goal_status;
ros::Subscriber occ_subs;
nav_msgs::Path plan_path;



//move_base_msgs::MoveBaseActionGoal goal;
geometry_msgs::PoseStamped goal;

nav_msgs::Odometry actual_pose;
nav_msgs::OccupancyGrid map_copy;
std::string robot_topic;
std::string goal_topic;
std::string map_topic;
std::string pose_topic;
std::string base_link_topic;
std::string goal_status_topic;
std::string occ_topic;

int **real_map;
int **mapa;

int first_map=0;
int abortcont=0;



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
  

void ros_set_goal_CallBack(nav_msgs::Odometry odometry)
{
    std::ofstream myfile;

  if(goal_plan==1){
    
    actual_pose = odometry;
    myfile.open ("/home/rcolares/catkin_ws/src/ros-pioneer3at/teste.txt",  std::ios::out | std::ios::app );
    myfile << goal_plan<<"\n";
    myfile.close();
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
    
  
    //goal.goal.target_pose.header.frame_id = map_topic;
    //goal.goal.target_pose.header.stamp = ros::Time::now();
    
    //goal.goal.target_pose.pose.position.x = x + ((frontier_vector[chose_goal].x_mean*map_cell)+map_x) - transform.getOrigin().x();
    //goal.goal.target_pose.pose.position.y = y + (((map_height-frontier_vector[chose_goal].y_mean)*map_cell)+map_y) - transform.getOrigin().y();
    //goal.goal.target_pose.pose.position.z = odometry.pose.pose.position.z;
    //goal.goal.target_pose.pose.orientation.x = 0.0;
    //goal.goal.target_pose.pose.orientation.y = 0.0;
    //goal.goal.target_pose.pose.orientation.z = 1.0;
    //goal.goal.target_pose.pose.orientation.w = 1.0;

    goal.pose.position.x = x + ((frontier_vector[chose_goal].x_mean*map_cell)+map_x) - transform.getOrigin().x();
    goal.pose.position.y = y + (((map_height-frontier_vector[chose_goal].y_mean)*map_cell)+map_y) - transform.getOrigin().y();
    goal.pose.position.z = odometry.pose.pose.position.z;
    goal.pose.orientation.x = 0.0;
    goal.pose.orientation.y = 0.0;
    goal.pose.orientation.z = 1.0;
    goal.pose.orientation.w = 1.0;


    frontier_cmd_vel.publish(goal);
    myfile.open ("/home/rcolares/catkin_ws/src/ros-pioneer3at/teste.txt",  std::ios::out | std::ios::app );
    myfile << goal_plan<<"\n";
    myfile.close();
    goal_plan = 4;
    
  }else if(goal_plan==4){
    
    double y = odometry.pose.pose.position.y;
    double x = odometry.pose.pose.position.x;
    double xd = goal.pose.position.x;
    double yd = goal.pose.position.y;
    myfile.open ("/home/rcolares/catkin_ws/src/ros-pioneer3at/teste.txt",  std::ios::out | std::ios::app );
    myfile << goal_plan<<" "<<x<<" "<<y<<" "<<xd<<" "<<yd<<" "<<"\n";
    myfile.close();
    if( sqrt((x-xd)*(x-xd) + (y-yd)*(y-yd)) < 1.0){
      
      goal_plan = 0;
    }
      
  } 
}

void occ_save_CallBack(gmapping::occMap map){

    if(!first_map){
        int height = map.map.info.height;
        int width = map.map.info.width;
        std::ofstream myfile;

        myfile.open ("/home/rcolares/catkin_ws/src/ros-pioneer3at/occ.txt",  std::ios::out);
        for(int l=0;l<height;l++){
            for(int k=0;k<width;k++){
                int t = (height - l -1)*width +k;
                myfile << map.data[t]<<" ";

            }
            myfile<< std::endl;
        }
        myfile<<"end";
        myfile.close();
        first_map=1;
    }
}

void ros_save_map_Callback(nav_msgs::OccupancyGrid map)
{

    std::ofstream myfile;

    myfile.open ("/home/rcolares/catkin_ws/src/ros-pioneer3at/teste.txt",  std::ios::out | std::ios::app );
    myfile << "aqui\n";
    myfile.close();

  int width = map.info.width;
  int height = map.info.height;
  map_x = map.info.origin.position.x;
  map_y = map.info.origin.position.y;
  map_cell = map.info.resolution;
  map_height = height;
  map_width = width;
  
  map_copy = map;


  myfile.open ("/home/rcolares/catkin_ws/src/ros-pioneer3at/teste.txt",  std::ios::out | std::ios::app );
  myfile << "aqui22\n";
  myfile.close();

  if(goal_plan==0){
    real_map = (int**) malloc(height*sizeof(int*));
    mapa = (int**) malloc(height*sizeof(int*));
    for(int i=0; i<height; i++){
      real_map[i] = (int*) malloc(width*sizeof(int*));
      mapa[i] = (int*) malloc(width*sizeof(int*));
    }  
   
    mapTransform(map, width, height, real_map, mapa);
    
    createFrontiers(width, height, real_map, mapa);
    if(frontier_vector.size()==0){
      std::ofstream myfile;
      myfile.open ("/home/rcolares/catkin_ws/src/ros-pioneer3at/time.txt",  std::ios::out | std::ios::app );
      myfile << ros::Time::now()<<"\n";
      myfile.close();
      mapSaver(map, "/home/rcolares/catkin_ws/src/ros-pioneer3at/", id++);
      system("rosnode kill Pioneer3AT_frontier_explorer");
    }
    myfile.open ("/home/rcolares/catkin_ws/src/ros-pioneer3at/teste.txt",  std::ios::out | std::ios::app );
    myfile << goal_plan<<"\n";
    myfile.close();
    goal_plan=1;
  }else if(goal_plan==2){
    
      chose_goal = chooseGoal(actual_pose); 
      
      mapSaver(map, "/home/rcolares/catkin_ws/src/ros-pioneer3at/", id++);
    
      for(int i=0;i<height;i++){
        free(real_map[i]);
        free(mapa[i]);
      }
      free(mapa);
      free(real_map);
      myfile.open ("/home/rcolares/catkin_ws/src/ros-pioneer3at/teste.txt",  std::ios::out | std::ios::app );
      myfile << goal_plan<<"\n";
      myfile.close();
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
  n_.getParam("goal_status_topic", goal_status_topic);
  n_.getParam("occ_topic", occ_topic);
  
  std::ofstream myfile;
  myfile.open ("/home/rcolares/catkin_ws/src/ros-pioneer3at/time.txt",  std::ios::out | std::ios::app );
  myfile << ros::Time::now()<<"\n";
  myfile.close();
  
  frontier_cmd_vel = n.advertise<geometry_msgs::PoseStamped>(goal_topic, 1);
  //frontier_goal_status = n.subscribe(goal_status_topic, 1, ros_goal_status_Callback);
  frontier_map = n.subscribe(map_topic, 1, ros_save_map_Callback);
  frontier_pose = n.subscribe(pose_topic, 1, ros_set_goal_CallBack);
  occ_subs = n.subscribe(occ_topic, 1, occ_save_CallBack);
  ros::spin();
  
}
