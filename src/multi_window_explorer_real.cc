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

#include <tf/transform_datatypes.h>

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
#include "pioneer3at/OccMap.h"
#include "pioneer3at/Poses.h"
#include "pioneer3at/Signal.h"
// #include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include "wavefront.cc"
#include "utility_function.cc"


int id =0;
int goal_plan=-1;
int chose_goal=-1;
int cont_pose=0;
double xfactor = 0.225 / 0.05;
double yfactor = /*0*/0.115 / 0.05;
double map_x,map_y, map_height, map_width;
double map_cell;

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

geometry_msgs::PoseStamped goal;
//move_base_msgs::MoveBaseActionGoal goal;

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

pioneer3at::Poses posesWave;

int abortcont=0;

int **real_map;
double **mapa;
double **occ_real_map;
double **wave_map;
double **ww_map;
int timer = 0;
double start_timer;
int last_status =-1;

double origin_x, origin_y;


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

void ros_signal_Callback(pioneer3at::Signal sig){
    if(sig.sig == 1){

        goal_plan = 0;
    }
    std::ofstream myfile;
    std::string filename = "/home/rcolares/catkin_ws/src/ros-pioneer3at/maps/signal" + robot_topic +"_"".txt";
    myfile.open (filename.c_str(),  std::ios::out | std::ios::app );
    myfile << sig.sig<<"  "<<goal_plan<<"\n";
    myfile.close();
}


void ros_goal_status_Callback(actionlib_msgs::GoalStatusArray goals){
  
//   if(goals.status_list[0].status != last_status){
//       std::ofstream myfile;
//       std::string filename = "/home/rcolares/catkin_ws/src/ros-pioneer3at/maps/abort" + robot_topic +"_"".txt";
//       myfile.open (filename.c_str(),  std::ios::out | std::ios::app );
//       myfile << goal_plan<<"  "<<goals.status_list[0].status<<"\n";
//       myfile.close();
//       last_status = goals.status_list[0].status;
//   }
  
  if(goal_plan==4){
    if(goals.status_list[0].status == 3 || goals.status_list[0].status == 4){
      abortcont++;
      goal_plan=0;
     
    }
  }
}


void ros_other_robot_pose_Callback(ar_track_alvar_msgs::AlvarMarkers alvos){
  
    int y_mine = map_height - ((actual_pose.pose.pose.position.y - map_y)/map_cell);
    int x_mine = (actual_pose.pose.pose.position.x - map_x)/map_cell;
    int y_other =  -1;
    int x_other =  -1;
    int alvo_id;
    double dist;
    double Roll, Pitch, Yaw, TPitch;
    double MRoll, MPitch, MYaw;


    std::ofstream myfile;

    double qx = actual_pose.pose.pose.orientation.x;
    double qy = actual_pose.pose.pose.orientation.y;
    double qz = actual_pose.pose.pose.orientation.z;
    double qw = actual_pose.pose.pose.orientation.w;
    tf::Quaternion qm(qx, qy, qz, qw);
    tf::Matrix3x3 mm(qm);
    mm.getRPY(MRoll, MPitch, MYaw);
  
  
    if(alvos.markers.size()>0){

        for(int i = 0; i<alvos.markers.size();i++){
            int marker_id = alvos.markers[i].id;

            if(marker_id==15){
                double q_x = alvos.markers[i].pose.pose.orientation.x;
                double q_y = alvos.markers[i].pose.pose.orientation.y;
                double q_z = alvos.markers[i].pose.pose.orientation.z;
                double q_w = alvos.markers[i].pose.pose.orientation.w;
                tf::Quaternion q(q_x, q_y, q_z, q_w);
                tf::Matrix3x3 m(q);
                m.getRPY(Roll, Pitch, Yaw);
                std::string filename = "/home/rcolares/catkin_ws/src/ros-pioneer3at/maps/angle_"+robot_topic+".txt";
                myfile.open (filename.c_str(),  std::ios::out | std::ios::app );
                myfile <<Pitch<<"\n";
                myfile.close();
            }

            //if( marker_id == 0 || marker_id == 6 || marker_id ==15 || marker_id == 17){
            if(marker_id == 15 /*&& (Pitch < 0.1 && Pitch > -0.1)/*|| marker_id ==17 || marker_id == 6*/){

                double camera_x = -alvos.markers[i].pose.pose.position.x;
                double camera_z = alvos.markers[i].pose.pose.position.z;
                double d = sqrt(pow(camera_x,2) + pow(camera_z, 2));

                double alpha = atan2(camera_x,camera_z);

                if(camera_x >=0)
                    alpha += MPitch;
                else
                    alpha -= MPitch;

//                y_other = y_mine + (d*cos(alpha))/0.05 + yfactor;
//                x_other = x_mine + (d*sin(alpha))/0.05 + xfactor;

                y_other =  y_mine + alvos.markers[i].pose.pose.position.x/0.05+yfactor;
                x_other =  x_mine + alvos.markers[i].pose.pose.position.z/0.05+xfactor;
                alvo_id = marker_id;

//                y_other =  618;
//                x_other =  544;


//                double q_x = alvos.markers[i].pose.pose.orientation.x;
//                double q_y = alvos.markers[i].pose.pose.orientation.y;
//                double q_z = alvos.markers[i].pose.pose.orientation.z;
//                double q_w = alvos.markers[i].pose.pose.orientation.w;
//                tf::Quaternion q(q_x, q_y, q_z, q_w);
//                tf::Matrix3x3 m(q);
//                m.getRPY(Roll, Pitch, Yaw);


        	
                dist = sqrt((y_mine - y_other)*(y_mine - y_other) + (x_mine - x_other)*(x_mine - x_other));

                std::string filename = "/home/rcolares/catkin_ws/src/ros-pioneer3at/maps/marker_"+robot_topic+".txt";
                myfile.open (filename.c_str(),  std::ios::out | std::ios::app );
                myfile << x_mine <<" "<< y_mine <<" "<<x_other<<" "<<y_other<<" "<<Pitch<<" "<<alvo_id<<"\n";
                myfile.close();





                if(dist>=20 && dist <= 60.0){
                    if(!timer){
                        timer=1;
                        posesWave.dist = dist;
                        posesWave.x_other = x_other;
                        posesWave.y_other = y_other;
                        posesWave.x = x_mine;
                        posesWave.y = y_mine;
                        // 	    std::ofstream myfile;
                        std::string filename = "/home/rcolares/catkin_ws/src/ros-pioneer3at/maps/encounter_"+robot_topic+".txt";
                        myfile.open (filename.c_str(),  std::ios::out | std::ios::app );
                        myfile << x_mine <<"   "<< y_mine <<" ---- "<<x_other<<"   "<<y_other<<"  "<<marker_id<<"\n";
                        myfile.close();
                        goal_plan=5;
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

            }

            break;
        }
    }
    if(x_mine >=0){



        pioneer3at::Poses aux;
        aux.dist = dist;
        aux.x_other = x_other;
        aux.y_other = y_other;
        aux.x = x_mine;
        aux.y = y_mine;
        aux.my_yaw = MYaw;
        aux.yaw = Pitch;
        aux.id = alvo_id;
        frontier_robots_poses.publish(aux);
    }



}



void ros_set_goal_CallBack(nav_msgs::Odometry odometry)
{
    std::ofstream myfile;
    std::string filename = "/home/rcolares/catkin_ws/src/ros-pioneer3at/maps/debug_"+robot_topic+".txt";

    if(goal_plan==1){
        actual_pose = odometry;
        goal_plan=2;
    }else if(goal_plan==3){
    
   
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
        //    goal.goal.target_pose.header.frame_id = map_topic;
        //    goal.goal.target_pose.header.stamp = ros::Time::now();
        //     goal.goal.target_pose.pose.position.x = x;
        //     goal.goal.target_pose.pose.position.y = y;
        //    goal.goal.target_pose.pose.position.x = x + ((goal_cell.x*map_cell)+map_x) - transform.getOrigin().x();
        //    goal.goal.target_pose.pose.position.y = y + (((map_height-goal_cell.y)*map_cell)+map_y) - transform.getOrigin().y();
        //     if(robot_topic[5] == '1'){
        //       goal.goal.target_pose.pose.position.x = x + 2;
        //       goal.goal.target_pose.pose.position.y = y;
        //     }else{
        //       goal.goal.target_pose.pose.position.x = x;
        //       goal.goal.target_pose.pose.position.y = y+2;
        //     }

        //    goal.goal.target_pose.pose.position.z = odometry.pose.pose.position.z;
        //    goal.goal.target_pose.pose.orientation.x = 0.0;
        //    goal.goal.target_pose.pose.orientation.y = 0.0;
        //    goal.goal.target_pose.pose.orientation.z = 1.0;
        //    goal.goal.target_pose.pose.orientation.w = 1.0;

        origin_x = transform.getOrigin().x();
        origin_y = transform.getOrigin().y();
        goal.pose.position.x = x + ((goal_cell.x*map_cell)+map_x) - transform.getOrigin().x();
        goal.pose.position.y = y + (((map_height-goal_cell.y)*map_cell)+map_y) - transform.getOrigin().y();
        //    goal.pose.position.x = x;
        //    goal.pose.position.y = y;
        goal.pose.position.z = odometry.pose.pose.position.z;
        goal.pose.orientation.x = 0.0;
        goal.pose.orientation.y = 0.0;
        goal.pose.orientation.z = 1.0;
        goal.pose.orientation.w = 1.0;


        if(robot_topic[5] == '1'){
            frontier_cmd_vel.publish(goal);
            goal_plan = 4;
        }

        //     std::ofstream myfile;
        //     std::string filename = "/home/rcolares/catkin_ws/src/ros-pioneer3at/maps/goals_" + robot_topic +".txt";
        //     myfile.open (filename.c_str(),  std::ios::out | std::ios::app );
        //     myfile << goal.goal.target_pose.pose.position.x <<"   "<< goal.goal.target_pose.pose.position.y <<"\n";

    }else if(goal_plan==4){

        double y = odometry.pose.pose.position.y;
        double x = odometry.pose.pose.position.x;
        //    double xd = goal.goal.target_pose.pose.position.x;
        //    double yd = goal.goal.target_pose.pose.position.y;
        double xd = goal.pose.position.x;
        double yd = goal.pose.position.y;
        actual_pose = odometry;

        if( sqrt((x-xd)*(x-xd) + (y-yd)*(y-yd)) < 2.0){

            goal_plan = 0;
        }


    }else if(goal_plan==5){
        double y = odometry.pose.pose.position.y;
        double x = odometry.pose.pose.position.x;
        goal.header.frame_id = map_topic;
        goal.header.stamp = ros::Time::now();
//        goal.pose.position.x = x + ((goal_cell.x*map_cell)+map_x) - transform.getOrigin().x();
//        goal.pose.position.y = y + (((map_height-goal_cell.y)*map_cell)+map_y) - transform.getOrigin().y();
        goal.pose.position.x = x;
        goal.pose.position.y = y;
        goal.pose.position.z = odometry.pose.pose.position.z;
        goal.pose.orientation.x = 0.0;
        goal.pose.orientation.y = 0.0;
        goal.pose.orientation.z = 1.0;
        goal.pose.orientation.w = 1.0;


        if(robot_topic[5] == '1'){
            frontier_cmd_vel.publish(goal);
//            goal_plan = 4;
        }
    }else if(goal_plan==6){
        double y = odometry.pose.pose.position.y;
        double x = odometry.pose.pose.position.x;
        goal.header.frame_id = map_topic;
        goal.header.stamp = ros::Time::now();
//        goal.pose.position.x = x + ((500*map_cell)+map_x) - origin_x;
//        goal.pose.position.y = y + (((map_height-500)*map_cell)+map_y) - origin_y;
        goal.pose.position.x = 0;
        goal.pose.position.y = 0;
        goal.pose.position.z = odometry.pose.pose.position.z;
        goal.pose.orientation.x = 0.0;
        goal.pose.orientation.y = 0.0;
        goal.pose.orientation.z = 1.0;
        goal.pose.orientation.w = 1.0;


        if(robot_topic[5] == '1'){
            frontier_cmd_vel.publish(goal);
            //            goal_plan = 4;
        }
    }
}

void ros_save_occ_map_Callback(pioneer3at::OccMap occ_map){
  
  int width = occ_map.map.info.width;
  int height = occ_map.map.info.height;
  map_x = occ_map.map.info.origin.position.x;
  map_y = occ_map.map.info.origin.position.y;
  map_cell = occ_map.map.info.resolution;
  map_height = height;
  map_width = width;
  
//  mapSaver(occ_map.map, "/home/rcolares/catkin_ws/src/ros-pioneer3at/maps/", id++);


//  std::ofstream myfile;
//  std::string filename = "/home/rcolares/catkin_ws/src/ros-pioneer3at/maps/debug_"+robot_topic+".txt";
  
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
    
    createFrontiers(width, height, real_map, mapa, occ_real_map, robot_topic);

      goal_plan=1;
  }else if(goal_plan==2){
      
    int y = map_height - ((actual_pose.pose.pose.position.y - map_y)/map_cell);
    int x = (actual_pose.pose.pose.position.x - map_x)/map_cell;
    
       
//     std::ofstream myfile;
//     std::string filename = "/home/rcolares/catkin_ws/src/ros-pioneer3at/maps/wavepose" + robot_topic +".txt";
//     myfile.open (filename.c_str(),  std::ios::out | std::ios::app );
//     myfile << posesWave.x_other <<"   "<< posesWave.y_other <<"\n";
//     myfile.close();
    


    multi_wavefront(width, height, real_map, wave_map, mapa, x, y, posesWave.x_other, posesWave.y_other, robot_topic);


    chooseGoal(mapa, wave_map, ww_map, width, height, robot_topic);
    


    if(!goal_cell.valid || abortcont == 3){
      std::ofstream myfile;
      std::string filename = "/home/rcolares/catkin_ws/src/ros-pioneer3at/maps/time_"+robot_topic+".txt";
      myfile.open (filename.c_str(),  std::ios::out | std::ios::app );
      myfile << ros::Time::now();
      myfile.close();
      std::string cmd = "rosnode kill Pioneer3AT_window_explorer";
      system(cmd.c_str());
    }


    mapSaver(occ_map.map, "/home/rcolares/catkin_ws/src/ros-pioneer3at/maps/", id++);
    
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
  n_.getParam("robots_poses_topic", robots_poses_topic);
  n_.getParam("other_robot_pose_topic", other_robot_pose_topic);
  n_.getParam("signal_topic", signal_topic);
  
  
  std::ofstream myfile;
  std::string filename = "/home/rcolares/catkin_ws/src/ros-pioneer3at/maps/time_"+robot_topic+".txt";
  myfile.open (filename.c_str(),  std::ios::out | std::ios::app );
  myfile << ros::Time::now()<<"\n";
  myfile.close();
  
  if(goal_plan==-1){
    posesWave.x_other = -1;
    posesWave.y_other = -1;
    goal_plan = 0;
  }
    
  frontier_cmd_vel = n.advertise<geometry_msgs::PoseStamped>(goal_topic, 1);
//  frontier_cmd_vel = n.advertise<move_base_msgs::MoveBaseActionGoal>(goal_topic, 1);
  frontier_robots_poses = n.advertise<pioneer3at::Poses>(robots_poses_topic,1);
  
  frontier_occ_map = n.subscribe(occ_map_topic, 1, ros_save_occ_map_Callback);
  frontier_goal_status = n.subscribe(goal_status_topic, 1, ros_goal_status_Callback);
  frontier_pose = n.subscribe(pose_topic, 1, ros_set_goal_CallBack);
  frontier_other_robot_pose = n.subscribe(other_robot_pose_topic, 1, ros_other_robot_pose_Callback);
  map_node_signal_rcv = n.subscribe(signal_topic, 1 , ros_signal_Callback);
  ros::spin();
  
}
