#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/String.h>
#include <nav_msgs/GetMap.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <ros/console.h>
// #include "pioneer3at/mapStruct.h"
#include <cstring>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <algorithm>


#include "map_merge.cc"
#include "pioneer3at/OccMap.h"
#include "pioneer3at/Poses.h"
#include "pioneer3at/Signal.h"

#define NUM_ROBOTS 2
#define MAP_SIZE 1001

ros::Publisher map_publisher;
ros::Publisher occ_map_publisher;
ros::Publisher signal_publisher;
ros::Subscriber occ_map_subscriber;
ros::Subscriber other_robot_subscriber;
ros::Subscriber other_map_subscriber;
ros::Subscriber robots_pose_subscriber;
ros::Subscriber other_robot_pose_subscriber;

std::string map_topic;
std::string occ_map_topic;
std::string occ_gmapping_map_topic;
std::string robot_topic;
std::string other_robot_topic;
std::string other_map_topic;
std::string robots_pose_topic;
std::string other_robot_pose_topic;
std::string other_robot;
std::string signal_topic;
int robot_id;

pioneer3at::OccMap actual_map;
pioneer3at::OccMap other_map;


pioneer3at::Poses robotPose;
pioneer3at::Poses other_robotPose;
pioneer3at::Poses startPose;
pioneer3at::Signal sig;

// int **real_map;
// int **real_map_other;
// int **real_map_merge;

int width_map;
int height_map;
int width_other;
int height_other;
int first =1;
int flag_topic = 0;
int flag_pose=0;
int flag_map=0;
int flag_merge=0;
int timer=0;
double start_timer;



double origin_y, origin_x, initial_height, initial_width, resolution;


void mapTransform(nav_msgs::OccupancyGrid map,  int width, int height, int **real_map, int i){
  for(int l=0;l<MAP_SIZE;l++){
    for(int k=0;k<MAP_SIZE;k++){
      if(l>=height || k >= width){
        real_map[l][k] = -1;
      }else{
        int t = (height - l -1)*width +k;
        real_map[l][k] = map.data[t];
      }
    }
  }

//   char name[512];
//   sprintf ( name, "mapf_%d", i);
//   std::string name = "final_map"+number;
//   savemap(real_map, MAP_SIZE, MAP_SIZE, name);
}

void occ_mapTransform(pioneer3at::OccMap occ_map, double **map){
  for(int l=0;l<MAP_SIZE;l++){
    for(int k=0;k<MAP_SIZE;k++){
      if(l>=occ_map.map.info.height || k >= occ_map.map.info.width){
        map[l][k] = -1;
      }else{
        int t = (occ_map.map.info.height - l -1)*occ_map.map.info.width +k;
        map[l][k] = occ_map.data[t];
      }
    }
  }
}

// std::ofstream myfile;
//   std::string filename = "/home/rafael/catkin_ws/src/ros-pioneer3at/maps/teste";
//   filename+=map_topic[6];
//   filename+=".txt";
//   myfile.open (filename.c_str(),  std::ios::out);/* | std::ios::app );*/



void map2gm(int **map){
  nav_msgs::OccupancyGrid aux;
  aux.header = actual_map.map.header;
  aux.info = actual_map.map.info;
//   aux.info.height = MAP_SIZE;
//   aux.info.width = MAP_SIZE;
  int t=0;
  aux.data.resize(MAP_SIZE*MAP_SIZE);
  for(int l=MAP_SIZE-1;l>=0;l--)
    for(int k=0;k<MAP_SIZE;k++)
      aux.data[t++] = map[l][k];

  actual_map.map = aux;
}

void occmap2gm(double **map){
  int t=0;
  actual_map.data.clear();
  actual_map.data.resize(MAP_SIZE*MAP_SIZE);
  for(int l=MAP_SIZE-1;l>=0;l--)
    for(int k=0;k<MAP_SIZE;k++)
      actual_map.data[t++] = map[l][k];
}

void mergeSameMap(pioneer3at::OccMap new_map){

  int **map1, **map2, **map;
  double  **occ_map1, **occ_map2, **occ_map;

  map1 = (int**) malloc(MAP_SIZE*sizeof(int*));
  map2 = (int**) malloc(MAP_SIZE*sizeof(int*));
  map = (int**) malloc(MAP_SIZE*sizeof(int*));
  occ_map1 = (double**) malloc(MAP_SIZE*sizeof(double*));
  occ_map2 = (double**) malloc(MAP_SIZE*sizeof(double*));
  occ_map = (double**) malloc(MAP_SIZE*sizeof(double*));
  for(int i=0; i<MAP_SIZE; i++){
    map1[i] = (int*) malloc(MAP_SIZE*sizeof(int*));
    map2[i] = (int*) malloc(MAP_SIZE*sizeof(int*));
    map[i] = (int*) malloc(MAP_SIZE*sizeof(int*));
    occ_map1[i] = (double*) malloc(MAP_SIZE*sizeof(double*));
    occ_map2[i] = (double*) malloc(MAP_SIZE*sizeof(double*));
    occ_map[i] = (double*) malloc(MAP_SIZE*sizeof(double*));
  }

  mapTransform(new_map.map, new_map.map.info.width, new_map.map.info.height, map1, 1);
  mapTransform(actual_map.map, actual_map.map.info.width, actual_map.map.info.height, map2, 2);

  occ_mapTransform(new_map, occ_map1);
  occ_mapTransform(actual_map, occ_map2);

  stitchsamemap(map, map1, map2, occ_map, occ_map1, occ_map2, MAP_SIZE, MAP_SIZE, map_topic[6]);

  map2gm(map);
  occmap2gm(occ_map);

  for(int i=0; i<MAP_SIZE; i++){
    free(map1[i]);
    free(map2[i]);
    free(map[i]);
    free(occ_map1[i]);
    free(occ_map2[i]);
    free(occ_map[i]);
  }
  free(map1);
  free(map2);
  free(map);
  free(occ_map1);
  free(occ_map2);
  free(occ_map);

}

void mergeMaps(){
  int **map1, **map2, **map;
  double  **occ_map1, **occ_map2, **occ_map;

  map1 = (int**) malloc(MAP_SIZE*sizeof(int*));
  map2 = (int**) malloc(MAP_SIZE*sizeof(int*));
  map = (int**) malloc(MAP_SIZE*sizeof(int*));
  occ_map1 = (double**) malloc(MAP_SIZE*sizeof(double*));
  occ_map2 = (double**) malloc(MAP_SIZE*sizeof(double*));
  occ_map = (double**) malloc(MAP_SIZE*sizeof(double*));
  for(int i=0; i<MAP_SIZE; i++){
    map1[i] = (int*) malloc(MAP_SIZE*sizeof(int*));
    map2[i] = (int*) malloc(MAP_SIZE*sizeof(int*));
    map[i] = (int*) malloc(MAP_SIZE*sizeof(int*));
    occ_map1[i] = (double*) malloc(MAP_SIZE*sizeof(double*));
    occ_map2[i] = (double*) malloc(MAP_SIZE*sizeof(double*));
    occ_map[i] = (double*) malloc(MAP_SIZE*sizeof(double*));
  }

  mapTransform(actual_map.map, actual_map.map.info.width, actual_map.map.info.height, map1, 1);
  mapTransform(other_map.map, other_map.map.info.width, other_map.map.info.height, map2, 2);

  occ_mapTransform(actual_map, occ_map1);
  occ_mapTransform(other_map, occ_map2);

//   int h = compute_similarity(map1, map2, WIDTH, HEIGHT);
//   int agr = agreed(map1, map2, r1, r2);
//   int dis = disagreed(map1, map2, r1, r2);
//   double indicator = ((double)agr/((double)agr+dis));

  stitchmap(map, map1, map2, occ_map, occ_map1, occ_map2, robotPose, other_robotPose, MAP_SIZE, MAP_SIZE, map_topic[6]);

  map2gm(map);
  occmap2gm(occ_map);

  for(int i=0; i<MAP_SIZE; i++){
    free(map1[i]);
    free(map2[i]);
    free(map[i]);
    free(occ_map1[i]);
    free(occ_map2[i]);
    free(occ_map[i]);
  }
  free(map1);
  free(map2);
  free(map);
  free(occ_map1);
  free(occ_map2);
  free(occ_map);
  flag_merge=0;
  flag_pose=0;
  flag_map=0;
  flag_topic=0;

}

void ros_other_robot_Callback(std::string topic){

    if(other_robot.size()>0 && flag_topic ==0){
        other_map_topic = topic+other_map_topic;;
        robots_pose_topic = robot_topic+robots_pose_topic;
        other_robot_pose_topic = topic+other_robot_pose_topic;
        flag_topic = 1;
    }


}


void ros_robots_pose_Callback(pioneer3at::Poses aux){

        robotPose = aux;
        if(aux.dist>=20.0 && aux.dist<=150.0 && aux.id >0){
            if(!timer){
                startPose.x = aux.x;
                startPose.y = aux.y;
                char number[10];
                sprintf ( number, "%d", aux.id );
                std::string s(number);

                robots_pose_topic = "/robot"+ s + robots_pose_topic;
                other_robot_pose_topic = "/robot"+ s + other_robot_pose_topic;
                timer=1;
                flag_pose=1;
            }
        }
        if(sqrt(pow((aux.x - startPose.x),2) - pow((aux.y - startPose.y),2)) > 100)
            timer=0;
}



void ros_other_pose_Callback(pioneer3at::Poses aux){
  if(flag_map==1){
    other_robotPose = aux;
    flag_merge=1;
  }
 }

void ros_other_map_Callback(pioneer3at::OccMap other_robot_map){
  if(flag_pose==1){
    other_map = other_robot_map;
    flag_map=1;

  }
}

pioneer3at::OccMap enlargeMap(pioneer3at::OccMap occ_map){

   int **gmap, **smap;
  pioneer3at::OccMap aux;

  aux.map.header = occ_map.map.header;
  aux.map.info = occ_map.map.info;
  aux.map.info.height = MAP_SIZE;
  aux.map.info.width = MAP_SIZE;
  aux.map.info.origin.position.y = -MAP_SIZE*resolution/2;
  aux.map.info.origin.position.x = -MAP_SIZE*resolution/2;
  aux.map.data.resize(MAP_SIZE*MAP_SIZE);
  std::fill(aux.map.data.begin(), aux.map.data.end(), -1);
  aux.data.resize(MAP_SIZE*MAP_SIZE);
  std::fill(aux.data.begin(), aux.data.end(), -1);

  int t, g;
  for(int i = 0; i < initial_height; i++){
    for(int e = 0; e < initial_width; e++){
      t = (initial_height - (initial_height -i))*initial_width + e;
      g = (MAP_SIZE - (MAP_SIZE/2 - origin_y/resolution - i)- 1)*MAP_SIZE + (MAP_SIZE/2 + origin_x/resolution + e);
      aux.map.data[g] = occ_map.map.data[t];
      aux.data[g] = occ_map.data[t];
    }
  }

  return aux;

}





void ros_occ_map_Callback(pioneer3at::OccMap occ_map){

  pioneer3at::OccMap aux;

  origin_y = occ_map.map.info.origin.position.y;
  origin_x = occ_map.map.info.origin.position.x;
  initial_height = occ_map.map.info.height;
  initial_width = occ_map.map.info.width;
  resolution = occ_map.map.info.resolution;
  sig.sig = 0;
  if(first){
    aux = enlargeMap(occ_map);
    actual_map = aux;
    first=0;
  }else if(flag_merge==1){
    aux = enlargeMap(occ_map);
    mergeSameMap(aux);
    mergeMaps();
    sig.sig = 1;
  }else{
    aux = enlargeMap(occ_map);
    mergeSameMap(aux);
  }

  map_publisher.publish(actual_map.map);
  occ_map_publisher.publish(actual_map);
  signal_publisher.publish(sig);
}







int main( int argc, char* argv[] )
{

  // Initialize ROS
  ros::init(argc, argv, "Map_node");
  ros::NodeHandle n;
  ros::NodeHandle n_("~");


  n_.getParam("robot_id", robot_id);
  n_.getParam("map_topic", map_topic);
  n_.getParam("occ_map_topic", occ_map_topic);
  n_.getParam("occ_gmapping_map_topic", occ_gmapping_map_topic);
  n_.getParam("robot_topic", robot_topic);
  n_.getParam("other_map_topic", other_map_topic);
  n_.getParam("robots_pose_topic", robots_pose_topic);
  n_.getParam("other_robot_pose_topic", other_robot_pose_topic);
  n_.getParam("signal_topic", signal_topic);

//  signal_publisher = n.advertise<pioneer3at::Signal>(signal_topic, 1);
  ros::Rate r(10);
  while(1){
      map_publisher = n.advertise<nav_msgs::OccupancyGrid>(map_topic, 1);
      occ_map_publisher = n.advertise<pioneer3at::OccMap>(occ_map_topic, 1);
      occ_map_subscriber = n.subscribe(occ_gmapping_map_topic, 1, ros_occ_map_Callback);
      //other_robot_subscriber = n.subscribe(other_robot_topic, 1, ros_other_robot_Callback);
      robots_pose_subscriber = n.subscribe(robots_pose_topic, 1, ros_robots_pose_Callback);
      if(flag_pose==1){
          other_map_subscriber = n.subscribe(other_map_topic, 1, ros_other_map_Callback);

          other_robot_pose_subscriber = n.subscribe(other_robot_pose_topic, 1, ros_other_pose_Callback);
      }
      ros::spinOnce();
  }

}
