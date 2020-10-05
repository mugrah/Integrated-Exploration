#include "pioneer3at/OccMap.h"
#include "ros/package.h"



void save_map_simple(pioneer3at::OccMap map, std::string robot, int count)
{
  ROS_INFO("Received a %d X %d map @ %.3f m/pix",
            map.map.info.width,
            map.map.info.height,
            map.map.info.resolution);
  char sysCall[512];
  std::string package = ros::package::getPath("pioneer3at");
  std::string mapdatafile = package + "/maps/" + robot + "_" + boost::to_string(count);
  ROS_ERROR_STREAM(mapdatafile);
  sprintf(sysCall, "%s.ppm", mapdatafile.c_str());
  FILE* printFile = fopen(sysCall, "w");
  fprintf(printFile, "P6\n # particles.ppm \n %d %d\n", map.map.info.width, map.map.info.height);
  fprintf(printFile, "255\n");

  for(unsigned int y = 0; y < map.map.info.height; y++) {
      for(unsigned int x = 0; x < map.map.info.width; x++) {
          unsigned int i = x + (map.map.info.height - y - 1) * map.map.info.width;
          if (map.map.data[i] == 0)  //occ [0,0.1)
              fprintf(printFile, "%c%c%c", 255, 255, 255);
          else if (map.map.data[i] == +100)  //occ (0.65,1]
              fprintf(printFile, "%c%c%c", 0, 0, 0);
          else  //occ [0.1,0.65]
              fprintf(printFile, "%c%c%c", 125, 125, 125);
          
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


  ROS_INFO("Done\n");
}


void save_map_pose(pioneer3at::OccMap map, mapPose m_pose, std::string robot, int count)
{
  ROS_INFO("Received a %d X %d map @ %.3f m/pix",
            map.map.info.width,
            map.map.info.height,
            map.map.info.resolution);
  char sysCall[512];
  std::string package = ros::package::getPath("pioneer3at");
  std::string mapdatafile = package + "/maps/" + robot + "_" + boost::to_string(count);
  sprintf(sysCall, "%s.ppm", mapdatafile.c_str());
  FILE* printFile = fopen(sysCall, "w");
  fprintf(printFile, "P6\n # particles.ppm \n %d %d\n", map.map.info.width, map.map.info.height);
  fprintf(printFile, "255\n");

  for(unsigned int y = 0; y < map.map.info.height; y++) {
      for(unsigned int x = 0; x < map.map.info.width; x++) {
          unsigned int i = x + (map.map.info.height - y - 1) * map.map.info.width;
          // if( y == m_pose.y && x == m_pose.x)
          if((y >= m_pose.y - 3 && y <= m_pose.y + 3) && (x >= m_pose.x - 3 && x <= m_pose.x + 3))
              fprintf(printFile, "%c%c%c", 255, 0, 0);   
          else if (map.map.data[i] == 0)  //occ [0,0.1)
              fprintf(printFile, "%c%c%c", 255, 255, 255);
          else if (map.map.data[i] == +100)  //occ (0.65,1]
              fprintf(printFile, "%c%c%c", 0, 0, 0);
          else  //occ [0.1,0.65]
              fprintf(printFile, "%c%c%c", 125, 125, 125);
          
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


  ROS_INFO("Done\n");
}

void save_map_goal(pioneer3at::OccMap map, mapPose m_pose, mapPose m_goal, std::string robot, int count)
{
  ROS_INFO("Received a %d X %d map @ %.3f m/pix",
            map.map.info.width,
            map.map.info.height,
            map.map.info.resolution);
  char sysCall[512];
  std::string package = ros::package::getPath("pioneer3at");
  std::string mapdatafile = package + "/maps" + robot + "goal" + boost::to_string(count);
  sprintf(sysCall, "%s.ppm", mapdatafile.c_str());
  FILE* printFile = fopen(sysCall, "w");
  fprintf(printFile, "P6\n # particles.ppm \n %d %d\n", map.map.info.width, map.map.info.height);
  fprintf(printFile, "255\n");

  for(unsigned int y = 0; y < map.map.info.height; y++) {
      for(unsigned int x = 0; x < map.map.info.width; x++) {
          unsigned int i = x + (map.map.info.height - y - 1) * map.map.info.width;
          if( y == m_pose.y && x == m_pose.x)
          // if((y >= m_pose.y - 5 && y <= m_pose.y + 5) && (x >= m_pose.x - 5 && x <= m_pose.x + 5))
            fprintf(printFile, "%c%c%c", 255, 0, 0);
          // else if((y >= m_goal.y - 5 && y <= m_goal.y + 5) && (x >= m_goal.x - 5 && x <= m_goal.x + 5))
          else if((y == m_goal.y) && (x == m_goal.x))
            fprintf(printFile, "%c%c%c", 255, 0, 255);
          else if (y == 0 && x == 0)
            fprintf(printFile, "%c%c%c", 0, 0, 255);
          else if (i == 0)
            fprintf(printFile, "%c%c%c", 0, 255, 0);
          else if (i==10)
            fprintf(printFile, "%c%c%c", 255, 0, 0);    
          else if (map.data[i] == 0)  //occ [0,0.1)
            fprintf(printFile, "%c%c%c", 255, 255, 255);
          else if (map.map.data[i] == +100)  //occ (0.65,1]
            fprintf(printFile, "%c%c%c", 0, 0, 0);
          else  //occ [0.1,0.65]
            fprintf(printFile, "%c%c%c", 125, 125, 125);
          
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


  ROS_INFO("Done\n");
}


