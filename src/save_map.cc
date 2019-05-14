#include "gmapping/occMap.h"

void save_map_simple(gmapping::occMap map, std::string robot)
{
    ROS_INFO("Received a %d X %d map @ %.3f m/pix",
                   map.map.info.width,
                   map.map.info.height,
                   map.map.info.resolution);


          std::string mapdatafile = "/home/colares/catkin_ws/src/integrated-exploration/maps" + robot + "map.pgm";
          ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());
          FILE* out = fopen(mapdatafile.c_str(), "w");
          if (!out)
          {
            ROS_ERROR("Couldn't save map file to %s", mapdatafile.c_str());
            return;
          }

          fprintf(out, "P5\n# CREATOR: Map_generator.cpp %.3f m/pix\n%d %d\n255\n",
                  map.map.info.resolution, map.map.info.width, map.map.info.height);
          for(unsigned int y = 0; y < map.map.info.height; y++) {
            for(unsigned int x = 0; x < map.map.info.width; x++) {
              unsigned int i = x + (map.map.info.height - y - 1) * map.map.info.width;
              if (map.map.data[i] == 0) { //occ [0,0.1)
                fputc(254, out);
              } else if (map.map.data[i] == +100) { //occ (0.65,1]
                fputc(000, out);
              } else { //occ [0.1,0.65]
                fputc(205, out);
              }
            }
          }

          fclose(out);

          ROS_INFO("Done\n");
}


void save_map_pose(gmapping::occMap map, mapPose m_pose, std::string robot, int count)
{
    ROS_INFO("Received a %d X %d map @ %.3f m/pix",
             map.map.info.width,
             map.map.info.height,
             map.map.info.resolution);
    char sysCall[512];

    std::string mapdatafile = "/home/colares/catkin_ws/src/integrated-exploration/maps" + robot + "pose" + boost::to_string(count);
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
            else if (y == 0 && x == 0)
                fprintf(printFile, "%c%c%c", 0, 0, 255);
            else if (i == 0)
                fprintf(printFile, "%c%c%c", 0, 255, 0);
            else if (i==10)
                fprintf(printFile, "%c%c%c", 255, 0, 0);    
            else if (map.data[i] == 0)  //occ [0,0.1)
                fprintf(printFile, "%c%c%c", 255, 255, 255);
            else if (map.data[i] == +100)  //occ (0.65,1]
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

void save_map_goal(gmapping::occMap map, mapPose m_pose, mapPose m_goal, std::string robot, int count)
{
    ROS_INFO("Received a %d X %d map @ %.3f m/pix",
             map.map.info.width,
             map.map.info.height,
             map.map.info.resolution);
    char sysCall[512];

    std::string mapdatafile = "/home/colares/catkin_ws/src/integrated-exploration/maps" + robot + "goal" + boost::to_string(count);
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
            else if((y >= m_goal.y - 5 && y <= m_goal.y + 5) && (x >= m_goal.x - 5 && x <= m_goal.x + 5))
              fprintf(printFile, "%c%c%c", 255, 0, 255);
            else if (y == 0 && x == 0)
              fprintf(printFile, "%c%c%c", 0, 0, 255);
            else if (i == 0)
              fprintf(printFile, "%c%c%c", 0, 255, 0);
            else if (i==10)
              fprintf(printFile, "%c%c%c", 255, 0, 0);    
            else if (map.data[i] == 0)  //occ [0,0.1)
              fprintf(printFile, "%c%c%c", 255, 255, 255);
            else if (map.data[i] == +100)  //occ (0.65,1]
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


