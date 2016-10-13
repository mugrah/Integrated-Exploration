#include "gmapping/occMap.h"


void save_map_simple(gmapping::occMap map)
{
    ROS_INFO("Received a %d X %d map @ %.3f m/pix",
                   map.map.info.width,
                   map.map.info.height,
                   map.map.info.resolution);


          std::string mapdatafile = "/home/rcolares/catkin_ws/src/ros-pioneer3at/maps/map.pgm";
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


          std::string mapmetadatafile = "/home/rcolares/catkin_ws/src/ros-pioneer3at/maps/map.yaml";
          ROS_INFO("Writing map occupancy data to %s", mapmetadatafile.c_str());
          FILE* yaml = fopen(mapmetadatafile.c_str(), "w");


          /*
    resolution: 0.100000
    origin: [0.000000, 0.000000, 0.000000]
    #
    negate: 0
    occupied_thresh: 0.65
    free_thresh: 0.196
           */

          geometry_msgs::Quaternion orientation = map.map.info.origin.orientation;
          tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
          double yaw, pitch, roll;
          mat.getEulerYPR(yaw, pitch, roll);

          fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
                  mapdatafile.c_str(), map.map.info.resolution, map.map.info.origin.position.x, map.map.info.origin.position.y, yaw);

          fclose(yaml);

          ROS_INFO("Done\n");



}


