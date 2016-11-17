#include "gmapping/occMap.h"

void save_map_simple(gmapping::occMap map, std::string robot)
{
    ROS_INFO("Received a %d X %d map @ %.3f m/pix",
                   map.map.info.width,
                   map.map.info.height,
                   map.map.info.resolution);


          std::string mapdatafile = "/home/rcolares/catkin_ws/src/ros-pioneer3at/maps/" + robot + "map.pgm";
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


