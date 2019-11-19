#include <vector>
#include <cstdlib>

#include "frontier_tools.h"
#include "pioneer3at/OccMap.h"


int verify_frontier_vector(unsigned i, std::vector<frontier_data> frontier_vector){
    for(int k=0;k<frontier_vector.size();k++){
        if( i == frontier_vector[k].center)
            return 1;
    }
    return 0;
}

void save_frontiers(pioneer3at::OccMap map, std::vector<frontier_data> frontier_vector)
{
  ROS_INFO("Received a %d X %d map @ %.3f m/pix",
            map.map.info.width,
            map.map.info.height,
            map.map.info.resolution);
  char sysCall[512];
  std::string package = ros::package::getPath("pioneer3at");
  std::string mapdatafile = package + "/maps/frontiers";
  sprintf(sysCall, "%s.ppm", mapdatafile.c_str());
  FILE* printFile = fopen(sysCall, "w");
  fprintf(printFile, "P6\n # particles.ppm \n %d %d\n", map.map.info.width, map.map.info.height);
  fprintf(printFile, "255\n");

  for(unsigned int y = 0; y < map.map.info.height; y++) {
      for(unsigned int x = 0; x < map.map.info.width; x++) {
          unsigned int i = x + (map.map.info.height - y - 1) * map.map.info.width;
          if(verify_frontier_vector(i, frontier_vector))
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


int check_if_frontier(pioneer3at::OccMap *map, unsigned int x, unsigned int y){
    int flag = 0;
    unsigned int it;
    int height = map->map.info.height;
    int width = map->map.info.width;

    it = x-1 + (height - y-1 - 1) * width;
    // ROS_ERROR_STREAM(map->data[it]);
    if((map->data[it] > 0 && map->data[it] < 1) || (map->data[it] == -1))
        flag = 1;
    else if (map->map.data[it] == +100)
        return 0;

    it = x-1 + (height - y - 1) * width;
    if((map->data[it] > 0 && map->data[it] < 1) || (map->data[it] == -1))
        flag = 1;
    else if (map->map.data[it] == +100)
        return 0;

    it = x-1 + (height - y+1 - 1) * width;
    if((map->data[it] > 0 && map->data[it] < 1) || (map->data[it] == -1))
        flag = 1;
    else if (map->map.data[it] == +100)
        return 0;

    it = x + (height - y-1 - 1) * width;
    if((map->data[it] > 0 && map->data[it] < 1) || (map->data[it] == -1))
        flag = 1;
    else if (map->map.data[it] == +100)
        return 0;

    it = x + (height - y+1 - 1) * width;
    if((map->data[it] > 0 && map->data[it] < 1) || (map->data[it] == -1))
        flag = 1;
    else if (map->map.data[it] == +100)
        return 0;

    it = x+1 + (height - y-1 - 1) * width;
    if((map->data[it] > 0 && map->data[it] < 1) || (map->data[it] == -1))
        flag = 1;
    else if (map->map.data[it] == +100)
        return 0;

    it = x+1 + (height - y - 1) * width;
    if((map->data[it] > 0 && map->data[it] < 1) || (map->data[it] == -1))
        flag = 1;
    else if (map->map.data[it] == +100)
        return 0;

    it = x+1 + (height - y+1 - 1) * width;
    if((map->data[it] > 0 && map->data[it] < 1) || (map->data[it] == -1))
        flag = 1;
    else if (map->map.data[it] == +100)
        return 0;
      
    return flag;
}

std::vector<frontier_data> createFrontiers(pioneer3at::OccMap *map){
  
    int flag;
    int height = map->map.info.height;
    int width = map->map.info.width;    

    std::vector<frontier_data> frontier_vector;

    for(unsigned int y = 1; y < height-1; y++) {
        for(unsigned int x = 1; x < width-1; x++) {
            unsigned int i = x + (height - y - 1) * width;
            if(map->map.data[i] == 0){
                if(check_if_frontier(map, x, y)){
                    if(frontier_vector.size()==0){
                        frontier_data aux;
                        aux.num = 1;
                        aux.x.push_back(x);
                        aux.y.push_back(y);
                        frontier_vector.push_back(aux);
                    }else{
                        flag=0;
                        for(int k = 0; k < frontier_vector.size(); k++){
                            for(int j = 0; j < frontier_vector[k].x.size(); j++){
                                float dist = sqrt((frontier_vector[k].y[j]-y)*(frontier_vector[k].y[j]-y) + (frontier_vector[k].x[j]-x)*(frontier_vector[k].x[j]-x));
                                if(dist<=7){
                                    frontier_vector[k].num+=1;
                                    frontier_vector[k].x.push_back(x);
                                    frontier_vector[k].y.push_back(y);
                                    flag=1;
                                    break;
                                }
                            }
                            if(flag==1)
                                break;
                        }
                        if(flag==0){
                            frontier_data aux;
                            aux.num = 1;
                            aux.x.push_back(x);
                            aux.y.push_back(y);
                            frontier_vector.push_back(aux);
                        }
                    }
                }
            }
        }
    }
    int j=0;
    while(j<frontier_vector.size()){
        if(frontier_vector[j].num<30)
            frontier_vector.erase(frontier_vector.begin()+j);
        else
            j++;
    }

    double dist;
    double mindist;
    unsigned int it;
    for(int k=0;k<frontier_vector.size();k++){
        mindist=-1;

        for(int i = 0; i<frontier_vector[k].x.size(); i++){
            dist = 0;

            for(int e = 0; e<frontier_vector[k].x.size(); e++){
                if(i != e)
                    dist += sqrt((frontier_vector[k].x[i] - frontier_vector[k].x[e])*(frontier_vector[k].x[i] - frontier_vector[k].x[e]) + (frontier_vector[k].y[i] - frontier_vector[k].y[e])*(frontier_vector[k].y[i] - frontier_vector[k].y[e]));
            }

            it = frontier_vector[k].x[i] + (height - frontier_vector[k].y[i] - 1) * width;
            
            if((mindist==-1 || dist<mindist) && (map->map.data[it] == 0)){
                mindist = dist;
                frontier_vector[k].min_dist = i;
                frontier_vector[k].x_mean = frontier_vector[k].x[i];
                frontier_vector[k].y_mean = frontier_vector[k].y[i];
                frontier_vector[k].center = frontier_vector[k].x_mean + (height - frontier_vector[k].y_mean - 1) * width;

            }
        }
    }
    save_frontiers(*map, frontier_vector);

    return frontier_vector;

}

