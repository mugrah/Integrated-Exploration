#include <vector>
#include <cstdlib>

#include "frontier_tools.h"
#include "pioneer3at/OccMap.h"


void mapTransform(nav_msgs::OccupancyGrid map,  int width, int height, int **real_map, int **mapa){
  for(int l=0;l<height;l++){
    for(int k=0;k<width;k++){
//        std::ofstream myfile;
//   std::string filename = "/home/rafael/catkin_ws/src/ros-pioneer3at/maps/teste.txt";
//   myfile.open (filename.c_str(),  std::ios::out | std::ios::app );
//   myfile << l<<" "<<k<<"\n";
//   myfile.close();
      int t = (height - l -1)*width +k;
      real_map[l][k] = map.data[t];
      if(real_map[l][k] == +100)
        mapa[l][k] = 1;
      else
        mapa[l][k]=-1;
    }
  }
}


int check_if_frontier(pioneer3at::OccMap *map, unsigned int x, unsigned int y){
    int flag = 0;
    unsigned int it;
    int height = map->map.info.height;
    int width = map->map.info.width;

    it = x-1 + (height - y-1 - 1) * width;
    if(map->map.data[it] == 0)
        flag = 1;
    else if (map->map.data[it] == +100)
        return 0;

    it = x-1 + (height - y - 1) * width;
    if(map->map.data[it] == 0)
        flag = 1;
    else if (map->map.data[it] == +100)
        return 0;

    it = x-1 + (height - y+1 - 1) * width;
    if(map->map.data[it] == 0)
        flag = 1;
    else if (map->map.data[it] == +100)
        return 0;

    it = x + (height - y-1 - 1) * width;
    if(map->map.data[it] == 0)
        flag = 1;
    else if (map->map.data[it] == +100)
        return 0;

    it = x + (height - y+1 - 1) * width;
    if(map->map.data[it] == 0)
        flag = 1;
    else if (map->map.data[it] == +100)
        return 0;

    it = x+1 + (height - y-1 - 1) * width;
    if(map->map.data[it] == 0)
        flag = 1;
    else if (map->map.data[it] == +100)
        return 0;

    it = x+1 + (height - y - 1) * width;
    if(map->map.data[it] == 0)
        flag = 1;
    else if (map->map.data[it] == +100)
        return 0;

    it = x+1 + (height - y+1 - 1) * width;
    if(map->map.data[it] == 0)
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
            if(map->map.data[i] == -1){
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

    int dist;
    int mindist;
    for(int k=0;k<frontier_vector.size();k++){
        mindist=-1;

        for(int i = 0; i<frontier_vector[k].x.size(); i++){
            dist = 0;

            for(int e = 1; e<frontier_vector[k].x.size(); e++){
                dist += sqrt((frontier_vector[k].x[i] - frontier_vector[k].x[e])*(frontier_vector[k].x[i] - frontier_vector[k].x[e]) + (frontier_vector[k].y[i] - frontier_vector[k].y[e])*(frontier_vector[k].y[i] - frontier_vector[k].y[e]));
            }

            if(mindist==-1 || dist<mindist){
                mindist = dist;
                frontier_vector[k].min_dist = i;
                frontier_vector[k].x_mean = frontier_vector[k].x[i];
                frontier_vector[k].y_mean = frontier_vector[k].y[i];
                frontier_vector[k].center = frontier_vector[k].x_mean + (height - frontier_vector[k].y_mean - 1) * width;

            }
        }
    }

    return frontier_vector;

}



