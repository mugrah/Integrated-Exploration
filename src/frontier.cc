#include <nav_msgs/GetMap.h>
#include <vector>
#include <fstream>
#include <cstdlib>
#include <iostream>

typedef struct frontier_position {
  int x_mean, y_mean, num, min_dist;
  std::vector<int> x;
  std::vector<int> y;
}iposs;

std::vector<frontier_position> frontier_vector;


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


int check_if_frontier(int **real_map, int j, int k){
  int flag = 0;
  if(real_map[j-1][k-1] == 0)
    flag++;
  if(real_map[j-1][k]== 0)
    flag++;
  if(real_map[j-1][k+1] == 0)
    flag++;
  if(real_map[j][k-1] == 0)
    flag++;
  if(real_map[j][k+1] == 0)
    flag++;
  if(real_map[j+1][k-1] == 0)
    flag++;
  if(real_map[j+1][k] == 0)
    flag++;
  if(real_map[j+1][k+1] == 0)
    flag++;
  if(real_map[j-1][k-1] == +100)
    return 0;
  if(real_map[j-1][k]== +100)
    return 0;
  if(real_map[j-1][k+1] == +100)
    return 0;
  if(real_map[j][k-1] == +100)
    return 0;
  if(real_map[j][k+1] == +100)
    return 0;
  if(real_map[j+1][k-1] == +100)
    return 0;
  if(real_map[j+1][k] == +100)
    return 0;
  if(real_map[j+1][k+1] == +100)
    return 0;
  return flag;
}

void createFrontiers(int width, int height, int **real_map, int **mapa){
  
  int flag;
  
  frontier_vector.clear();
  
  for(int i=1;i<height-1;i++){
    for(int e=1;e<width-1;e++){
      if(real_map[i][e] ==-1){
        if(check_if_frontier(real_map, i, e)){
          mapa[i][e]=0;
          
          if(frontier_vector.size()==0){
            frontier_position aux;
//             aux.x_mean = e;
//             aux.y_mean = i;
            aux.num = 1;
            aux.x.push_back(e);
            aux.y.push_back(i);
            frontier_vector.push_back(aux);
          }else{
            flag=0;
            for(int k = 0; k < frontier_vector.size(); k++){
              for(int j = 0; j < frontier_vector[k].x.size(); j++){
                float dist = sqrt((frontier_vector[k].y[j]-i)*(frontier_vector[k].y[j]-i) + (frontier_vector[k].x[j]-e)*(frontier_vector[k].x[j]-e));
                if(dist<=7){
//                   frontier_vector[k].x_mean = (int)(frontier_vector[k].x_mean + e)/2;
//                   frontier_vector[k].y_mean = (int)(frontier_vector[k].y_mean + i)/2;
                  frontier_vector[k].num+=1;
                  frontier_vector[k].x.push_back(e);
                  frontier_vector[k].y.push_back(i);
                  flag=1;
                  break;
                }
              }
              if(flag==1)
                break;
            }
            if(flag==0){
              frontier_position aux;
//               aux.x_mean = e;
//               aux.y_mean = i;
              aux.num = 1;
              aux.x.push_back(e);
              aux.y.push_back(i);
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
//     int xsoma=0;
//     int ysoma=0;
    
    mindist=-1;
    
    for(int i = 0; i<frontier_vector[k].x.size(); i++){
      dist = 0;
      
      for(int e = 1; e<frontier_vector[k].x.size(); e++){
        dist += sqrt((frontier_vector[k].x[i] - frontier_vector[k].x[e])*(frontier_vector[k].x[i] - frontier_vector[k].x[e]) + (frontier_vector[k].y[i] - frontier_vector[k].y[e])*(frontier_vector[k].y[i] - frontier_vector[k].y[e]));
//         xsoma += frontier_vector[i].x[e];
//         ysoma += frontier_vector[i].y[e];
      }

      if(mindist==-1 || dist<mindist){
        mindist = dist;
        frontier_vector[k].min_dist = i;
        frontier_vector[k].x_mean = frontier_vector[k].x[i];
        frontier_vector[k].y_mean = frontier_vector[k].y[i];
      }
    }
  }
  

  
  
//   myfile.open ("/home/rafael/catkin_ws/src/ros-pioneer3at/maps/teste.txt",  std::ios::out);// | std::ios::app );
//   myfile<<frontier_vector.size()<<"\n";
//   for(int i=0;i<frontier_vector.size();i++){
//     myfile<<"---> "<<frontier_vector[i].x_mean<<"  "<<frontier_vector[i].y_mean <<"  "<<frontier_vector[i].num<<"\n";
//     for(int e =0; e<frontier_vector[i].x.size(); e++){
//       myfile<<"    "<<e+1<<"  "<<frontier_vector[i].x[e]<<"  "<<frontier_vector[i].y[e]<<"\n";
//     }
//   }
//   myfile<<"----------------------------------\n";
//   myfile.close();
}



