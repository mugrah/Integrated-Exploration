#include <nav_msgs/GetMap.h>
#include <vector>
#include <fstream>
#include <cstdlib>
#include <iostream>
#include <string>

#include "pioneer3at/OccMap.h"

#define WINDOW_SIZE 9


int contador = 0;

void save_map(double **mapa, int width, int height, std::string robot_id){
  
  char sysCall[512];
  char number[512]; 
  sprintf ( number, "%d", contador++);
  
  std::string mapdatafile = "/home/rafael/catkin_ws/src/ros-pioneer3at/maps/window_";
  mapdatafile+=robot_id.at(robot_id.size()-1);
  mapdatafile+="_";
  mapdatafile+=number;
  sprintf(sysCall, "%s.ppm", mapdatafile.c_str());
  FILE* printFile = fopen(sysCall, "w");
  fprintf(printFile, "P6\n # particles.ppm \n %d %d\n", width, height);
  fprintf(printFile, "255\n");

  
  for(unsigned int y = 0; y <height; y++) {
    for(unsigned int x = 0; x < width; x++) {
      fprintf(printFile, "%c%c%c", (int)mapa[y][x], (int)mapa[y][x], (int)mapa[y][x]);
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

void mapTransform(pioneer3at::OccMap map,  int width, int height, int **real_map, double **mapa, double **occ_real_map){

  for(int l=0;l<height;l++){
    for(int k=0;k<width;k++){
      int t = (height - l -1)*width +k;
      real_map[l][k] = map.map.data[t];
      mapa[l][k]=0.0;
      occ_real_map[l][k] = map.data[t];
    }
  }
}

double functionCell(double **occ_real_map, int k, int j){
  double sigma = 0.1;
  double f=0.0;
  
  
  for(int i = k-1; i <= k+1; i++){
    for(int e = j-1; e<= j+1; e++){
      if(occ_real_map[k][j] != occ_real_map[i][e]){
        if(occ_real_map[k][j] < 0.95 && occ_real_map[i][e] < 0.95){ 
          if(occ_real_map[k][j] == -1 ){
            f += -occ_real_map[k][j] + exp(-((pow((occ_real_map[i][e]-0.5),2))/(2*pow(sigma,2))));
          }else if(occ_real_map[i][e] == -1){
            f += -occ_real_map[i][e] + exp(-((pow((occ_real_map[k][j]-0.5),2))/(2*pow(sigma,2))));
          }else{
            f += exp(-((pow((occ_real_map[i][e]-0.5),2))/(2*pow(sigma,2)))) + exp(-((pow((occ_real_map[k][j]-0.5),2))/(2*pow(sigma,2))));
          }
        }else{
          f+=0.0;
        }
      }else{
        f+=0.0;
      }
    }
  }
  return f;
  
}
  

double functionWindow(int **real_map, double **occ_real_map, int i, int e){
  int cont_obs;
  cont_obs = 0;
  
  double f = 0.0;
  for(int k= i-WINDOW_SIZE/2; k <= i + WINDOW_SIZE/2;k++){
    for(int j=e-WINDOW_SIZE/2; j<= e + WINDOW_SIZE/2;j++){
      if(real_map[k][j] == 100)
        return 0.0;
    }
  }
  for(int k= i-WINDOW_SIZE/2+1; k < i + WINDOW_SIZE/2;k++){
    for(int j=e-WINDOW_SIZE/2+1; j< e + WINDOW_SIZE/2;j++){
      f += functionCell(occ_real_map, k, j);
    }
  }
  return f;
}

void createFrontiers(int width, int height, int **real_map, double **mapa, double **occ_real_map, std::string robot_id){
  double max, min, max_cell;
  
  min = 0.0;
  max = pow((WINDOW_SIZE-2), 2)*12;
  for(int i = WINDOW_SIZE/2; i < height - WINDOW_SIZE/2; i++){
    for(int e = WINDOW_SIZE/2; e < width - WINDOW_SIZE/2; e++){
      mapa[i][e] = functionWindow(real_map, occ_real_map, i, e);
      mapa[i][e] = ((mapa[i][e] - min)/(max - min))*255.0;
      
    }
  }
    
  //save_map(mapa, width, height, robot_id);
}


// std::ofstream myfile;
// myfile.open ("/home/rafael/catkin_ws/src/ros-pioneer3at/maps/window.txt",  std::ios::out);// | std::ios::app );
// myfile << "--------> ("<< i << ","<< e<<")\n";
// myfile <<"("<<occ_real_map[k][j]<<") ";
// myfile.close();