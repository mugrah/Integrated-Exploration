#include <nav_msgs/GetMap.h>
#include <vector>
#include <fstream>
#include <cstdlib>
#include <iostream>
#include <string>

#include "pioneer3at/OccMap.h"
#include "frontier.cc"

int colors1[]={255,  0 ,  0 , 255, 255,  0 , 255, 255, 255, 255,  0 , 127, 127, 127,  0 , 127}; 
int colors2[]={ 0 , 255,  0 , 255,  0 , 255, 255, 127,  0 , 127, 255, 255, 255,  0 , 127, 127};
int colors3[]={ 0 ,  0 , 255,  0 , 255, 255, 255,  0 , 127, 127, 127,  0 , 127, 255, 255, 255};

int x_1, y_1, x_2, y_2;
int contador_uwave=0;

void save_unknown_wave(int **u_wave, int width, int height, std::string robot_id){
  
  char sysCall[512];
  char number[512];
  sprintf ( number, "%d", contador_uwave++);
  std::string mapdatafile = "/home/rafael/catkin_ws/src/ros-pioneer3at/maps/u_wave_";
//   mapdatafile+=robot_id.at(robot_id.size()-1);
  mapdatafile+=robot_id;
  mapdatafile+="_";
  mapdatafile+=number;
  sprintf(sysCall, "%s.ppm", mapdatafile.c_str());
  FILE* printFile = fopen(sysCall, "w");
  fprintf(printFile, "P6\n # particles.ppm \n %d %d\n", width, height);
  fprintf(printFile, "255\n");

  
  for(unsigned int y = 0; y <height; y++) {
    for(unsigned int x = 0; x < width; x++) {
      if(u_wave[y][x] ==-1)
        fprintf(printFile, "%c%c%c", 0, 0, 0);
      else{
        int color = u_wave[y][x];  
        fprintf(printFile, "%c%c%c", colors1[color], colors2[color], colors3[color]);  
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
  
  ROS_INFO("Done\n");
}


void mapTransform(pioneer3at::OccMap map,  int width, int height, int **real_map, int **mapa, double **occ_real_map){

  for(int l=0;l<height;l++){
    for(int k=0;k<width;k++){
      int t = (height - l -1)*width +k;
      real_map[l][k] = map.map.data[t];
      mapa[l][k]=0;
      occ_real_map[l][k] = map.data[t];
    }
  }
}

void find_limits(int **real_map, int width, int height){
    
    int x_min, y_min, x_max, y_max;
    x_min = y_min = 99999;
    x_max = y_max = 0;
    
    for(int i = 0; i < height; i++){
        for(int e = 0; e < width; e++){
            if(real_map[i][e] != -1){
                if(i<y_min)
                    y_min = i;
                if(i>y_max)
                    y_max = i;
                if(e<x_min)
                    x_min = e;
                if(e>x_max)
                    x_max = e;
            }
        }
    }
    x_1 = x_min;
    y_1 = y_min;
    x_2 = x_max;
    y_2 = y_max;
}

void up_down(int **u_wave, int **real_map, int *wave_size){

    for(int i = y_1-30; i < y_2+30; i++){
        for(int e = x_1-30; e < x_2+30; e++){
            if(u_wave[i][e] != -1){
                if(u_wave[i-1][e-1]==-1 && real_map[i-1][e-1]==-1){
                    u_wave[i-1][e-1] == u_wave[i][e];
                    wave_size[u_wave[i][e]]++;
                }
                if(u_wave[i-1][e]==-1 && real_map[i-1][e]==-1){
                    u_wave[i-1][e] == u_wave[i][e];
                    wave_size[u_wave[i][e]]++;
                }
                if(u_wave[i-1][e+1]==-1 && real_map[i-1][e+1]==-1){
                    u_wave[i-1][e+1] == u_wave[i][e];
                    wave_size[u_wave[i][e]]++;
                }
                if(u_wave[i+1][e-1]==-1 && real_map[i+1][e-1]==-1){
                    u_wave[i+1][e-1] == u_wave[i][e];
                    wave_size[u_wave[i][e]]++;
                }
                if(u_wave[i+1][e]==-1 && real_map[i+1][e]==-1){
                    u_wave[i+1][e] == u_wave[i][e];
                    wave_size[u_wave[i][e]]++;
                }
                if(u_wave[i+1][e+1]==-1 && real_map[i+1][e+1]==-1){
                    u_wave[i+1][e+1] == u_wave[i][e];
                    wave_size[u_wave[i][e]]++;
                }
                if(u_wave[i][e-1]==-1 && real_map[i][e-1]==-1){
                    u_wave[i][e-1] == u_wave[i][e];
                    wave_size[u_wave[i][e]]++;
                }
                if(u_wave[i][e+1]==-1 && real_map[i][e+1]==-1){
                    u_wave[i][e+1] == u_wave[i][e];
                    wave_size[u_wave[i][e]]++;
                }
            }
        }
    }
}

void down_up(int **u_wave, int **real_map, int *wave_size){

    for(int i = y_2+30; i > y_1-30; i--){
        for(int e = x_2+30; e > x_1-30; e--){
            if(u_wave[i][e] != -1){
                if(u_wave[i-1][e-1]==-1 && real_map[i-1][e-1]==-1){
                    u_wave[i-1][e-1] == u_wave[i][e];
                    wave_size[u_wave[i][e]]++;
                }
                if(u_wave[i-1][e]==-1 && real_map[i-1][e]==-1){
                    u_wave[i-1][e] == u_wave[i][e];
                    wave_size[u_wave[i][e]]++;
                }
                if(u_wave[i-1][e+1]==-1 && real_map[i-1][e+1]==-1){
                    u_wave[i-1][e+1] == u_wave[i][e];
                    wave_size[u_wave[i][e]]++;
                }
                if(u_wave[i+1][e-1]==-1 && real_map[i+1][e-1]==-1){
                    u_wave[i+1][e-1] == u_wave[i][e];
                    wave_size[u_wave[i][e]]++;
                }
                if(u_wave[i+1][e]==-1 && real_map[i+1][e]==-1){
                    u_wave[i+1][e] == u_wave[i][e];
                    wave_size[u_wave[i][e]]++;
                }
                if(u_wave[i+1][e+1]==-1 && real_map[i+1][e+1]==-1){
                    u_wave[i+1][e+1] == u_wave[i][e];
                    wave_size[u_wave[i][e]]++;
                }
                if(u_wave[i][e-1]==-1 && real_map[i][e-1]==-1){
                    u_wave[i][e-1] == u_wave[i][e];
                    wave_size[u_wave[i][e]]++;
                }
                if(u_wave[i][e+1]==-1 && real_map[i][e+1]==-1){
                    u_wave[i][e+1] == u_wave[i][e];
                    wave_size[u_wave[i][e]]++;
                }
            }
        }
    }
}


int unknown_information(int **real_map, int width, int height){
    
    int **u_wave;
    
    int num =  frontier_vector.size();
    int wave_size[num];
    int max = 0;
    int chosen;
    
    std::ofstream myfile;
    std::string filename = "/home/rafael/catkin_ws/src/ros-pioneer3at/maps/abort.txt";
    
    myfile.open (filename.c_str(),  std::ios::out | std::ios::app );
    myfile << num <<" find_limits\n";
    myfile.close();
    
    find_limits(real_map, width, height);
    
    u_wave = (int**) malloc(height*sizeof(int*));
    for(int i=0; i<height; i++)
      u_wave[i] = (int*) malloc(width*sizeof(int*));
    
    myfile.open (filename.c_str(),  std::ios::out | std::ios::app );
    myfile << "memset\n";
    myfile.close();   
    
    memset(u_wave, -1, height*width*sizeof(int*));
    
    myfile.open (filename.c_str(),  std::ios::out | std::ios::app );
    myfile << "for\n";
    myfile.close(); 
    
    
    for(int k = 0; k<num;k++){
        myfile.open (filename.c_str(),  std::ios::out | std::ios::app );
        myfile << frontier_vector.at(k).y_mean<<"  "<<frontier_vector.at(k).x_mean<<"\n";
        myfile.close(); 
        u_wave[frontier_vector.at(k).y_mean][frontier_vector.at(k).x_mean] = k;
        myfile.open (filename.c_str(),  std::ios::out | std::ios::app );
        myfile <<u_wave[frontier_vector.at(k).y_mean][frontier_vector.at(k).x_mean]<<"\n";
        myfile.close();        
        wave_size[k]=1;
    }
    
    myfile.open (filename.c_str(),  std::ios::out | std::ios::app );
    myfile << "antes\n";
    myfile.close();
    
    up_down(u_wave, real_map, wave_size);
    down_up(u_wave, real_map, wave_size);
    
    myfile.open (filename.c_str(),  std::ios::out | std::ios::app );
    myfile << "depois\n";
    myfile.close();
    
    for(int i=0;i<num;i++){
        if(wave_size[i] > max){
            max = wave_size[i];
            chosen = i;
        }
    } 
    
    save_unknown_wave(u_wave, width, height, "robot");
    
    return chosen;
}