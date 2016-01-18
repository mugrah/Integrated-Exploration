#include <fstream>
#include <cstdlib>
#include <iostream>
#include <string>

int contador_wave = 0;

void save_wave_map(double **wave_map, int width, int height, std::string robot_id){
  
  char sysCall[512];
  char number[512];
  sprintf ( number, "%d", contador_wave++);
  std::string mapdatafile = "/home/rcolares/catkin_ws/src/ros-pioneer3at/maps/wavefront_";
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
      if(wave_map[y][x] ==-1)
        fprintf(printFile, "%c%c%c", 0, 0, 0);
      else
        fprintf(printFile, "%c%c%c", (int)wave_map[y][x]/2, 255-((int)wave_map[y][x]/2), 0);
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

void save_negwave_map(double **wave_map, int width, int height, std::string robot_id){
  
  char sysCall[512];
  char number[512];
  sprintf ( number, "%d", contador_wave++);
  std::string mapdatafile = "/home/rcolares/catkin_ws/src/ros-pioneer3at/maps/wavefront_";
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
      if(wave_map[y][x] ==-1)
        fprintf(printFile, "%c%c%c", 0, 0, 0);
      else
        fprintf(printFile, "%c%c%c", (int)wave_map[y][x], 0, 0);
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

void multi_wavefront(int width, int height, int **real_map, double **wave_map, double **mapa, int x, int y, int x_other, int y_other, std::string robot_id){
  double max, min, neg_max, neg_min;
  double **neg_wave;
  

  if(x_other != -1){
    neg_wave = (double**) malloc(height*sizeof(double*));
    for(int i = 0; i < height; i++)
      neg_wave[i] = (double*) malloc(width*sizeof(double*));
  }
  
  max=0.0;
  neg_max = 0.0;
  min=99999.0;
  neg_min = 99999.0;
  

  for(int i = 0; i < height; i++){
    for(int e = 0; e < width; e++){
      if((real_map[i][e] != -1 && real_map[i][e] != 100) || (mapa[i][e]!=0.0)){
        wave_map[i][e] = sqrt(pow((x-e),2)+pow((y-i),2));
        if(x_other != -1){
          neg_wave[i][e] = sqrt(pow((x_other - e), 2)+pow((y_other-i),2));
          if(neg_wave[i][e]>neg_max)
            neg_max=neg_wave[i][e];
          if(neg_wave[i][e]<neg_min)
            neg_min=neg_wave[i][e];
        }
//         if(wave_map[i][e] <60.0){
//          wave_map[i][e]=-1;
//         }else{
          if(wave_map[i][e]>max)
            max=wave_map[i][e];
          if(wave_map[i][e]<min)
            min=wave_map[i][e];
//         }
      }else{
        wave_map[i][e]=-1.0;
        if(x_other != -1)
          neg_wave[i][e]=-1.0;
      } 
    }
  }

  if(x_other != -1){
    for(int i = 0; i < height; i++){
      for(int e = 0; e < width; e++){
        if(neg_wave[i][e]!=-1.0){
          neg_wave[i][e] = ((neg_wave[i][e] - neg_max)/(neg_min - neg_max))*255.0;
        }
      }
    }
    std::string name = "neg_";
    name+=robot_id;
    save_negwave_map(neg_wave, width, height, name);
  }

  for(int i = 0; i < height; i++){
    for(int e = 0; e < width; e++){
      if(x_other != -1){
        if(wave_map[i][e]!=-1.0){
          wave_map[i][e] = ((wave_map[i][e] - min)/(max - min))*510.0 + neg_wave[i][e];
          if(wave_map[i][e] > 510.0)
            wave_map[i][e] = 510;
        }
      }else{
        if(wave_map[i][e]!=-1.0)
          wave_map[i][e] = ((wave_map[i][e] - min)/(max - min))*510.0;
      }
    }
  }


 save_wave_map(wave_map, width,height, robot_id);


 if(x_other!=-1){
    for(int i =0;i<height; i++)
      free(neg_wave[i]);
    free(neg_wave);
  }

}

void wavefront(int width, int height, int **real_map, double **wave_map, double **mapa, int x, int y, std::string robot_id){
  double max, min;
  
  max=0.0;
  min=99999.0;
  
  for(int i = 0; i < height; i++){
    for(int e = 0; e < width; e++){
      if((real_map[i][e] != -1 && real_map[i][e] != 100) || (mapa[i][e]!=0.0)){
        wave_map[i][e] = sqrt(pow((x-e),2)+pow((y-i),2));
//         if(wave_map[i][e] <60.0){
//          wave_map[i][e]=-1;
//         }else{
          if(wave_map[i][e]>max)
            max=wave_map[i][e];
          if(wave_map[i][e]<min)
            min=wave_map[i][e];
//         }
      }else{
        wave_map[i][e]=-1.0;
      } 
    }
  }
  
  for(int i = 0; i < height; i++){
    for(int e = 0; e < width; e++){
      if(wave_map[i][e]!=-1.0){
        wave_map[i][e] = ((wave_map[i][e] - min)/(max - min))*510.0;
      }
    }
  }
  //save_wave_map(wave_map, width,height, robot_id);
}
