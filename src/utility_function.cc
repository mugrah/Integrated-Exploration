#define ALPHAS 9
#define BETAS 4
#define C 1980


typedef struct goalCell{
  int x;
  int y;
  int valid;
}ff;

goalCell goal_cell;

int cont_ww=0;

void save_ww_map(double **ww_map, int width, int height, std::string robot_id){
  
  char sysCall[512];
  char number[512]; 
  sprintf ( number, "%d", cont_ww++);
  
  std::string mapdatafile = "/home/rcolares/catkin_ws/src/ros-pioneer3at/maps/ww_";
  mapdatafile+=robot_id.at(robot_id.size()-1);
  mapdatafile+="_";
  mapdatafile+=number;
  sprintf(sysCall, "%s.ppm", mapdatafile.c_str());
  FILE* printFile = fopen(sysCall, "w");
  fprintf(printFile, "P6\n # particles.ppm \n %d %d\n", width, height);
  fprintf(printFile, "255\n");

  
  for(unsigned int y = 0; y <height; y++) {
    for(unsigned int x = 0; x < width; x++) {
      if(ww_map[y][x]==0.0)
        fprintf(printFile, "%c%c%c", 0, 0, 0);
      else
        fprintf(printFile, "%c%c%c", 0 , (int)ww_map[y][x],0 );
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


void chooseGoal(double **mapa, int width, int height){
  double max_cell = 0.0;
  int cont_cell=0;
  for(int y = 0; y < height; y++){
    for(int x = 0; x < width; x++){
      if(mapa[y][x] > 0.0)
        cont_cell++;
      if(mapa[y][x] > max_cell){
        max_cell = mapa[y][x];
        goal_cell.x = x;
        goal_cell.y = y;
      }
      
    }
  }
  if(max_cell > 10.0 && cont_cell > 30)
    goal_cell.valid = 1;
  else
    goal_cell.valid = 0;
}

void chooseGoal(double **mapa, double **wave_map, double **ww_map, int width, int height, std::string robot_id){
  
  double function = -1.0;
  double f = 0.0;
  int cont_cell=0;
  double min=99999.0;
  double max = 0.0;
  
//   std::ofstream myfile;
//   std::string filename = "/home/rcolares/catkin_ws/src/ros-pioneer3at/maps/wave" + robot_id +"_"".txt";
//   myfile.open (filename.c_str(),  std::ios::out);// | std::ios::app );
  
  for(int y = 0; y < height; y++){
    for(int x = 0; x < width; x++){
      if(mapa[y][x]>0.0){
        //f = 3*mapa[y][x]/255.0 + (1 - wave_map[y][x]/510.0);
        //ww_map[y][x] = ((f - min)/(max - min))*255.0;
        
        f = 5*mapa[y][x]/255.0 + (pow((wave_map[y][x]/510),(ALPHAS-1))*pow((1-(wave_map[y][x]/510)),(BETAS-1)))*C;
        ww_map[y][x] = f;
        
        if(f>max)
          max=f;
        if(f<min)
          min=f;
        
//         myfile << x<<","<<y<<":  "<<f<<"  "<<ww_map[y][x]<<"  "<<mapa[y][x]<<"  "<<wave_map[y][x]<<"\n";
        
        if(mapa[y][x] > 0.0)
          cont_cell++;
        if(f > function){
          function = f;
          goal_cell.x = x;
          goal_cell.y = y;
        }
      }else{
        ww_map[y][x] = 0.0;
      }
    }
  }
  if(cont_cell > 30)
    goal_cell.valid = 1;
  else
    goal_cell.valid = 0;
  
  for(int y = 0; y < height; y++){
    for(int x = 0; x < width; x++){
      ww_map[y][x] = ((ww_map[y][x] - min)/(max - min))*255.0;
    }
  }
  
//   myfile.close();
  save_ww_map(ww_map, width, height, robot_id);
      
}
  
