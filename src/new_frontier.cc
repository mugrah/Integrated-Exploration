#include <nav_msgs/GetMap.h>
#include <vector>
#include <fstream>
#include <cstdlib>
#include <iostream>

#include "gmapping/occMap.h"
#include "frontiers.h"

void save_old_map(int **mapa, int width, int height, std::string robot_id, std::vector<frontier_position> &frontier_vector, double **occ_real_map){
    char sysCall2[512];

    std::string mapdatafile2 = "/home/rcolares/catkin_ws/src/ros-pioneer3at/maps/old_frontier_"+robot_id;
    sprintf(sysCall2, "%s.ppm", mapdatafile2.c_str());
    FILE* printFile2 = fopen(sysCall2, "w");
    fprintf(printFile2, "P6\n # particles.ppm \n %d %d\n", width, height);
    fprintf(printFile2, "255\n");

    for(int i = 0;i<frontier_vector.size();i++){
        for(int e=0; e<frontier_vector[i].x.size();e++){
            int x = frontier_vector[i].x[e];
            int y = frontier_vector[i].y[e];
            mapa[y][x] = 2;

        }
    }

    for(unsigned int y = 0; y <height; y++) {
        for(unsigned int x = 0; x < width; x++) {
            if(mapa[y][x]==2){
                fprintf(printFile2, "%c%c%c", 255,0,0);
            }else{
                if(mapa[y][x]==0)
                    fprintf(printFile2, "%c%c%c", 255,255,255);
                if(mapa[y][x]==+100)
                    fprintf(printFile2, "%c%c%c", 0,0,0);
                if(mapa[y][x]==-1)
                    fprintf(printFile2, "%c%c%c", 125,125,125);
            }
        }
    }
    fclose(printFile2);
    sprintf(sysCall2, "convert %s.ppm %s.png", mapdatafile2.c_str(), mapdatafile2.c_str());
    system(sysCall2);
    sprintf(sysCall2, "chmod 666 %s.ppm", mapdatafile2.c_str());
    system(sysCall2);
    sprintf(sysCall2, "chmod 666 %s.png", mapdatafile2.c_str());
    system(sysCall2);
    sprintf(sysCall2, "rm %s.ppm", mapdatafile2.c_str());
    system(sysCall2);
}



void save_map(int **mapa, int width, int height, std::string robot_id, std::vector<frontier_position> &frontier_vector, double **occ_real_map){

    char sysCall[512];
//    sprintf ( number, "%d", contador++);

    std::string mapdatafile = "/home/rcolares/catkin_ws/src/ros-pioneer3at/maps/new_frontier_"+robot_id;
    sprintf(sysCall, "%s.ppm", mapdatafile.c_str());
    FILE* printFile = fopen(sysCall, "w");
    fprintf(printFile, "P6\n # particles.ppm \n %d %d\n", width, height);
    fprintf(printFile, "255\n");

    for(int i = 0;i<frontier_vector.size();i++){
        for(int e=0; e<frontier_vector[i].x.size();e++){
            int x = frontier_vector[i].x[e];
            int y = frontier_vector[i].y[e];
            mapa[y][x] = i+2;

        }
    }


    for(unsigned int y = 0; y <height; y++) {
        for(unsigned int x = 0; x < width; x++) {
            if(mapa[y][x]==2)
                fprintf(printFile, "%c%c%c", 255,0,0);
            else if(mapa[y][x]==3)
                fprintf(printFile, "%c%c%c", 0,255,0);
            else if(mapa[y][x]==4)
                fprintf(printFile, "%c%c%c", 0,0,255);
            else if(mapa[y][x]==5)
                fprintf(printFile, "%c%c%c", 255,255,0);
            else if(mapa[y][x]==6)
                fprintf(printFile, "%c%c%c", 255,0,255);
            else
                fprintf(printFile, "%c%c%c", 255-(int)(255*occ_real_map[y][x]), 255-(int)(255*occ_real_map[y][x]), 255-(int)(255*occ_real_map[y][x]));


//            if(mapa[y][x]==0)
//                fprintf(printFile, "%c%c%c", 255,255,255);
//            if(mapa[y][x]==+100)
//                fprintf(printFile, "%c%c%c", 0,0,0);
//            if(mapa[y][x]==-1)
//                fprintf(printFile, "%c%c%c", 125,125,125);

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



void mapTransform(gmapping::occMap map,  int width, int height, int **real_map, double **mapa, double **occ_real_map, double **windows, double **neg_wave, double **wave_map){

  for(int l=0;l<height;l++){
    for(int k=0;k<width;k++){
      int t = (height - l -1)*width +k;
      real_map[l][k] = map.map.data[t];
      windows[l][k]=0.0;

      if(map.data[t] != -1)
          occ_real_map[l][k] = map.data[t];
      else
          occ_real_map[l][k] = 0.5;

      if(real_map[l][k] == +100)
          mapa[l][k] = 1;
      else
          mapa[l][k]=-1;

      if(real_map[l][k] == 0){
          wave_map[l][k] = 999999;
          neg_wave[l][k] = 999999;
      }else{
          wave_map[l][k] = -1;
          neg_wave[l][k] = -1;
      }

    }
  }
}




int check_if_frontier(int **real_map, double **occ_real_map, int j, int k){
    int flag = 0;
    if(occ_real_map[j-1][k-1] > 0 && occ_real_map[j-1][k-1] < 1)
        flag++;
    if(occ_real_map[j-1][k] > 0 && occ_real_map[j-1][k] < 1)
        flag++;
    if(occ_real_map[j-1][k+1] > 0 && occ_real_map[j-1][k+1] < 1)
        flag++;
    if(occ_real_map[j][k-1] > 0 && occ_real_map[j][k-1] < 1)
        flag++;
    if(occ_real_map[j][k+1] > 0 && occ_real_map[j][k+1] < 1)
        flag++;
    if(occ_real_map[j+1][k-1] > 0 && occ_real_map[j+1][k-1] < 1)
        flag++;
    if(occ_real_map[j+1][k] > 0 && occ_real_map[j+1][k] < 1)
        flag++;
    if(occ_real_map[j+1][k+1] > 0 && occ_real_map[j+1][k+1] < 1)
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

int check_if_old_frontier(int **real_map, int j, int k){
    int flag = 0;
    if(real_map[j-1][k-1] == -1)
        flag++;
    if(real_map[j-1][k]== -1)
        flag++;
    if(real_map[j-1][k+1] == -1)
        flag++;
    if(real_map[j][k-1] == -1)
        flag++;
    if(real_map[j][k+1] == -1)
        flag++;
    if(real_map[j+1][k-1] == -1)
        flag++;
    if(real_map[j+1][k] == -1)
        flag++;
    if(real_map[j+1][k+1] == -1)
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

void createOldFrontiers(int width, int height, double **occ_real_map, double **mapa, std::vector<frontier_position> &frontier_vector, std::string robot_topic, int **real_map){

    int flag;

    frontier_vector.clear();

    for(int i=1;i<height-1;i++){
        for(int e=1;e<width-1;e++){
            if(real_map[i][e]==0){
                if(check_if_old_frontier(real_map, i, e)){
                    mapa[i][e]=0;

                    if(frontier_vector.size()==0){
                        frontier_position aux;
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
            }
        }
    }

    //save_old_map(real_map, width, height, robot_topic, frontier_vector, occ_real_map);
}


void createFrontiers(int width, int height, double **occ_real_map, double **mapa, std::vector<frontier_position> &frontier_vector, std::string robot_topic, int **real_map){

    createOldFrontiers(width, height, occ_real_map, mapa, frontier_vector, robot_topic, real_map);

    if(frontier_vector.size()!=0){
        int flag;


        frontier_vector.clear();

        for(int i=1;i<height-1;i++){
            for(int e=1;e<width-1;e++){
                if(occ_real_map[i][e] ==0){
                    if(check_if_frontier(real_map, occ_real_map, i, e)){
                        mapa[i][e]=0;

                        if(frontier_vector.size()==0){
                            frontier_position aux;
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
                }
            }
        }
//    }
//    save_map(real_map, width, height, robot_topic, frontier_vector, occ_real_map);
    }
}
