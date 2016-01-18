
#include <nav_msgs/GetMap.h>
#include <vector>
#include <fstream>
#include <cstdlib>
#include <iostream>

#define MAP_FREE 999999

typedef struct robot_front{
    int id;
    int dist;

   // robot_front(int i, int d) : id(i), dist(d) {}

    //bool operator < (const frontier_position& str) const{
    //    return (dist < str.dist);
   // }

}ss;

struct less_than_key
{
    inline bool operator() (const robot_front& struct1, const robot_front& struct2)
    {
        return (struct1.dist < struct2.dist);
    }
};

typedef struct frontier_position {
    int x_mean, y_mean, num, min_dist;
    //double dist;
    std::vector<int> x;
    std::vector<int> y;
    std::vector<robot_front> wave;

//    bool operator < (const frontier_position& str) const{
//        return (dist < str.dist);
//    }


}iposs;

typedef struct robot_poses{

    int x, y, id;

}aa;

std::vector<frontier_position> frontier_vector;
std::vector<robot_poses> r;

void save_map(double **map, int width, int height, int id, std::string robot_topic ){

    char sysCall[500];
    std::string mapdatafile = "/home/rcolares/catkin_ws/src/ros-pioneer3at/maps/dist"+robot_topic;
    mapdatafile += id;
    sprintf(sysCall, "%s.ppm", mapdatafile.c_str());
    FILE* printFile = fopen(sysCall, "w");
    fprintf(printFile, "P6\n # particles.ppm \n %d %d\n", width, height);
    fprintf(printFile, "255\n");


    for(unsigned int y = 0; y <height; y++) {
        for(unsigned int x = 0; x < width; x++) {
            if(map[y][x] == -1 || map[y][x] == MAP_FREE)
                fprintf(printFile, "%c%c%c", 0, 0, 0);
            else
                fprintf(printFile, "%c%c%c", (int)(map[y][x]/2), (int)(255 - (map[y][x]/2)), 0);
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
}



void mapTransform(nav_msgs::OccupancyGrid map,  int width, int height, int **real_map, int **mapa, int **maps){
    for(int l=0;l<height;l++){
        for(int k=0;k<width;k++){
            int t = (height - l -1)*width +k;
            real_map[l][k] = map.data[t];
            if(real_map[l][k] == +100)
                mapa[l][k] = 1;
            else
                mapa[l][k]=-1;

            if(real_map[l][k] == 0)
                maps[l][k] = MAP_FREE;
            else
                maps[l][k] = -1;
        }
    }
}


int check_if_frontier(int **real_map, int j, int k){
    int flag = 0;
    if(real_map[j-1][k-1] == -1) //troquei o 0 or -1 pra fronteiras
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

void createFrontiers(int width, int height, int **real_map, int **mapa){

    int flag;

    frontier_vector.clear();

    for(int i=1;i<height-1;i++){
        for(int e=1;e<width-1;e++){
            if(real_map[i][e] ==0){ //troquei -1 por 0
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
}

void wavefront_map(int x, int y, int **map, int width, int height, int id, std::string robot_topic){
    int changes = 1;    

    int **mapa = (int**) malloc(height*sizeof(int*));
    double **print = (double**) malloc(height*sizeof(double*));
    for(int i=0; i<height; i++){
        mapa[i] = (int*) malloc(width*sizeof(int*));
        print[i] = (double*) malloc(width*sizeof(double*));
    }



    for(int i=0;i<height;i++){
        for(int e=0;e<width;e++){
            mapa[i][e] = map[i][e];
            print[i][e] = -1;
        }
    }

    mapa[y][x] = 0;

    while(changes!=0){

        changes = 0;

        for(int i = 1; i < height; i++){
            for(int e = 1; e < width; e ++){
                if(mapa[i][e] != MAP_FREE && mapa[i][e]!=-1){

                    if(mapa[i-1][e]==MAP_FREE){
                        mapa[i-1][e] = mapa[i][e]+1;
                        changes=1;
                    }
                    if(mapa[i-1][e-1]==MAP_FREE){
                        mapa[i-1][e-1] = mapa[i][e]+1;
                        changes=1;
                    }
                    if(mapa[i-1][e+1]==MAP_FREE){
                        mapa[i-1][e+1] = mapa[i][e]+1;
                        changes=1;
                    }
                    if(mapa[i+1][e]==MAP_FREE){
                        mapa[i+1][e] = mapa[i][e]+1;
                        changes=1;
                    }
                    if(mapa[i+1][e-1]==MAP_FREE){
                        mapa[i+1][e-1] = mapa[i][e]+1;
                        changes=1;
                    }
                    if(mapa[i+1][e+1]==MAP_FREE){
                        mapa[i+1][e+1] = mapa[i][e]+1;
                        changes=1;
                    }
                    if(mapa[i][e-1]==MAP_FREE){
                        mapa[i][e-1] = mapa[i][e]+1;
                        changes=1;
                    }
                    if(mapa[i][e+1]==MAP_FREE){
                        mapa[i][e+1] = mapa[i][e]+1;
                        changes=1;
                    }
                }
            }
        }

        for(int i = height-1; i >= 0; i--){
            for(int e = width-1; e >=0; e --){
                if(mapa[i][e] != MAP_FREE && mapa[i][e]!=-1){

                    if(mapa[i-1][e]==MAP_FREE){
                        mapa[i-1][e] = mapa[i][e]+1;
                        changes=1;
                    }
                    if(mapa[i-1][e-1]==MAP_FREE){
                        mapa[i-1][e-1] = mapa[i][e]+1;
                        changes=1;
                    }
                    if(mapa[i-1][e+1]==MAP_FREE){
                        mapa[i-1][e+1] = mapa[i][e]+1;
                        changes=1;
                    }
                    if(mapa[i+1][e]==MAP_FREE){
                        mapa[i+1][e] = mapa[i][e]+1;
                        changes=1;
                    }
                    if(mapa[i+1][e-1]==MAP_FREE){
                        mapa[i+1][e-1] = mapa[i][e]+1;
                        changes=1;
                    }
                    if(mapa[i+1][e+1]==MAP_FREE){
                        mapa[i+1][e+1] = mapa[i][e]+1;
                        changes=1;
                    }
                    if(mapa[i][e-1]==MAP_FREE){
                        mapa[i][e-1] = mapa[i][e]+1;
                        changes=1;
                    }
                    if(mapa[i][e+1]==MAP_FREE){
                        mapa[i][e+1] = mapa[i][e]+1;
                        changes=1;
                    }
                }
            }
        }
    }
    /*double max = 0.0;
    int min = 999999;
    for(int i = 0; i < height; i++){
        for(int e =0; e < width; e ++){
            if(mapa[i][e]!=-1 && mapa[i][e] != MAP_FREE){
                if(max < mapa[i][e]){
                    max = mapa[i][e];
                }
                if(min > mapa[i][e])
                    min = mapa[i][e];
            }
        }
    }

    for(int i = 0; i < height; i++){
        for(int e = 0; e < width; e++){
            if(mapa[i][e]!=-1 && mapa[i][e] != MAP_FREE){
                print[i][e] = ((mapa[i][e])/(max))*510;
            }else
                print[i][e] = -1;

        }
    }

    save_map(print, width, height, id, robot_topic);*/

    //std::ofstream myfile2;
    //std::string filename2 = "/home/rcolares/catkin_ws/src/ros-pioneer3at/maps/cons" + robot_topic +".txt";
    //myfile2.open (filename2.c_str(),  std::ios::out | std::ios::app );
    for(int i=0; i<r.size(); i++){
        robot_front aux;
        aux.id = i;
        aux.dist = mapa[r[i].y][r[i].x];

      //  myfile2<<"| "<< x <<" | "<< y <<" | " << id << " | "<< i <<" | " << r[i].x << " | "<< r[i].y <<" | "<< mapa[r[i].y][r[i].x] <<" |\n";

        frontier_vector[id].wave.push_back(aux);

    }
    //myfile2.close();



    for(int i=0;i<height;i++){
        free(mapa[i]);
        free(print[i]);
    }
    free(mapa);
    free(print);


}


void sort_frontiers(std::string robot_topic){

    for(int i=0; i<frontier_vector.size();i++){
        std::sort(frontier_vector[i].wave.begin(), frontier_vector[i].wave.end(), less_than_key());
    }

}





int check_closest(int id, int **map, int width, int height, std::string robot_topic){

    int choosen = -1;
    int dist = 999999;



    for(int i=0; i<frontier_vector.size();i++)
        wavefront_map(frontier_vector[i].x_mean, frontier_vector[i].y_mean, map, width, height, i, robot_topic);

    /*std::ofstream myfile2;
    std::string filename2 = "/home/rcolares/catkin_ws/src/ros-pioneer3at/maps/frontiers" + robot_topic +".txt";
    myfile2.open (filename2.c_str(),  std::ios::out);// | std::ios::app );
    myfile2<<"\n--------------------------------\n";
    for(int i=0; i<frontier_vector.size();i++){
        for(int e=0; e<frontier_vector[i].wave.size(); e++){
            myfile2<<"| "<< i << " | "<< e <<" | "<< frontier_vector[i].wave[e].id <<" | "<< frontier_vector[i].wave[e].dist<<" |\n";

        }
    }
    myfile2.close();*/

    for(int i=0; i<frontier_vector.size();i++){
        if(frontier_vector[i].wave[0].id == id){
            if(frontier_vector[i].wave[0].dist < dist){
                choosen = i;
                dist = frontier_vector[i].wave[0].dist;
            }
        }

    }

    int best_dist = 999999;

    if(choosen == -1){
        for(int i=0; i<frontier_vector.size();i++){

            for(int e=1; e<frontier_vector[i].wave.size(); e++){

                if(frontier_vector[i].wave[e].id == id){
                    if(frontier_vector[i].wave[e].dist < best_dist){
                        choosen = i;
                        best_dist = frontier_vector[i].wave[e].dist;
                    }
                }
            }
        }
    }

    /*std::ofstream myfile;
    std::string filename = "/home/rcolares/catkin_ws/src/ros-pioneer3at/maps/check" + robot_topic +".txt";
    myfile.open (filename.c_str(),  std::ios::out | std::ios::app );
    myfile << "ID -> " << id<<std::endl;
    myfile <<"Frontier -> "<<choosen <<std::endl;
    myfile <<"("<<frontier_vector[choosen].x_mean<<", "<<frontier_vector[choosen].y_mean<<")" <<std::endl;
    myfile << "------------------------------------" <<std::endl;

    myfile.close();*/
    return choosen;

}



