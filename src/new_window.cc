#include <nav_msgs/GetMap.h>
#include <vector>
#include <fstream>
#include <cstdlib>
#include <iostream>
#include <string>
#include <ostream>

#include "frontiers.h"

#include "gmapping/occMap.h"

#define WINDOW_SIZE 3


int contador = 0;

void save_map(double **mapa, int width, int height, std::string robot_id, std::vector<frontier_position> &frontier_vector){

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
            fprintf(printFile, "%c%c%c", 0,0,0);
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


double fInteracao(double **occ_real_map, int x, int y, int xj, int yj, std::string robot_topic){

    int lim = WINDOW_SIZE/2;
    double sigma = 0.01;
    double f =0.0;



    if((yj-1 >= y-lim) && (yj-1 <= y+lim) && (xj-1 >= x-lim) && (xj-1 <= x+lim))
        f += exp(-((pow((occ_real_map[yj][xj]-0.5),2))/(2*pow(sigma,2)))) + exp(-((pow((occ_real_map[yj-1][xj-1]-0.5),2))/(2*pow(sigma,2))));

//     myfile << yj-1<<" "<< y-lim<<" "<< xj<<" "<< x-lim<<"\n";

    if((yj-1 >= y-lim) && (yj-1 <= y+lim) && (xj >= x-lim) && (xj <= x+lim))
        f += exp(-((pow((occ_real_map[yj][xj]-0.5),2))/(2*pow(sigma,2)))) + exp(-((pow((occ_real_map[yj-1][xj]-0.5),2))/(2*pow(sigma,2))));

//     myfile << yj-1<<" "<< y-lim<<" "<< xj+1<<" "<< x-lim<<"\n";

    if((yj-1 >= y-lim) && (yj-1 <= y+lim) && (xj+1 >= x-lim) && (xj+1 <= x+lim))
        f += exp(-((pow((occ_real_map[yj][xj]-0.5),2))/(2*pow(sigma,2)))) + exp(-((pow((occ_real_map[yj-1][xj+1]-0.5),2))/(2*pow(sigma,2))));

//     myfile << yj<<" "<< y-lim<<" "<< xj-1<<" "<< x-lim<<"\n";

    if((yj >= y-lim) && (yj <= y+lim) && (xj-1 >= x-lim) && (xj-1 <= x+lim))
        f += exp(-((pow((occ_real_map[yj][xj]-0.5),2))/(2*pow(sigma,2)))) + exp(-((pow((occ_real_map[yj][xj-1]-0.5),2))/(2*pow(sigma,2))));

//     myfile << yj<<" "<< y-lim<<" "<< xj+1<<" "<< x-lim<<"\n";

    if((yj >= y-lim) && (yj <= y+lim) && (xj+1 >= x-lim) && (xj+1 <= x+lim))
        f += exp(-((pow((occ_real_map[yj][xj]-0.5),2))/(2*pow(sigma,2)))) + exp(-((pow((occ_real_map[yj][xj+1]-0.5),2))/(2*pow(sigma,2))));

//     myfile << yj+1<<" "<< y-lim<<" "<< xj-1<<" "<< x-lim<<"\n";

    if((yj+1 >= y-lim) && (yj+1 <= y+lim) && (xj >= x-lim) && (xj <= x+lim))
        f += exp(-((pow((occ_real_map[yj][xj]-0.5),2))/(2*pow(sigma,2)))) + exp(-((pow((occ_real_map[yj+1][xj-1]-0.5),2))/(2*pow(sigma,2))));

//     myfile << yj+1<<" "<< y-lim<<" "<< xj<<" "<< x-lim<<"\n";

    if((yj+1 >= y-lim) && (yj+1 <= y+lim) && (xj >= x-lim) && (xj <= x+lim))
        f += exp(-((pow((occ_real_map[yj][xj]-0.5),2))/(2*pow(sigma,2)))) + exp(-((pow((occ_real_map[yj+1][xj]-0.5),2))/(2*pow(sigma,2))));

//     myfile << yj+1<<" "<< y-lim<<" "<< xj+1<<" "<< x-lim<<"\n";

    if((yj+1 >= y-lim) && (yj+1 <= y+lim) && (xj+1 >= x-lim) && (xj+1 <= x+lim))
        f += exp(-((pow((occ_real_map[yj][xj]-0.5),2))/(2*pow(sigma,2)))) + exp(-((pow((occ_real_map[yj+1][xj+1]-0.5),2))/(2*pow(sigma,2))));


//     myfile.close();
    return f;

}


double fWindow(double **occ_real_map, int x, int y, std::string robot_topic){

    double fjanela=0.0;

//      std::ofstream myfile;
//      std::string filename = "/home/rcolares/catkin_ws/src/ros-pioneer3at/maps/teste"+robot_topic+".txt";
//      myfile.open (filename.c_str(),  std::ios::out | std::ios::app );

    for(int i = y - WINDOW_SIZE/2; i <= y + WINDOW_SIZE/2; i++){
        for(int e = x - WINDOW_SIZE/2; e <= x + WINDOW_SIZE/2 ;e++){
// 	    myfile<<occ_real_map[i][x]<<"\n";
        fjanela += fInteracao(occ_real_map, x, y, e, i, robot_topic);
        }
    }
//     myfile.close();
    return fjanela;



}



void window(int width, int height, int **real_map, double **mapa, double **occ_real_map, std::string robot_topic, std::vector<frontier_position> &frontier_vector){
    double max, min, max_cell;

    min = 9999999;
    max = 0;


//    std::string number = static_cast<std::ostringstream*>( &(std::ostringstream() << contador) )->str();

//    std::ofstream myfile;
//    std::string filename = "/home/rcolares/catkin_ws/src/ros-pioneer3at/maps/minmax"+robot_topic+".txt";
//    myfile.open (filename.c_str(),  std::ios::out | std::ios::app );
//    myfile<<frontier_vector.size()<<"\n";
//    myfile.close();
    for(int i = 0; i<frontier_vector.size();i++){
        frontier_vector[i].window = 0.0;
        for(int e = 0; e <  frontier_vector[i].x.size(); e++)
            frontier_vector[i].window += fWindow(occ_real_map, frontier_vector[i].x[e], frontier_vector[i].y[e], robot_topic);
//    myfile.open (filename.c_str(),  std::ios::out | std::ios::app );
//    myfile<<frontier_vector[i].window<<std::endl;
//    myfile<<frontier_vector[i].x.size()<<std::endl;
//    myfile.close();
    }

    for(int i = 0; i<frontier_vector.size();i++){
        if(frontier_vector[i].window > max)
            max = frontier_vector[i].window;
        if(frontier_vector[i].window < min)
            min = frontier_vector[i].window;
    }
//    myfile.open (filename.c_str(),  std::ios::out | std::ios::app );
//    myfile<<min<<std::endl;
//    myfile<<max<<std::endl;
//    myfile.close();
    for(int i = 0; i<frontier_vector.size();i++)
        frontier_vector[i].window = ((frontier_vector[i].window - min)/(max-min));

    //save_map(mapa, width, height, robot_id, frontier_vector);
}


void save_info(double **information_map, int width, int height, std::string robot_id){

    char sysCall[512];
    char number[512];
    sprintf ( number, "%d", contador++);


    std::string mapdatafile = "/home/rcolares/catkin_ws/src/ros-pioneer3at/maps/information_"+robot_id;
    mapdatafile+=robot_id;
    mapdatafile+="_";
    mapdatafile+=number;
    sprintf(sysCall, "%s.ppm", mapdatafile.c_str());
    FILE* printFile = fopen(sysCall, "w");
    fprintf(printFile, "P6\n # particles.ppm \n %d %d\n", width, height);
    fprintf(printFile, "255\n");




    for(unsigned int y = 0; y <height; y++) {
        for(unsigned int x = 0; x < width; x++) {
            if(information_map[y][x]==0)
                fprintf(printFile, "%c%c%c", 0, 0, 0);
            else
                fprintf(printFile, "%c%c%c", (int)(information_map[y][x]/2), 0, 255 - (int)(information_map[y][x]/2));
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

void information_potential(int width, int height, int **real_map, double **information_map, double **occ_real_map, std::string robot_topic, std::vector<frontier_position> &frontier_vector){

    double **aux_map;
    double min, max, f, sigma;
    min = 9999999;
    max = 0;

    sigma=0.01;

    std::string number = static_cast<std::ostringstream*>( &(std::ostringstream() << contador) )->str();

    aux_map = (double**) malloc(height*sizeof(double*));
    for(int i=0; i<height; i++)
        aux_map[i] = (double*) malloc(width*sizeof(double*));


//    std::ofstream myfile;
    std::ofstream myfile2;
//    std::string filename = "/home/rcolares/catkin_ws/src/ros-pioneer3at/maps/info_map_" + robot_topic +"_"+number+".txt";
    std::string filename2 = "/home/rcolares/catkin_ws/src/ros-pioneer3at/maps/info_potential_" + robot_topic +"_"+number+".txt";
//    myfile.open (filename.c_str(),  std::ios::out);// | std::ios::app );
    myfile2.open (filename2.c_str(),  std::ios::out);// | std::ios::app );

    for(int i=2;i<height-2;i++){
        for(int e=2;e<width-2;e++){
            aux_map[i][e] = 0;
            f=0;
            for(int k = i-1;k<=i+1;k++){
                for(int j = e-1; j<=e+1;j++){
                    if(occ_real_map[k][j]>0 && occ_real_map[k][j]<1)// && check_unknown(real_map, k, j) && real_map[k][j]!=+100)
                        f += exp(-((pow((occ_real_map[k][j]-0.5),2))/(2*pow(sigma,2)))) + exp(-((pow((occ_real_map[k][j]-0.5),2))/(2*pow(sigma,2))));
                    else
                        f+=0;
                }
            }
//            myfile<<occ_real_map[i][e]<<" ";
            myfile2<<f<<" ";
            information_map[i][e]=f;
//            if(f>max)
//                max = f;
//            if(f<min)
//                min=f;
        }
//        myfile<<"\n";
        myfile2<<"\n";
    }
    myfile2.close();

    for(int i = 0; i < frontier_vector.size();i++){
        for(int e = 0; e < frontier_vector[i].x.size(); e++){
            int x = frontier_vector[i].x[e];
            int y = frontier_vector[i].y[e];
            aux_map[y][x] = 1;
            if(information_map[y][x]>max)
                max = information_map[y][x];
            if(information_map[y][x]<min)
                min = information_map[y][x];
        }
    }


    for(int i=0;i<height;i++){
        for(int e=0;e<width;e++){
            if(aux_map[i][e] == 1)
                information_map[i][e] = (information_map[i][e]-min)/(max-min)*510;
            else
                information_map[i][e] = 0;
        }
    }

    save_info(information_map, width, height, robot_topic);

    for(int i=0;i<height;i++)
        free(aux_map[i]);
    free(aux_map);

}



/*******************************************************************************/



double entropia_fWindow(double **occ_real_map, int x, int y, std::string robot_topic){

    double fjanela=0.0;

//    std::ofstream myfile;
//    std::string filename = "/home/rcolares/catkin_ws/src/ros-pioneer3at/maps/teste" + robot_topic +".txt";
//    myfile.open (filename.c_str(),  std::ios::out | std::ios::app );
//    myfile <<"-----------------------------------------------------------\n";

    for(int i = y-WINDOW_SIZE/2; i<=y+WINDOW_SIZE/2; i++){
        for(int e = x-WINDOW_SIZE/2; e<=x+WINDOW_SIZE/2;e++){
//            myfile<<i<<" "<<e<<" "<<occ_real_map[i][e]<<"\n";
            if(occ_real_map[i][e]>0  && occ_real_map[i][e]<1){
                fjanela += (-((occ_real_map[i][e]*log(occ_real_map[i][e])) + ((1-occ_real_map[i][e])*log((1-occ_real_map[i][e])))));

            }else
                fjanela+=0;
        }
    }
//    myfile.close();
    return fjanela;
}


void entropy(int width, int height, int **real_map, double **mapa, double **occ_real_map, std::string robot_topic, std::vector<frontier_position> &frontier_vector){
    int x,y;
    double min = 9999999;
    double max = 0;


//    std::ofstream myfile;
//    std::string filename = "/home/rcolares/catkin_ws/src/ros-pioneer3at/maps/entropy" + robot_topic +"_"".txt";
//    myfile.open (filename.c_str(),  std::ios::out | std::ios::app );
//    myfile <<frontier_vector.size() <<"\n";
//    myfile.close();
    for(int i=0;i<frontier_vector.size();i++){
        for(int e = 0; e <  frontier_vector[i].x.size(); e++){
            x = frontier_vector[i].x[e];
            y = frontier_vector[i].y[e];

            frontier_vector[i].window += entropia_fWindow(occ_real_map,  x,  y, robot_topic);
        }
//        myfile.open (filename.c_str(),  std::ios::out | std::ios::app );
//        myfile<<frontier_vector[i].window<<std::endl;
//        myfile<<frontier_vector[i].x.size()<<std::endl;
//        myfile.close();

//        myfile << i<<"  "<<frontier_vector[i].window<<"\n";
    }



    for(int i = 0; i<frontier_vector.size();i++){
        if(frontier_vector[i].window > max)
            max = frontier_vector[i].window;
        if(frontier_vector[i].window < min)
            min = frontier_vector[i].window;
    }
//    myfile.open (filename.c_str(),  std::ios::out | std::ios::app );
//    myfile<<min<<std::endl;
//    myfile<<max<<std::endl;
//    myfile.close();

    for(int i = 0; i<frontier_vector.size();i++)
        frontier_vector[i].window = ((frontier_vector[i].window - min)/(max-min));


}



void save_entropy(double **entropy_map, int width, int height, std::string robot_id){

    char sysCall[512];
    char number[512];
    sprintf ( number, "%d", contador++);


    std::string mapdatafile = "/home/rcolares/catkin_ws/src/ros-pioneer3at/maps/entropy_"+robot_id;
    mapdatafile+=robot_id;
    mapdatafile+="_";
    mapdatafile+=number;
    sprintf(sysCall, "%s.ppm", mapdatafile.c_str());
    FILE* printFile = fopen(sysCall, "w");
    fprintf(printFile, "P6\n # particles.ppm \n %d %d\n", width, height);
    fprintf(printFile, "255\n");




    for(unsigned int y = 0; y <height; y++) {
        for(unsigned int x = 0; x < width; x++) {
            if(entropy_map[y][x] == 0)
                fprintf(printFile, "%c%c%c", 0, 0, 0);
            else
                fprintf(printFile, "%c%c%c", (int)(entropy_map[y][x]/2), 0, 255 - (int)(entropy_map[y][x]/2));
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

int check_unknown(int **real_map, int i, int e){

    if(real_map[i-1][e-1]!= -1 && real_map[i-1][e-1]!=+100)
        return 1;
    if(real_map[i-1][e]!= -1 && real_map[i-1][e]!=+100)
        return 1;
    if(real_map[i-1][e+1]!= -1 && real_map[i-1][e+1]!=+100)
        return 1;
    if(real_map[i][e-1]!= -1 && real_map[i][e-1]!=+100)
        return 1;
    if(real_map[i][e+1]!= -1 && real_map[i][e+1]!=+100)
        return 1;
    if(real_map[i+1][e-1]!= -1 && real_map[i+1][e-1]!=+100)
        return 1;
    if(real_map[i+1][e]!= -1 && real_map[i+1][e]!=+100)
        return 1;
    if(real_map[i+1][e=1]!= -1 && real_map[i+1][e+1]!=+100)
        return 1;

    return 0;

}


void entropy2(int width, int height, int **real_map, double **entropy_map, double **occ_real_map, std::string robot_topic, std::vector<frontier_position> &frontier_vector){

    double **aux_map;
    double min, max, f;
    min = 9999999;
    max = 0;

    aux_map = (double**) malloc(height*sizeof(double*));
    for(int i=0; i<height; i++)
        aux_map[i] = (double*) malloc(width*sizeof(double*));


    std::string number = static_cast<std::ostringstream*>( &(std::ostringstream() << contador) )->str();

//    std::ofstream myfile;
    std::ofstream myfile2;
//    std::string filename = "/home/rcolares/catkin_ws/src/ros-pioneer3at/maps/map_" + robot_topic +"_"+number+".txt";
    std::string filename2 = "/home/rcolares/catkin_ws/src/ros-pioneer3at/maps/entropy_map_" + robot_topic +"_"+number+".txt";
//    myfile.open (filename.c_str(),  std::ios::out);// | std::ios::app );
    myfile2.open (filename2.c_str(),  std::ios::out);// | std::ios::app );

    for(int i=2;i<height-2;i++){
        for(int e=2;e<width-2;e++){
            aux_map[i][e]=0;
            f=0;
            for(int k = i-1;k<=i+1;k++){
                for(int j = e-1; j<=e+1;j++){
                    if(occ_real_map[k][j]>0 && occ_real_map[k][j]<1)// && check_unknown(real_map, k, j) && real_map[k][j]!=+100)
                        f +=  (-((occ_real_map[k][j]*log(occ_real_map[k][j])) + ((1-occ_real_map[k][j])*log((1-occ_real_map[k][j])))));
                    else
                        f+=0;
                }
            }
//            myfile<<occ_real_map[i][e]<<" ";
            myfile2<<f<<" ";
            entropy_map[i][e]=f;
//            if(f>max)
//                max = f;
//            if(f<min)
//                min=f;
        }
//        myfile<<"\n";
        myfile2<<"\n";
    }

    myfile2.close();

    for(int i = 0; i < frontier_vector.size();i++){
        for(int e = 0; e < frontier_vector[i].x.size(); e++){
            int x = frontier_vector[i].x[e];
            int y = frontier_vector[i].y[e];
            aux_map[y][x] = 1;
            if(entropy_map[y][x]>max)
                max = entropy_map[y][x];
            if(entropy_map[y][x]<min)
                min = entropy_map[y][x];
        }
    }


    for(int i=0;i<height;i++){
        for(int e=0;e<width;e++){
            if(aux_map[i][e]==1)
                entropy_map[i][e] = (entropy_map[i][e]-min)/(max-min)*510;
            else
                entropy_map[i][e] = 0;
        }
    }

    save_entropy(entropy_map, width, height, robot_topic);

    for(int i=0;i<height;i++)
        free(aux_map[i]);
    free(aux_map);


}









