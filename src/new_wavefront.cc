#include <fstream>
#include <cstdlib>
#include <iostream>
#include <string>

#include "frontiers.h"

#define MAP_FREE 999999

#define ALPHAS 3
#define BETAS 7
#define C 252

int contador_wave = 0;

void wavefront(double **map, int height, int width, int x, int y, std::string robot_topic){
    int changes = 1;
    if(map[y][x]!=-1){
        map[y][x] = 0;
    }else{
        double dist = 9999999;
        int xr, yr;
        for(int i = y-5;i<y+5;i++){
            for(int e = x-5; e<x+5;e++){
                if(map[i][e] == MAP_FREE){
                    double d = sqrt(pow((x-e),2) + pow((y-i), 2));
                    if(d<dist){
                        dist=d;
                        xr=e;
                        yr=i;
                    }
                }
            }
        }
        map[yr][xr] = 0;
    }


    while(changes!=0){
        changes = 0;

        for(int i = 1; i < height; i++){
            for(int e = 1; e < width; e ++){
                if(map[i][e] != MAP_FREE && map[i][e]!=-1){

                    if(map[i-1][e]==MAP_FREE){
                        map[i-1][e] = map[i][e]+1;
                        changes=1;
                    }
                    if(map[i-1][e-1]==MAP_FREE){
                        map[i-1][e-1] = map[i][e]+1;
                        changes=1;
                    }
                    if(map[i-1][e+1]==MAP_FREE){
                        map[i-1][e+1] = map[i][e]+1;
                        changes=1;
                    }
                    if(map[i+1][e]==MAP_FREE){
                        map[i+1][e] = map[i][e]+1;
                        changes=1;
                    }
                    if(map[i+1][e-1]==MAP_FREE){
                        map[i+1][e-1] = map[i][e]+1;
                        changes=1;
                    }
                    if(map[i+1][e+1]==MAP_FREE){
                        map[i+1][e+1] = map[i][e]+1;
                        changes=1;
                    }
                    if(map[i][e-1]==MAP_FREE){
                        map[i][e-1] = map[i][e]+1;
                        changes=1;
                    }
                    if(map[i][e+1]==MAP_FREE){
                        map[i][e+1] = map[i][e]+1;
                        changes=1;
                    }
                }
            }
        }

        for(int i = height-1; i >= 0; i--){
            for(int e = width-1; e >=0; e --){
                if(map[i][e] != MAP_FREE && map[i][e]!=-1){

                    if(map[i-1][e]==MAP_FREE){
                        map[i-1][e] = map[i][e]+1;
                        changes=1;
                    }
                    if(map[i-1][e-1]==MAP_FREE){
                        map[i-1][e-1] = map[i][e]+1;
                        changes=1;
                    }
                    if(map[i-1][e+1]==MAP_FREE){
                        map[i-1][e+1] = map[i][e]+1;
                        changes=1;
                    }
                    if(map[i+1][e]==MAP_FREE){
                        map[i+1][e] = map[i][e]+1;
                        changes=1;
                    }
                    if(map[i+1][e-1]==MAP_FREE){
                        map[i+1][e-1] = map[i][e]+1;
                        changes=1;
                    }
                    if(map[i+1][e+1]==MAP_FREE){
                        map[i+1][e+1] = map[i][e]+1;
                        changes=1;
                    }
                    if(map[i][e-1]==MAP_FREE){
                        map[i][e-1] = map[i][e]+1;
                        changes=1;
                    }
                    if(map[i][e+1]==MAP_FREE){
                        map[i][e+1] = map[i][e]+1;
                        changes=1;
                    }
                }
            }
        }
    }
}




void multi_wavefront(int width, int height, double **wave_map, double **neg_wave, int x, int y, int x_other, int y_other, std::string robot_topic, std::vector<frontier_position> &frontier_vector){

    wavefront(wave_map, height, width, x, y, robot_topic);

    double max = 0.0;
    double min = 9999999;
    double neg_max = 0.0;
    double neg_min = 999999;

    for(int i=0; i<height; i++){
        for(int e = 0; e<width; e++){
            if(wave_map[i][e]!= -1 && wave_map[i][e]!= MAP_FREE){
                if(wave_map[i][e] > max)
                    max = wave_map[i][e];
                if(wave_map[i][e] < min)
                    min = wave_map[i][e];
            }
        }
    }


    for(int i=0; i<height; i++)
        for(int e = 0; e<width; e++)
            if(wave_map[i][e]!= -1 && wave_map[i][e]!= MAP_FREE)
                wave_map[i][e] = (wave_map[i][e] - min)/(max-min);


    if(x_other != -1){
        wavefront(neg_wave, height, width, x_other, y_other, robot_topic);

        for(int i=0; i<height; i++){
            for(int e = 0; e<width; e++){
                if(neg_wave[i][e]!= -1 && neg_wave[i][e]!= MAP_FREE){
                    if(neg_wave[i][e] > neg_max)
                        neg_max = neg_wave[i][e];
                    if(neg_wave[i][e] < neg_min)
                        neg_min = neg_wave[i][e];
                }
            }
        }

        for(int i=0; i<height; i++)
            for(int e = 0; e<width; e++)
                if(wave_map[i][e]!= -1 && wave_map[i][e]!= MAP_FREE)
                    neg_wave[i][e] = (neg_wave[i][e] - neg_min)/(neg_max-neg_min);



    }
    max = 0.0;
    min = 9999999;
    neg_max = 0.0;
    neg_min = 999999;



    for(int i=0; i<frontier_vector.size(); i++){
        int xf = frontier_vector[i].x_mean;
        int yf = frontier_vector[i].y_mean;

        frontier_vector[i].wave = (pow((wave_map[yf][xf]),(ALPHAS-1))*pow((1-(wave_map[yf][xf])),(BETAS-1)))*C;

        if(frontier_vector[i].wave > max)
            max = frontier_vector[i].wave;
        if(frontier_vector[i].wave < min)
            min = frontier_vector[i].wave;

//        if(x_other !=-1){
//            frontier_vector[i].neg_wave = (pow((neg_wave[yf][xf]),(ALPHAS-1))*pow((1-(neg_wave[yf][xf])),(BETAS-1)))*C;
//            if(frontier_vector[i].neg_wave > neg_max)
//                neg_max = frontier_vector[i].neg_wave;
//            if(frontier_vector[i].neg_wave < neg_min)
//                neg_min = frontier_vector[i].neg_wave;
//        }

    }

    for(int i=0; i<frontier_vector.size(); i++){
        frontier_vector[i].wave = (frontier_vector[i].wave - min)/(max-min);

//        if(x_other !=-1){
//            frontier_vector[i].neg_wave = (frontier_vector[i].neg_wave - neg_min)/(neg_max-neg_min);
//        }
    }

}
