#include <stdlib.h>
#include <stdio.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "pioneer3at/poses.h"
#include <string>




int cont =0;

void savemap(int **mapa, int height, int width, char *name){

    char sysCall[512];
    char number[512];
    //   sprintf ( number, "%d", contador++);

    std::string mapdatafile = "/home/rcolares/catkin_ws/src/ros-pioneer3at/maps/";
    mapdatafile+=name;
    sprintf(sysCall, "%s.ppm", mapdatafile.c_str());
    FILE* printFile = fopen(sysCall, "w");
    fprintf(printFile, "P6\n # particles.ppm \n %d %d\n", width, height);
    fprintf(printFile, "255\n");


    for(unsigned int y = 0; y <height; y++) {
        for(unsigned int x = 0; x < width; x++) {
            if(mapa[y][x] ==100)
                fprintf(printFile, "%c%c%c", 0, 0, 0);
            else if(mapa[y][x]==0)
                fprintf(printFile, "%c%c%c", 255, 255, 255);
            else
                fprintf(printFile, "%c%c%c", 52, 52, 52);
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

    //   ROS_INFO("Done\n");
}


void savemappos(int **mapa, int height, int width, char *name, pioneer3at::poses r){

    char sysCall[512];
    char number[512];
    //   sprintf ( number, "%d", contador++);

    std::string mapdatafile = "/home/rcolares/catkin_ws/src/ros-pioneer3at/maps/";
    mapdatafile+=name;
    sprintf(sysCall, "%s.ppm", mapdatafile.c_str());
    FILE* printFile = fopen(sysCall, "w");
    fprintf(printFile, "P6\n # particles.ppm \n %d %d\n", width, height);
    fprintf(printFile, "255\n");


    for(unsigned int y = 0; y <height; y++) {
        for(unsigned int x = 0; x < width; x++) {
            if((y >= r.y_other-5 && y<=r.y_other+5) && (x >= r.x_other-5 && x<=r.x_other+5))
                fprintf(printFile, "%c%c%c", 255, 0, 0);
            else if(mapa[y][x] ==100)
                fprintf(printFile, "%c%c%c", 0, 0, 0);
            else if(mapa[y][x]==0)
                fprintf(printFile, "%c%c%c", 255, 255, 255);
            else
                fprintf(printFile, "%c%c%c", 52, 52, 52);
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

    //   ROS_INFO("Done\n");
}



void compute_dmap(int **m, int **d_map, int c, int width, int height){
    FILE *out;
    out =fopen("/home/rcolares/catkin_ws/src/ros-pioneer3at/maps/tests/window_9x9/my_method/2/d_map.txt", "w");
    for(int y = 0; y < height; y++){
        for(int x = 0; x < width; x++){
            if(m[y][x] == c)
                d_map[y][x] = 0;
            else
                d_map[y][x] = (width * height)+1;
        }
    }
    for(int y = 1; y < height; y++){
        for(int x = 1; x < width; x++){
            int h;
            if((d_map[y-1][x]+1) < (d_map[y][x-1]+1))
                h = d_map[y-1][x]+1;
            else
                h = d_map[y][x-1]+1;

            if(d_map[y][x]>h)
                d_map[y][x] = h;
        }
    }
    for(int y = height-2; y > 0; y--){
        for(int x = width-2; x > 0; x--){
            int h;
            if((d_map[y+1][x]+1) < (d_map[y][x+1]+1))
                h = d_map[y+1][x]+1;
            else
                h = d_map[y][x+1]+1;

            if(d_map[y][x]>h)
                d_map[y][x] = h;

            fprintf(out, "%d,", d_map[y][x]);
        }
        fprintf(out,"\n");
    }
    fclose(out);
}


int compute_d1(int **map1, int **map2, int **d_map, int c, int width, int height){
    int d1=0;

    compute_dmap(map2, d_map, c, width, height);

    for(int y = 0; y < height; y++){
        for(int x = 0; x < width; x++){
            if(map1[y][x]==c)
                d1 = d1 + d_map[y][x];
        }
    }
    return d1;
}

int compute_d2(int **map1, int **map2, int **d_map, int c, int width, int height){
    int d2=0;
    
    compute_dmap(map1, d_map, c, width, height);

    for(int y = 0; y < height; y++){
        for(int x = 0; x < width; x++){
            if(map2[y][x]==c)
                d2 = d2 + d_map[y][x];
        }
    }
    return d2;
}

int compute_similarity(int **map1, int **map2, int width, int height){
    int **d_map;
    int c[] = {0, 255};
    int sim =0;

    d_map = (int**) malloc(height*sizeof(int*));
    for(int i=0; i<height; i++)
        d_map[i] = (int*) malloc(width*sizeof(int*));

    for(int i = 0; i<2; i++){
        sim = sim + compute_d1(map1, map2, d_map, c[i], width, height) + compute_d2(map1, map2, d_map, c[i], width, height);
    }


    for(int i=0;i<height;i++)
        free(d_map[i]);
    free(d_map);

    return sim;

}

int agreed(int **map1, int **map2, pioneer3at::poses r1, pioneer3at::poses r2){

    int agreed = 0;

    if(r1.x < r1.x_other){
        if(r1.y < r1.y_other){
            for(int y = r1.y-30; y <= r1.y_other + 30; y++){
                for(int x = r1.x-30; x <= r1.x_other+30; x++){
                    if(map1[y][x] != 125 && map2[r1.y_other - (r1.y -y)][r1.x_other - (r1.x -x)] != 125){
                        if(map1[y][x] == map2[r1.y_other - (r1.y -y)][r1.x_other - (r1.x -x)])
                            agreed++;
                    }
                }
            }
        }else{
            for(int y = r1.y_other-30; y <= r1.y + 30; y++){
                for(int x = r1.x-30; x <= r1.x_other+30; x++){
                    if(map1[y][x] != 125 && map2[r1.y_other - (r1.y -y)][r1.x_other - (r1.x -x)] != 125){
                        if(map1[y][x] == map2[r1.y_other - (r1.y -y)][r1.x_other - (r1.x -x)])
                            agreed++;
                    }
                }
            }
        }
    }else{
        if(r1.y < r1.y_other){
            for(int y = r1.y-30; y <= r1.y_other + 30; y++){
                for(int x = r1.x_other-30; x <= r1.x+30; x++){
                    if(map1[y][x] != 125 && map2[r1.y_other - (r1.y -y)][r1.x_other - (r1.x -x)] != 125){
                        if(map1[y][x] == map2[r1.y_other - (r1.y -y)][r1.x_other - (r1.x -x)])
                            agreed++;
                    }
                }
            }
        }else{
            for(int y = r1.y_other-30; y <= r1.y + 30; y++){
                for(int x = r1.x_other-30; x <= r1.x+30; x++){
                    if(map1[y][x] != 125 && map2[r1.y_other - (r1.y -y)][r1.x_other - (r1.x -x)] != 125){
                        if(map1[y][x] == map2[r1.y_other - (r1.y -y)][r1.x_other - (r1.x -x)])
                            agreed++;
                    }
                }
            }
        }
    }
    
    return agreed;
}

int disagreed(int **map1, int **map2, pioneer3at::poses r1, pioneer3at::poses r2){

    int disagreed = 0;

    if(r1.x < r1.x_other){
        if(r1.y < r1.y_other){
            for(int y = r1.y-30; y <= r1.y_other + 30; y++){
                for(int x = r1.x-30; x <= r1.x_other+30; x++){
                    if(map1[y][x] != 125 && map2[r1.y_other - (r1.y -y)][r1.x_other - (r1.x -x)] != 125){
                        if(map1[y][x] != map2[r1.y_other - (r1.y -y)][r1.x_other - (r1.x -x)])
                            disagreed++;
                    }
                }
            }
        }else{
            for(int y = r1.y_other-30; y <= r1.y + 30; y++){
                for(int x = r1.x-30; x <= r1.x_other+30; x++){
                    if(map1[y][x] != 125 && map2[r1.y_other - (r1.y -y)][r1.x_other - (r1.x -x)] != 125){
                        if(map1[y][x] != map2[r1.y_other - (r1.y -y)][r1.x_other - (r1.x -x)])
                            disagreed++;
                    }
                }
            }
        }
    }else{
        if(r1.y < r1.y_other){
            for(int y = r1.y-30; y <= r1.y_other + 30; y++){
                for(int x = r1.x_other-30; x <= r1.x+30; x++){
                    if(map1[y][x] != 125 && map2[r1.y_other - (r1.y -y)][r1.x_other - (r1.x -x)] != 125){
                        if(map1[y][x] != map2[r1.y_other - (r1.y -y)][r1.x_other - (r1.x -x)])
                            disagreed++;
                    }
                }
            }
        }else{
            for(int y = r1.y_other-30; y <= r1.y + 30; y++){
                for(int x = r1.x_other-30; x <= r1.x+30; x++){
                    if(map1[y][x] != 125 && map2[r1.y_other - (r1.y -y)][r1.x_other - (r1.x -x)] != 125){
                        if(map1[y][x] != map2[r1.y_other - (r1.y -y)][r1.x_other - (r1.x -x)])
                            disagreed++;
                    }
                }
            }
        }
    }
    
    return disagreed;
}

void translate(cv::Mat& src, cv::Mat& dst, pioneer3at::poses mine, pioneer3at::poses other){

    int len = std::max(src.cols, src.rows);
    cv::Mat warp_mat( 2, 3, CV_32FC1 );
    warp_mat.at<float>(0,0) = 1.0;
    warp_mat.at<float>(0,1) = 0;
    warp_mat.at<float>(0,2) = mine.x_other - other.x;
    warp_mat.at<float>(1,0) = 0;
    warp_mat.at<float>(1,1) = 1;
    warp_mat.at<float>(1,2) = mine.y_other - other.y;

    cv::warpAffine(src, dst, warp_mat, cv::Size(len, len), cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(52.0, 52.0, 52.0));
}


void rotate(cv::Mat& src, double angle, cv::Mat& dst, pioneer3at::poses r2)
{
    int len = std::max(src.cols, src.rows);
    //     cv::Point2f pt(len/2., len/2.);
    cv::Point2f pt(r2.x_other, r2.y_other);
    cv::Mat r = cv::getRotationMatrix2D(pt, angle, 1.0);

    cv::warpAffine(src, dst, r, cv::Size(len, len), cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(52.0, 52.0, 52.0));
}


void stitchmap(int **map, int **map1, int **map2, double **occ_map, double **occ_map1, double **occ_map2, pioneer3at::poses r1, pioneer3at::poses r2, int height, int width, double angle){
    
    pioneer3at::poses r_c;
    double d1x, d1y, d2x, d2y;

    //std::ofstream myfile;
    //std::string filename = "/home/rcolares/catkin_ws/src/ros-pioneer3at/maps/stitch.txt";

    //myfile.open (filename.c_str(),  std::ios::out | std::ios::app );
    //myfile <<"testing boundaries"<<std::endl;
    //myfile.close();

    for(int y=0;y<height;y++){
        for(int x=0;x<width;x++){
            if(map1[y][x]!=-1){
                map[y][x]=map1[y][x];
                occ_map[y][x] = occ_map1[y][x];

                //       }else if(map2[r2.y_other - (r1.y -y)][r2.x_other - (r1.x -x)]!=-1)
                //         map[y][x]=map2[r2.y_other - (r1.y -y)][r2.x_other - (r1.x -x)];
            }else if(map2[y][x]!=-1){
                map[y][x]=map2[y][x];
                occ_map[y][x] = occ_map2[y][x];
            }else{
                map[y][x]=-1;
                occ_map[y][x]=-1;
            }
        }
    }
    //   298   289 ---- 277   244  49.6588
    //   273   244 ---- 298   287  49.7393

    //myfile.open (filename.c_str(),  std::ios::out | std::ios::app );
    //myfile <<"defining name"<<std::endl;
    //myfile.close();

    char name[512];
    sprintf ( name, "global_map_%lf", angle);
    //   std::string name = "final_map"+number;

    //myfile.open (filename.c_str(),  std::ios::out | std::ios::app );
    //myfile <<name<<std::endl;
    //myfile.close();

    savemap(map, height, width, name);

    //myfile.open (filename.c_str(),  std::ios::out | std::ios::app );
    //myfile <<"map saved"<<std::endl;
    //myfile.close();
}

void stitchsamemap(int **map, int **map1, int **map2, double **occ_map, double **occ_map1, double **occ_map2, int height, int width, double angle){
    
    for(int y=0;y<height;y++){
        for(int x=0;x<width;x++){
            if(map1[y][x]!=-1){
                map[y][x]=map1[y][x];
                occ_map[y][x] = occ_map1[y][x];
            }else if(map2[y][x]!=-1){
                map[y][x]=map2[y][x];
                occ_map[y][x] = occ_map2[y][x];
            }
            else{
                map[y][x]=-1;
                occ_map[y][x] = -1;
            }
        }
    }
    //   char name[512];
    //   sprintf ( name, "local_map_%lf", angle);
    //   std::string name = "final_map"+number;
    //   savemap(map, height, width, name);
}


void map2cv(int **map2, cv::Mat& map2_rotate, int height, int width){

    for( int i = 0; i < height; ++i)
        for( int j = 0; j < width; ++j)
            map2_rotate.at<float>(i,j) = map2[i][j];

}

void occmap2cv(double **map, cv::Mat& cv_occMap, int height, int width){
    for( int i = 0; i < height; ++i)
        for( int j = 0; j < width; ++j)
            cv_occMap.at<float>(i,j) = map[i][j];
}

void cv2occmap(double **map, cv::Mat& cv_occMap, int height, int width){
    for( int i = 0; i < height; ++i)
        for( int j = 0; j < width; ++j)
            map[i][j] = cv_occMap.at<float>(i,j);
}

void cv2map(int **map2, cv::Mat& map2_rotate, int height, int width){

    for( int i = 0; i < height; ++i)
        for( int j = 0; j < width; ++j)
            map2[i][j] = (int) map2_rotate.at<float>(i,j);

}  
