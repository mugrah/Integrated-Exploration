#include <fstream>
#include <iostream>

#include "frontiers.h"

#define A 1
#define B 0
#define D 1

typedef struct goalCell{
  int x;
  int y;
  int valid;
}ff;

goalCell goal_cell;


void chooseGoal(std::vector<frontier_position> &frontier_vector, std::string robot_topic){

    double f_max = 0;
    double f;
    double size = 0;
    goal_cell.valid=0;

    std::ofstream myfile;
    std::string filename = "/home/rcolares/catkin_ws/src/ros-pioneer3at/maps/function"+robot_topic+".txt";
    myfile.open (filename.c_str(),  std::ios::out);// | std::ios::app );

//    for(int i=0; i<frontier_vector.size(); i++){
////        if(frontier_vector[i].neg_wave!=-1)
////            f = A*frontier_vector[i].window + B*frontier_vector[i].wave - D*frontier_vector[i].neg_wave;
////        else
////            f = A*frontier_vector[i].window + B*frontier_vector[i].wave;
////        myfile<<f<<" "<<A*frontier_vector[i].window<<" "<<B*frontier_vector[i].wave<<" "<<D*frontier_vector[i].neg_wave<<std::endl;
//        int x = frontier_vector[i].x_mean;
//        int y = frontier_vector[i].y_mean;
//        myfile<<x<<" "<<y<<" ";
//        //if(x>250 && x < 1250 && y>250 && y<1250 )
//            f = A*frontier_vector[i].window;
//        //else
//          //  f=-1;

//        myfile<<f<<"\n";
//        //if(i==4){
//            if(f > f_max){
//                f_max = f;
//                goal_cell.valid=1;
//                size = frontier_vector[i].x.size();
//                goal_cell.x = frontier_vector[i].x_mean;
//                goal_cell.y = frontier_vector[i].y_mean;
//            }else if (f==f_max && frontier_vector[i].x.size() > size){
//                f_max = f;
//                goal_cell.valid=1;
//                size = frontier_vector[i].x.size();
//                goal_cell.x = frontier_vector[i].x_mean;
//                goal_cell.y = frontier_vector[i].y_mean;
//            }
//        //}

//    }

    goal_cell.valid=1;
    goal_cell.x = frontier_vector[1].x_mean;
    goal_cell.y = frontier_vector[1].y_mean;

    myfile<<"---------------------------\n";
    myfile.close();

}




