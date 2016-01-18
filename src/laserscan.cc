#include <sensor_msgs/LaserScan.h>
#include <stdio.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <fstream>
#include <iostream>

ros::Subscriber coords;

int id=0;

void ros_coords_Callback(sensor_msgs::LaserScan scans){

    std::ofstream myfile;
    std::string filename = "/home/rcolares/catkin_ws/src/ros-pioneer3at/maps/debug.txt";
    myfile.open (filename.c_str(),  std::ios::out | std::ios::app );

    myfile<<id++<<"\n";

    for(int i=0;i<scans.ranges.size();i++){

        myfile << scans.ranges[i]<<"  ";




    }
    myfile<<"\n\n";
    myfile.close();

}



int main( int argc, char* argv[] )
{
    // Initialize ROS
    ros::init(argc, argv, "Print_Laser");
    ros::NodeHandle n;
    ros::NodeHandle n_("~");

    std::string coords_topic = "/Pioneer3AT/laserscan";


    coords = n.subscribe(coords_topic, 1, ros_coords_Callback);

    ros::spin();

}
