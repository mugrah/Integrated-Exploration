#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <ros/console.h>
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <cstring>
#include <vector>
#include <fstream>
#include <iostream>
#include <algorithm>


int cont=0;

std::string image_topic;
ros::Subscriber image_subscriber;



void ros_image_save_Callback(const sensor_msgs::Image image){

//    cv_bridge::CvImagePtr cv_ptr;

//    cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);

//    char name[500];
//    sprintf(name, "/home/rcolares/catkin_ws/src/ros-pioneer3at/map/camera%d/img%d.png", image_topic[6], cont);

//    cv::imwrite(name,cv_ptr->image);

}






int main( int argc, char* argv[] )
{

    // Initialize ROS
    ros::init(argc, argv, "Map_node");
    ros::NodeHandle n;
    ros::NodeHandle n_("~");

    n_.getParam("image_topic", image_topic);

    image_subscriber = n.subscribe(image_topic, 1, ros_image_save_Callback);

    ros::spin();

}
