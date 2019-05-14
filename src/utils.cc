#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include "utils.h"




odomPose map2odom(mapPose mPose, nav_msgs::MapMetaData info, tf::StampedTransform transform){
    odomPose oPose;

    double map_cell = info.resolution;
    double map_x = info.origin.position.x;
    double map_y = info.origin.position.y;
    double map_height = info.height;

    oPose.x = ((mPose.x*map_cell)+map_x) - transform.getOrigin().x();
    oPose.y = (((map_height-mPose.y)*map_cell)+map_y) - transform.getOrigin().y();

    oPose.yaw = 0.0;

    return oPose;
}

mapPose odom2map(nav_msgs::Odometry *pose, gmapping::occMap *map, tf::StampedTransform *transform, std::string robot){
    mapPose mPose;

    double map_cell = map->map.info.resolution;
    double map_x = map->map.info.origin.position.x;
    double map_y = map->map.info.origin.position.y;
    double map_height = map->map.info.height;
    double r_pose_x = pose->pose.pose.position.x;
    double r_pose_y = pose->pose.pose.position.y;
    
    mPose.x = (r_pose_x - map_x)/map_cell;
    mPose.y = map_height - (r_pose_y - map_y)/map_cell;
    mPose.yaw = 0.0;

    return mPose;
}

mapPose goal2map(geometry_msgs::PoseStamped *pose, gmapping::occMap *map, tf::StampedTransform *transform, std::string robot){
    mapPose mPose;

    double map_cell = map->map.info.resolution;
    double map_x = map->map.info.origin.position.x;
    double map_y = map->map.info.origin.position.y;
    double map_height = map->map.info.height;
    double r_pose_x = pose->pose.position.x;
    double r_pose_y = pose->pose.position.y;
    
    mPose.x = (r_pose_x - map_x)/map_cell;
    mPose.y = map_height - (r_pose_y - map_y)/map_cell;
    mPose.yaw = 0.0;

    return mPose;
}

void saveRawOdom(nav_msgs::Odometry pose, std::string robot_topic){

    double y = pose.pose.pose.position.y;
    double x = pose.pose.pose.position.x;
    double z = pose.pose.pose.position.z;
    double qx = pose.pose.pose.orientation.x;
    double qy = pose.pose.pose.orientation.y;
    double qz = pose.pose.pose.orientation.z;
    double qw = pose.pose.pose.orientation.w;

    double roll, pitch, yaw;
    tf::Quaternion qm(qx, qy, qz, qw);
    tf::Matrix3x3 mm(qm);
    mm.getRPY(roll, pitch, yaw);

    std::ofstream myfile;
    std::string filename = "/home/rcolares/catkin_ws/src/ros-pioneer3at/maps/" + robot_topic +"_raw_odom.txt";
    myfile.open (filename.c_str(),  std::ios::out | std::ios::app );
    myfile << x << " " << y << " " << z << " " << roll << " " << pitch << " " << yaw << "\n";
    myfile.close();

}

void saveOdomPose(odomPose pose, std::string robot_topic){
    std::ofstream myfile;
    std::string filename = "/home/rcolares/catkin_ws/src/ros-pioneer3at/maps/" + robot_topic +"_odom_pose.txt";
    myfile.open (filename.c_str(),  std::ios::out | std::ios::app );
    myfile << pose.x << " " << pose.y << " " << pose.yaw << "\n";
    myfile.close();
}

void saveMapPose(mapPose pose, std::string robot_topic){
    std::ofstream myfile;
    std::string filename = "/home/rcolares/catkin_ws/src/ros-pioneer3at/maps/" + robot_topic +"_map_pose.txt";
    myfile.open (filename.c_str(),  std::ios::out | std::ios::app );
    myfile << pose.x << " " << pose.y << " " << pose.yaw << "\n";
    myfile.close();
}


