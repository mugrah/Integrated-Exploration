#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <geometry_msgs/PoseStamped.h>
#include "gmapping/occMap.h"
#include "utils.h"


void setNewGoal(gmapping::occMap map, mapPose m_pose){



}



void publishGoal(nav_msgs::Odometry pose)
{

    double y = pose.pose.pose.position.y;
    double x = pose.pose.pose.position.x;
    double z = pose.pose.pose.position.z;
    double qx = pose.pose.pose.orientation.x;
    double qy = pose.pose.pose.orientation.y;
    double qz = pose.pose.pose.orientation.z;
    double qw = pose.pose.pose.orientation.w;





    r_goal.header.frame_id = map_topic;
    r_goal.header.stamp = ros::Time::now();

    r_goal.pose.position.x = x+1;
    r_goal.pose.position.y = y;
    r_goal.pose.position.z = z;
    r_goal.pose.orientation.x = qx;
    r_goal.pose.orientation.y = qy;
    r_goal.pose.orientation.z = qz;
    r_goal.pose.orientation.w = qw;


    goal_pub.publish(r_goal);


}



