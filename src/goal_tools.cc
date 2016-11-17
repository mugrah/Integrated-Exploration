#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <geometry_msgs/PoseStamped.h>

void set_new_goal(nav_msgs::Odometry pose)
{

    double y = pose.pose.pose.position.y;
    double x = pose.pose.pose.position.x;
    double z = pose.pose.pose.position.z;
    double qx = pose.pose.pose.orientation.x;
    double qy = pose.pose.pose.orientation.y;
    double qz = pose.pose.pose.orientation.z;
    double qw = pose.pose.pose.orientation.w;


    tf::StampedTransform transform;
    tf::TransformListener listener;

    try{
        listener.waitForTransform(map_topic, base_link_topic, ros::Time(0), ros::Duration(5.0));
        listener.lookupTransform(map_topic, base_link_topic, ros::Time(0), transform);
    }catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }


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
