#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <geometry_msgs/PoseStamped.h>
#include "gmapping/occMap.h"
#include "utils.h"
#include "distance_cost.cc"

std::vector<double> distCost;
std::vector<double> infGain;
std::vector<double> coordCost;
std::vector<double> uFunction;

int utilityFunction(int a_beta, int b_beta, double alpha, double beta, double gama){

    double max=0.0;
    int max_i;

    for(unsigned int y = 0; y < height; y++) {
        for(unsigned int x = 0; x < width; x++) {
            unsigned int i = x + (height - y - 1) * width;
            if(distCost[i] != -1.0)
                distCost[i] = pow(distCost[i], (a_beta-1))*pow((1-distCost[i]), (b_beta-1));

            uFunction[i] = beta*distCost[i] /*+ alpha*infGain[i] + gama*coordCost[i]*/;
            if(uFunction[i]>max){
                max=uFunction;
                max_i = i;
            }

        }
    }
    return max_i;
}

void publishGoal(ros::Publisher goal_pub, nav_msgs::Odometry pose, int maxUtility, gmapping::occMap map, tf::StampedTransform transform)
{
    geometry_msgs::PoseStamped r_goal;
    double y = pose.pose.pose.position.y;
    double x = pose.pose.pose.position.x;
    double z = pose.pose.pose.position.z;
    double qx = pose.pose.pose.orientation.x;
    double qy = pose.pose.pose.orientation.y;
    double qz = pose.pose.pose.orientation.z;
    double qw = pose.pose.pose.orientation.w;

    int yGoalMap = maxUtility/map.map.info.height;
    int xGoalMap = maxUtility - yGoalMap*map.map.info.height;

    r_goal.header.frame_id = map_topic;
    r_goal.header.stamp = ros::Time::now();

    r_goal.pose.position.x = ((xGoalMap*map.map.info.resolution)+map.map.info.origin.position.x) - transform.getOrigin().x();
    r_goal.pose.position.y = (((map.map.info.height-yGoalMap)*map.map.info.resolution) + map.map.info.origin.position.y) - transform.getOrigin().y();
    r_goal.pose.position.z = z;
    r_goal.pose.orientation.x = qx;
    r_goal.pose.orientation.y = qy;
    r_goal.pose.orientation.z = qz;
    r_goal.pose.orientation.w = qw;

    goal_pub.publish(r_goal);

}

mapPose setNewGoal(ros::Publisher goal_pub, gmapping::occMap map, nav_msgs::Odometry pose, mapPose m_pose, tf::StampedTransform transform, int a_beta, int b_beta, double alpha, double beta, double gama){

    int maxUtility;
    mapPose m_goal;

    distCost = calculate_cost_map(map, m_pose);
//    infGain = calculate_inf_map(map, m_pose);
//    coordCost = calculate_coord_map(map, m_pose);
    maxUtility = utilityFunction(a_beta, b_beta, alpha, beta, gama);
    publishGoal(pose, maxUtility, map, transform);

    return m_goal;


}





