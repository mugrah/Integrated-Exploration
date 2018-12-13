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

int utilityFunction(int height, int width, double a_beta, double b_beta, double alpha, double beta, double gama){

    double max=0.0;
    unsigned int max_i;
    
    for(unsigned int y = 1; y < height; y++) {
        for(unsigned int x = 1; x < width; x++) {
            unsigned int i = x + (height - y - 1) * width;
            if(distCost[i] != -1.0)
                distCost[i] = pow(distCost[i], (a_beta-1.0))*pow((1.0-distCost[i]), (b_beta-1.0));

            uFunction[i] = beta*distCost[i] /*+ alpha*infGain[i] + gama*coordCost[i]*/;
            
            if(uFunction[i]>max){
                max=uFunction[i];
                max_i = i;
            }

        }
    }
    return max_i;
}

void publishGoal(ros::Publisher goal_pub, nav_msgs::Odometry pose, int maxUtility, gmapping::occMap map, tf::StampedTransform *transform)
{
    geometry_msgs::PoseStamped r_goal;
    double y = pose.pose.pose.position.y;
    double x = pose.pose.pose.position.x;
    double z = pose.pose.pose.position.z;
    double qx = pose.pose.pose.orientation.x;
    double qy = pose.pose.pose.orientation.y;
    double qz = pose.pose.pose.orientation.z;
    double qw = pose.pose.pose.orientation.w;

    uint yGoalMap = map.map.info.height - maxUtility/map.map.info.width - 1;
    uint xGoalMap = maxUtility - (maxUtility/map.map.info.width) * map.map.info.width; 
    // ROS_ERROR_STREAM("Goal Map " << yGoalMap << " " << xGoalMap << " " << maxUtility);
    // ROS_ERROR_STREAM("Robot Pose " << y << " " << x);

    r_goal.header.frame_id = map.map.header.frame_id;
    r_goal.header.stamp = ros::Time::now();

    r_goal.pose.position.x = x + ((xGoalMap*map.map.info.resolution)+map.map.info.origin.position.x) - transform->getOrigin().x();
    r_goal.pose.position.y = y + (((map.map.info.height-yGoalMap)*map.map.info.resolution) + map.map.info.origin.position.y) - transform->getOrigin().y();
    r_goal.pose.position.z = z;
    r_goal.pose.orientation.x = qx;
    r_goal.pose.orientation.y = qy;
    r_goal.pose.orientation.z = qz;
    r_goal.pose.orientation.w = qw;

    // ROS_ERROR_STREAM("Robot Goal " << r_goal.pose.position.y << " " << r_goal.pose.position.x);

    // ROS_ERROR_STREAM("Transform " << transform->getOrigin().y() << " " << transform->getOrigin().x());

    // ROS_ERROR_STREAM("Resolution " << map.map.info.resolution);

    // ROS_ERROR_STREAM("Height Width " << map.map.info.height << " " << map.map.info.width);
    
    // ROS_ERROR_STREAM("Map Origin " << map.map.info.origin.position.y << " " << map.map.info.origin.position.x);
    // ROS_ERROR_STREAM("Y goal calc " << map.map.info.height - yGoalMap << " " << yGoalMap << " " << map.map.info.height);
    goal_pub.publish(r_goal);

}

mapPose setNewGoal(ros::Publisher goal_pub, gmapping::occMap map, nav_msgs::Odometry pose, mapPose m_pose, tf::StampedTransform *transform, int a_beta, int b_beta, double alpha, double beta, double gama){

    int height = map.map.info.height;
    int width = map.map.info.width;

    int maxUtility;
    uFunction.assign(map.data.size(), 0.0);

    mapPose m_goal;

    distCost = calculate_cost_map(map, m_pose);
//    infGain = calculate_inf_map(map, m_pose);
//    coordCost = calculate_coord_map(map, m_pose);
    maxUtility = utilityFunction(height, width, a_beta, b_beta, alpha, beta, gama);
    publishGoal(goal_pub, pose, maxUtility, map, transform);

    return m_goal;


}

int verify_if_goal_is_frontier(gmapping::occMap map, mapPose m_goal) {

    double total, unknown;
    total = unknown = 0;

    for(unsigned int y = m_goal.y - 5; y < map.map.info.height + 5; y++) {
        for(unsigned int x = m_goal.x - 5; x < map.map.info.width + 5; x++) {
            unsigned int i = x + (map.map.info.height - y - 1) * map.map.info.width;
            if (map.data[i] > 0.3 || map.data[i] < 0.7)
                unknown++;
            total++;
        }
    }
    
    if (unknown / total < 0.7) {
        ROS_ERROR_STREAM(unknown / total);
        return 1;
    }
    return 0;
}





