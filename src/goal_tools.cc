#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <geometry_msgs/PoseStamped.h>
#include "pioneer3at/OccMap.h"
#include "utils.h"
#include "frontier_tools.h"
#include "distance_cost.cc"
#include "frontier_tools.cc"


std::vector<double> distCost;
std::vector<double> normDist;
std::vector<double> infGain;
std::vector<double> coordCost;
std::vector<double> uFunction;
std::vector<frontier_data> frontiers;

nav_msgs::GetPlan mountGetPlanMsg(std::string frame_map, geometry_msgs::Pose orig, geometry_msgs::Pose dest){
    nav_msgs::GetPlan srv_msg;
    srv_msg.request.start.header.stamp = ros::Time::now();
    srv_msg.request.start.header.frame_id = frame_map;
    ROS_INFO("MAP FRAME ID: %s", frame_map.data());
    srv_msg.request.goal.header = srv_msg.request.start.header;

    srv_msg.request.start.pose = orig;
    srv_msg.request.goal.pose = dest;
    srv_msg.request.tolerance = 0.1;
    return srv_msg;
}

// int utilityFunction(int height, int width, double a_beta, double b_beta, double alpha, double beta, double gama){

//     double max=0.0;
//     unsigned int max_i;
    
//     // for(unsigned int y = 1; y < height; y++) {
//         // for(unsigned int x = 1; x < width; x++) {
//             // unsigned int i = x + (height - y - 1) * width;
//     for(int j = 0; j < frontiers.size(); j++){
//             // if(distCost[i] != -1.0)
//                 // distCost[i] = pow(distCost[i], (a_beta-1.0))*pow((1.0-distCost[i]), (b_beta-1.0));
//         unsigned int i = frontiers[j].center;
//         // distCost[i] = pow(distCost[i], (a_beta-1.0))*pow((1.0-distCost[i]), (b_beta-1.0));
//         uFunction[i] = beta*distCost[i] /*+ alpha*infGain[i] + gama*coordCost[i]*/;
        
        


//         if(uFunction[i]>max){
//             max=uFunction[i];
//             max_i = i;
//         }

//         // }
//     }
//     return max_i;
// }


int utilityFunction(std::vector<double> distances, double alpha, double beta){ // return the index of the best frontier
    double utility, best_utility = 0.0;
    int best_utility_idx = 0;
    for(int i = 0; i < distances.size(); i++){
        utility = pow(distances[i], (alpha-1.0))*pow((1.0-distances[i]), (beta-1.0));
        if(utility > best_utility){
            best_utility = utility;
            best_utility_idx = i;
        }
    }
    return best_utility_idx;
}

geometry_msgs::PoseStamped publishGoal(pioneer3at::OccMap *map, int max_utility_idx){
    geometry_msgs::PoseStamped r_goal;
    
    r_goal.header.frame_id = map->map.header.frame_id;
    r_goal.header.stamp = ros::Time::now();
    r_goal.pose.position.x = frontiers[max_utility_idx].x_mean * map->map.info.resolution + map->map.info.origin.position.x;
    r_goal.pose.position.y = (map->map.info.height - frontiers[max_utility_idx].y_mean) * map->map.info.resolution + map->map.info.origin.position.y;
    r_goal.pose.orientation.w = 1.0;

    return r_goal;
}

geometry_msgs::PoseStamped setNewGoal(pioneer3at::OccMap *map, nav_msgs::Odometry pose, mapPose m_pose, tf::StampedTransform *transform, int a_beta, int b_beta, double alpha, double beta, double gama, ros::ServiceClient path_srv){
    int height = map->map.info.height;
    int width = map->map.info.width;
    int max_utility_idx;
    float dist_min = 0, dist_max;
    nav_msgs::GetPlan frontier_path;
    geometry_msgs::Pose dest;

    uFunction.assign(map->data.size(), 0.0);

    mapPose m_goal;
    geometry_msgs::PoseStamped r_goal;
    r_goal.pose.position.z = -1;

    frontiers = createFrontiers(map);
    ROS_INFO("Number of frontiers: %lu", frontiers.size());
    if (frontiers.size() > 0){
        normDist.resize(frontiers.size());
        for(int i = 0; i < frontiers.size(); i++){
            dest.position.x = frontiers[i].x_mean * map->map.info.resolution + map->map.info.origin.position.x;
            dest.position.y = (map->map.info.height - frontiers[i].y_mean) * map->map.info.resolution + map->map.info.origin.position.y;
            dest.orientation.w = 1.0;
            // normDist[i] = 
            frontier_path = mountGetPlanMsg(map->map.header.frame_id, pose.pose.pose, dest);
            if(path_srv.call(frontier_path)){
                normDist[i] = frontier_path.response.plan.poses.size();
                ROS_INFO("Intermediate Plan Size: %f", normDist[i]);
                // full_path.insert(full_path.end(), frontier_path.response.plan.poses.begin(), frontier_path.response.plan.poses.end());
            }
            else{
                ROS_ERROR("Failed to call make_plan service in frontier %i", i);
                // return false;
            }
        }
        
        dist_min = normDist[0];
        dist_max = normDist[0];
        for(int i = 1; i < normDist.size(); i++){ // get max e min values
            if(normDist[i] < dist_min){
                dist_min = normDist[i];
            }
            if(normDist[i] > dist_max){
                dist_max = normDist[i];
            }
        }

        for(int i = 0; i < normDist.size(); i++){ // get distance normalized
            normDist[i] = (normDist[i] - dist_min)/(dist_max - dist_min);
        }
    
    //    infGain = calculate_inf_map(map, m_pose);
    //    coordCost = calculate_coord_map(map, m_pose);
        // maxUtility = utilityFunction(height, width, a_beta, b_beta, alpha, beta, gama);
        max_utility_idx = utilityFunction(normDist, alpha, beta);
        r_goal = publishGoal(map, max_utility_idx);
    }
    
    return r_goal;
}

bool verify_if_goal_is_frontier(pioneer3at::OccMap map, mapPose m_goal) {

    double total, unknown;
    total = unknown = 0.0;

    ROS_ERROR_STREAM("GOAL " << m_goal.x << "  " << m_goal.y);

    for(unsigned int y = m_goal.y - 5; y < m_goal.y + 5; y++) {
        for(unsigned int x = m_goal.x - 5; x < m_goal.x + 5; x++) {
            unsigned int i = x + (map.map.info.height - y - 1) * map.map.info.width;
            if (map.data[i] == -1)
                unknown++;
            total++;
        }
    }

    ROS_ERROR_STREAM("UNKNOWN / TOTAL  = " << unknown / total);
    if (unknown / total < 0.3) {
        return false;
    }

    return true;
}

bool verify_if_goal_is_near(nav_msgs::Odometry r_pose, geometry_msgs::PoseStamped r_goal) {

    double y = r_pose.pose.pose.position.y;
    double x = r_pose.pose.pose.position.x;
    double xd = r_goal.pose.position.x;
    double yd = r_goal.pose.position.y;

    if( sqrt((x-xd)*(x-xd) + (y-yd)*(y-yd)) < 2.0){
        return true;
    }

    return false;
}
