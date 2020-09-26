#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <geometry_msgs/PoseStamped.h>
#include "pioneer3at/OccMap.h"
#include "utils.h"
#include "frontier_tools.h"
#include "distance_cost.cc"
#include "frontier_tools.cc"
#include "information_gain.cc"

typedef struct cost{
    int frontier_idx;
    double value;
}cost;

typedef struct utility{
    double dist_cost;
    double inf_gain;
    double coord;
}utility;

// std::vector<double> distCost;
// std::vector<double> infGain;
// std::vector<double> coordCost;
// std::vector<double> uFunction;
std::vector<frontier_data> frontiers;

std::map<int, utility> utility_map; 

nav_msgs::GetPlan mountGetPlanMsg(std::string frame_map, geometry_msgs::Pose orig, geometry_msgs::Pose dest){
    nav_msgs::GetPlan srv_msg;
    srv_msg.request.start.header.stamp = ros::Time::now();
    srv_msg.request.start.header.frame_id = frame_map;
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


int utilityFunction(std::list<cost> dist_cost, std::list<cost> inf_gain, double gama1, double gama2, double gama3){ // return the index of the best frontier
    // double utility, best_utility = 0.0;
    // double dist, inf, coord = 0.0;
    double value = 0.0;
    // std::map<int, double> dist, inf, coord;
    std::map<int, double> utility;
    double best_utility = 0.0;
    int best_utility_idx = 0;
    for (std::list<cost>::iterator it = dist_cost.begin(); it != dist_cost.end(); ++it){
        value = gama2 * it->value;
        auto m_it = utility.find(it->frontier_idx); 
        if (m_it == utility.end()) {
            utility.insert(std::make_pair(it->frontier_idx, value));
        } else {
            m_it->second += value;
        }
    }
    for (std::list<cost>::iterator it = inf_gain.begin(); it != inf_gain.end(); ++it){
        value = gama1 * it->value;
        auto m_it = utility.find(it->frontier_idx); 
        if (m_it == utility.end()) {
            utility.insert(std::make_pair(it->frontier_idx, value));
        } else {
            m_it->second += value;
        }
    }

    for(std::map<int, double>::iterator it = utility.begin(); it != utility.end(); ++it){
        if(it->second > best_utility){
            best_utility = it->second;
            best_utility_idx = it->first;
        }
    }
    return best_utility_idx;
}

geometry_msgs::PoseStamped mountGoal(pioneer3at::OccMap *map, int max_utility_idx){
    geometry_msgs::PoseStamped r_goal;
    
    r_goal.header.frame_id = map->map.header.frame_id;
    r_goal.header.stamp = ros::Time::now();
    r_goal.pose.position.x = frontiers[max_utility_idx].x_mean * map->map.info.resolution + map->map.info.origin.position.x;
    r_goal.pose.position.y = (map->map.info.height - frontiers[max_utility_idx].y_mean) * map->map.info.resolution + map->map.info.origin.position.y;
    r_goal.pose.orientation.w = 1.0;
    return r_goal;
}

std::list<cost> calculate_dist_cost(pioneer3at::OccMap *map, nav_msgs::Odometry pose, mapPose m_pose, std::string path_srv_name, double robot_radius, double alpha, double beta){
    int height = map->map.info.height;
    int width = map->map.info.width;
    unsigned int ic;
    
    float dist_min = 0, dist_max;
    double max=0.0;
    nav_msgs::GetPlan frontier_path;
    geometry_msgs::Pose dest;
    std::list<cost> dist_cost;
    cost cost_aux;
    std::vector<double> wavefront_map;


    frontiers = createFrontiers(map, robot_radius);
    ROS_INFO("Number of frontiers: %lu", frontiers.size());
    if (frontiers.size() == 0){
        ROS_ERROR_STREAM("FRONTIER SIZE IS ZERO");
        return dist_cost; // no more frontiers
    }

    wavefront_map = wavefront(map, m_pose, max);

    for(int i = 0; i < frontiers.size(); i++){ // evaluate path distance from frontiers
        dest.position.x = frontiers[i].x_mean * map->map.info.resolution + map->map.info.origin.position.x;
        dest.position.y = (map->map.info.height - frontiers[i].y_mean) * map->map.info.resolution + map->map.info.origin.position.y;
        dest.orientation.w = 1.0;
        frontier_path = mountGetPlanMsg(map->map.header.frame_id, pose.pose.pose, dest);
        ros::service::waitForService(path_srv_name, 5000);
        if(ros::service::call(path_srv_name, frontier_path)){
            
            if(frontier_path.response.plan.poses.size() > 0){
                cost_aux.frontier_idx = i;
                cost_aux.value = frontier_path.response.plan.poses.size();
                dist_cost.push_back(cost_aux);
            } else {
                ic = frontiers[i].x_mean + (height - frontiers[i].y_mean - 1) * width;
                cost_aux.frontier_idx = i;
                cost_aux.value = wavefront_map[ic];
                dist_cost.push_back(cost_aux);
            }
        }
        else{
            ROS_ERROR("Failed to call make_plan service in frontier %i", i);
            // return false;
        }
    }
    if(dist_cost.empty()){ // didn't find any valid path
        ROS_ERROR_STREAM("DIST COST IS EMPTY");
        return dist_cost;
    }

    dist_min = dist_cost.front().value; // finding the min e max values
    dist_max = dist_cost.front().value;
    for (std::list<cost>::iterator it = dist_cost.begin(); it != dist_cost.end(); ++it){
        if(it->value < dist_min){
            dist_min = it->value;
        }
        if(it->value > dist_max){
            dist_max = it->value;
        }
    }

    for (std::list<cost>::iterator it = dist_cost.begin(); it != dist_cost.end(); ++it){
        it->value = (it->value - dist_min)/(dist_max - dist_min); // normalize the distance
        it->value = pow(it->value, (alpha-1.0))*pow((1.0-it->value), (beta-1.0));
    }

    dist_min = dist_cost.front().value; // finding the min e max values
    dist_max = dist_cost.front().value;
    for (std::list<cost>::iterator it = dist_cost.begin(); it != dist_cost.end(); ++it){
        if(it->value < dist_min){
            dist_min = it->value;
        }
        if(it->value > dist_max){
            dist_max = it->value;
        }
    }
    for (std::list<cost>::iterator it = dist_cost.begin(); it != dist_cost.end(); ++it){
        it->value = (it->value - dist_min)/(dist_max - dist_min); // normalize the distance
    }

    return dist_cost;

}

std::list<cost> calculate_inf_gain(pioneer3at::OccMap *map, int n_size, double sigma){
    std::vector<double> inf_map;
    std::list<cost> inf_gain;
    cost cost_aux;
    
    int height = map->map.info.height;
    int width = map->map.info.width;
    unsigned int ic;
    double inf_min = 0, inf_max;

    inf_map = calculate_information_map(map, n_size, sigma);
    for(int i = 0; i < frontiers.size(); i++){
        cost_aux.value = 0.0;
        for(int e = 0; e < frontiers[i].x.size(); e++){
            ic = frontiers[i].x[e] + (height - frontiers[i].y[e] - 1) * width;
            cost_aux.value += inf_map[ic];

        }
        cost_aux.frontier_idx = i;
        inf_gain.push_back(cost_aux);
    }

    inf_min = inf_gain.front().value; // finding the min e max values
    inf_max = inf_gain.front().value;
    for (std::list<cost>::iterator it = inf_gain.begin(); it != inf_gain.end(); ++it){
        if(it->value < inf_min){
            inf_min = it->value;
        }
        if(it->value > inf_max){
            inf_max = it->value;
        }
    }

    for (std::list<cost>::iterator it = inf_gain.begin(); it != inf_gain.end(); ++it){
        it->value = (it->value - inf_min)/(inf_max - inf_min); // normalize the distance
    }

    return inf_gain;
}

bool setNewGoal(pioneer3at::OccMap *map, nav_msgs::Odometry pose, mapPose m_pose, int alpha, int beta, double gama1, double gama2, double gama3, std::string path_srv_name, ros::Publisher goal_pub, double robot_radius, int n_size, double sigma){
    std::list<cost> dist_cost;
    std::list<cost> inf_gain;
    int max_utility_idx;
    geometry_msgs::PoseStamped new_goal;
    

    dist_cost = calculate_dist_cost(map, pose, m_pose, path_srv_name, robot_radius, alpha, beta);
    if (dist_cost.size() == 0) {
        // No more frontiers
        return false;
    }
    inf_gain = calculate_inf_gain(map, n_size, sigma);
//    coordCost = calculate_coord_map(map, m_pose);
    // maxUtility = utilityFunction(height, width, a_beta, b_beta, alpha, beta, gama);
    max_utility_idx = utilityFunction(dist_cost, inf_gain, gama1, gama2, gama3);
    new_goal = mountGoal(map, max_utility_idx);
    goal_pub.publish(new_goal);

    return true;
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
