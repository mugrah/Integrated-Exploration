#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>


typedef struct mapPose{
    int x;
    int y;
    double yaw;
}a;

typedef struct odomPose{
    double x;
    double y;
    double yaw;
}b;


odomPose map2odom(mapPose mPose, nav_msgs::MapMetaData_ info, tf::StampedTransform transform){
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

mapPose odom2map(odomPose oPose, nav_msgs::MapMetaData_ info, tf::StampedTransform transform){
    mapPose mPose;

    double map_cell = info.resolution;
    double map_x = info.origin.position.x;
    double map_y = info.origin.position.y;
    double map_height = info.height;

    mPose.x = (map_x - oPose.x - transform.getOrigin().x())/map_cell;
    mPose.y = map_height - (oPose.y + transform.getOrigin().y() - map_y)/map_cell;
    mPose.yaw = 0.0;


    return mPose;
}


