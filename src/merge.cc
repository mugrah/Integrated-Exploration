#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/String.h>
#include <nav_msgs/GetMap.h>
#include <tf/transform_listener.h>
#include "tf/LinearMath/Matrix3x3.h"
#include <ros/console.h>

namespace nm=nav_msgs;
namespace gm=geometry_msgs;



double minX (nm::MapMetaData info)
 {
   gm::Polygon p=gridPolygon(info);
   return min(p.points[0].x, min(p.points[1].x, min(p.points[2].x, p.points[3].x)));
 }
 
 double maxX (const nm::MapMetaData& info)
 {
   const gm::Polygon p=gridPolygon(info);
   return max(p.points[0].x, max(p.points[1].x, max(p.points[2].x, p.points[3].x)));
 }
 
 double minY (const nm::MapMetaData& info)
 {
   const gm::Polygon p=gridPolygon(info);
   return min(p.points[0].y, min(p.points[1].y, min(p.points[2].y, p.points[3].y)));
 }
 
 double maxY (const nm::MapMetaData& info)
 {
   const gm::Polygon p=gridPolygon(info);
   return max(p.points[0].y, max(p.points[1].y, max(p.points[2].y, p.points[3].y)));
 }

 gm::Pose transformPose (const tf::Pose trans, const gm::Pose p)
 {
   tf::Pose pose;
   tf::poseMsgToTF(p, pose);
   gm::Pose transformed;
   tf::poseTFToMsg(trans*pose, transformed);
   return transformed;
 }
  
// Get the dimensions of a combined grid
 nav_msgs::MapMetaData getCombinedGridInfo (vector<nav_msgs::OccupancyGrid> grids, double resolution)
 {
   ROS_ASSERT (grids.size() > 0);
   nm::MapMetaData info;
   info.resolution = resolution;
   tf::Pose trans;
   tf::poseMsgToTF(grids[0]->info.origin, trans);
   
 
// #ifdef GRID_UTILS_GCC_46
// #pragma GCC diagnostic push
// #pragma GCC diagnostic ignored "-Wuninitialized"
// #endif
 
   boost::optional<double> min_x, max_x, min_y, max_y;
   BOOST_FOREACH (nm::OccupancyGrid g, grids) {
     nm::MapMetaData grid_info = g.info;
     grid_info.origin = transformPose(trans.inverse(), g.info.origin);
     if (!(min_x && *min_x < minX(grid_info)))
       min_x = minX(grid_info);
     if (!(min_y && *min_y < minY(grid_info)))
       min_y = minY(grid_info);
     if (!(max_x && *max_x > maxX(grid_info)))
       max_x = maxX(grid_info);
     if (!(max_y && *max_y > maxY(grid_info)))
       max_y = maxY(grid_info);
   }
   
// #ifdef GRID_UTILS_GCC_46
// #pragma GCC diagnostic pop
// #endif
 
   const double dx = *max_x - *min_x;
   const double dy = *max_y - *min_y;
   ROS_ASSERT ((dx > 0) && (dy > 0));
   gm::Pose pose_in_grid_frame;
   pose_in_grid_frame.position.x = *min_x;
   pose_in_grid_frame.position.y = *min_y;
   pose_in_grid_frame.orientation.w = 1.0;
   info.origin = transformPose(trans, pose_in_grid_frame);
   info.height = ceil(dy/info.resolution);
   info.width = ceil(dx/info.resolution);
 
   return info;
 }



nm::OccupancyGrid combineGrids (vector<nav_msg::OccupancyGrid> grids)
 {
   
   
   nm::OccupancyGrid combined_grid;
   combined_grid.info = getCombinedGridInfo(grids, grids[0]->info.resolution);  
   //AQUI   
   combined_grid->data.resize(combined_grid->info.width*combined_grid->info.height);
   fill(combined_grid->data.begin(), combined_grid->data.end(), -1);
   ROS_DEBUG_NAMED ("combine_grids", "Combining %zu grids", grids.size());
 
   BOOST_FOREACH (const GridConstPtr& grid, grids) {
     for (coord_t x=0; x<(int)grid->info.width; x++) {
       for (coord_t y=0; y<(int)grid->info.height; y++) {
         const Cell cell(x, y);
         const signed char value=grid->data[cellIndex(grid->info, cell)];
 
         // Only proceed if the value is not unknown 
         if ((value>=0) && (value<=100)) {
           BOOST_FOREACH (const Cell& intersecting_cell, 
                          intersectingCells(combined_grid->info, grid->info, cell)) {
             const index_t ind = cellIndex(combined_grid->info, intersecting_cell);
             combined_grid->data[ind] = max(combined_grid->data[ind], value);
           }
         }
       }
     }
   }
   
   ROS_DEBUG_NAMED ("combine_grids", "Done combining grids");
   return combined_grid;
 }
 
 
 GridPtr combineGrids (const vector<GridConstPtr>& grids)
 {
   ROS_ASSERT (grids.size()>0);
   return combineGrids(grids, grids[0]->info.resolution);
 }