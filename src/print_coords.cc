#include <ros/ros.h>
#include <ros/console.h>
#include <string>
#include <cstdio>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <geometry_msgs/Quaternion.h>
//#include <ar_track_alvar/Alvar.h>
//#include <ar_track_alvar/Marker.h>
//#include <ar_track_alvar/MarkerDetector.h>
// #include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
//#include "ar_track_alvar/AlvarMarker.h"
//#include "ar_track_alvar/AlvarMarkers.h"

#include <eigen3/Eigen/Dense>


ros::Subscriber coords;

double longest = 0.0;

void ros_coords_Callback(ar_track_alvar_msgs::AlvarMarkers alvos){

    if(alvos.markers.size()>0){
        for(int i=0;i<alvos.markers.size();i++){

            if(alvos.markers[i].id==6){
                double x = alvos.markers[i].pose.pose.position.x;
                double y = alvos.markers[i].pose.pose.position.y;
                double z = alvos.markers[i].pose.pose.position.z;
                double qx = alvos.markers[i].pose.pose.orientation.x;
                double qy = alvos.markers[i].pose.pose.orientation.y;
                double qz = alvos.markers[i].pose.pose.orientation.z;
                double qw = alvos.markers[i].pose.pose.orientation.w;



                double dist = sqrt( x*x + y*y + z*z);
                int id = alvos.markers[i].id;
                printf("ID\t Dist\t x\t y\t z\t\n");
                printf("%d\t %lf\t %lf\t %lf\t %lf\t\n\n\n\n", id, dist, x, y, z);

                /*Eigen::Vector4f q( qx, qy, qz, qw );
      double nq = q.dot(q);

      q *= sqrt( 2.0/nq );
      Eigen::MatrixXf mq(4,4);
      mq = q  * q.transpose(); // outer product

      Eigen::Matrix4f rotation;

      rotation <<  1.0-mq(1, 1)-mq(2, 2), mq(0, 1)-mq(2, 3)    , mq(0, 2)+mq(1, 3)    , x,
                  mq(0, 1)+mq(2, 3)    , 1.0-mq(0, 0)-mq(2, 2), mq(1, 2)-mq(0, 3)    , y,
                  mq(0, 2)-mq(1, 3)    , mq(1, 2)+mq(0, 3)    , 1.0-mq(0, 0)-mq(1, 1), z,
                  0.0                  , 0.0                  , 0.0                  , 1.0;

      std::cout << rotation << std::endl;
      std::cout << "\n\n\n";*/

                tf::Quaternion q(qx, qy, qz, qw);
                tf::Matrix3x3 m(q);
                double Roll, Pitch, Yaw;
                m.getRPY(Roll, Pitch, Yaw);
                printf("Roll\t Pitch\t Yaw\t\n");

                std::cout<<Roll<<"  "<<Pitch<<"  "<<Yaw<<"\n\n\n";

            }
        }
    }
}


int main( int argc, char* argv[] )
{
    // Initialize ROS
    ros::init(argc, argv, "Print_Coords");
    ros::NodeHandle n;
    ros::NodeHandle n_("~");

    std::string coords_topic = "/ar_pose_marker";


    coords = n.subscribe(coords_topic, 1, ros_coords_Callback);

    ros::spin();

}
