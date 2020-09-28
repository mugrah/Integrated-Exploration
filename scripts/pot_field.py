#!/usr/bin/env python3
import rospy
import sys
import tf
import math
import numpy as np
from geometry_msgs.msg import Pose2D # comment
from nav_msgs.msg import Odometry
import message_filters
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class PotentialField:

    def __init__(self):
        self.vel_msg = Twist()
        
        self.holonomic = rospy.get_param('~holonomic', False)

        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.vel_sub = rospy.Subscriber('raw_cmd_vel', Twist, self.velCb)
        self.laser_sub = message_filters.Subscriber('base_scan_filtered', LaserScan)
        self.odom_sub = message_filters.Subscriber('odom', Odometry)
        self.ts = message_filters.TimeSynchronizer([self.odom_sub, self.laser_sub], 10) # syncronize topics
        self.ts.registerCallback(self.odomLaserCb)

    def velCb(self, vel_data):
        self.vel_msg = vel_data

    def odomLaserCb(self, odom_data, laser_data):
        obs_range = rospy.get_param('~min_obstacle_range', 0.7)
        desai_d = rospy.get_param('~desai_const', 2.5)
        repul_g = rospy.get_param('~repulsive_gain', 0.05)

        #convert quartenion to rad
        (R, P, Y) = tf.transformations.euler_from_quaternion([0.0, 0.0, odom_data.pose.pose.orientation.z, odom_data.pose.pose.orientation.w])
        pose_theta = Y
        
        (repuls_x, repuls_y) = self.repulsiveVector(laser_data, obs_range)

        # transform from robot frame to global and apply gain
        force_x = -(repuls_x*math.cos(pose_theta) - repuls_y*math.sin(pose_theta)) * repul_g
        force_y = -(repuls_x*math.sin(pose_theta) + repuls_y*math.cos(pose_theta)) * repul_g

        # apply desai
        if(self.holonomic == False):
            self.vel_msg.linear.x += force_x * math.cos(pose_theta) + force_y * math.sin(pose_theta)
            self.vel_msg.angular.z += (-force_x * math.sin(pose_theta) + force_y * math.cos(pose_theta))/desai_d
        else:
            self.vel_msg.linear.x += force_x
            self.vel_msg.linear.y += force_y

        self.vel_pub.publish(self.vel_msg)


    # return the repulsive vector
    def repulsiveVector(self, lasers, min_obs_range):
        x = 0.0
        y = 0.0
        for i in range(len(lasers.ranges)):
            if lasers.ranges[i] <= min_obs_range :
                rep_force = (1/lasers.ranges[i] - 1/min_obs_range) * (1/lasers.ranges[i] - 1/min_obs_range)
                x += rep_force * np.cos(i*lasers.angle_increment+lasers.angle_min)
                y += rep_force * np.sin(i*lasers.angle_increment+lasers.angle_min)
        return(x, y)


def main(args):
    rospy.init_node('pot_field', anonymous=True)
    pf = PotentialField()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main(sys.argv)