#!/usr/bin/env python
import rospy
import sys
import message_filters
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
import numpy as np


class LaserFilter:

    def __init__(self):
        self.laser_pub = rospy.Publisher('base_scan_filtered', LaserScan, queue_size=10)

        self.vel_sub = rospy.Subscriber('cmd_vel', Twist, self.velCb)
        self.odom_sub = message_filters.Subscriber('base_pose_ground_truth', Odometry)
        self.laser_sub = message_filters.Subscriber('base_scan', LaserScan)
        self.ts = message_filters.TimeSynchronizer([self.odom_sub, self.laser_sub], 10) # syncronize topics
        self.ts.registerCallback(self.odomLaserCb)

        self.vel = Twist()
        

    def buildLaserMsg(self, old_laser, new_laser):
        new_scan = LaserScan()
        new_scan = old_laser # take the lasers configs
        new_scan.ranges = new_laser # with ranges transformed
        return new_scan
    
    def velCb(self, velocity):
        self.vel = velocity


    def odomLaserCb(self, odom_data, laser_data):
        updated = np.zeros(len(laser_data.ranges), dtype=np.int8)
        drift_theta = 11.0
        drift_linear = 0.1

        scans = np.roll(laser_data.ranges, 0) # turn lasers into numpy array

        # avoid processing when robot is not moving
        if odom_data.twist.twist.angular.z != 0:
            idx_drift = int(round(odom_data.twist.twist.angular.z * drift_theta))
            scans = np.roll(laser_data.ranges, idx_drift)

        if odom_data.twist.twist.linear.x != 0  or odom_data.twist.twist.linear.y != 0 :
            drift_linear *= -self.vel.linear.x
            aux_scans = scans.copy()

            for i in range(0, len(aux_scans)):
                if aux_scans[i] >= laser_data.range_max:
                    updated[i] = 1 # mark the updated indexes
                    continue    
                x = aux_scans[i] * np.cos(i*laser_data.angle_increment+laser_data.angle_min) - drift_linear
                y = aux_scans[i] * np.sin(i*laser_data.angle_increment+laser_data.angle_min)

                idx = int(round((math.atan2(y, x) - laser_data.angle_min) / laser_data.angle_increment))
                scans[idx] = math.sqrt(x*x + y*y)
                updated[idx] = 1 # mark the updated indexes

            for i in range(0, len(updated)): # deal with rays not updated
                if updated[i] == 1:
                    continue # updated
                if i == 0: # first
                    next_id = scans[i+1]
                    prev_id = scans[-1]
                elif i == (len(updated)-1): # last
                    next_id = scans[0]
                    prev_id = scans[-2]
                else:
                    next_id = scans[i+1]
                    prev_id = scans[i-1]

                update_scan = (next_id + prev_id) / 2
                if abs(update_scan - scans[i]) < 2*drift_linear:
                    scans[i] = update_scan

        self.laser_pub.publish(self.buildLaserMsg(laser_data, scans))


def main(args):
    rospy.init_node('laser_filter', anonymous=False)
    l_f = LaserFilter()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main(sys.argv)