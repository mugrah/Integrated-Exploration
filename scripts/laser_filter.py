#!/usr/bin/env python
import rospy
import sys
import message_filters
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
import numpy as np


class LaserFilter:

    def __init__(self):
        self.laser_pub = rospy.Publisher('base_scan_filtered', LaserScan, queue_size=10)

        self.odom_sub = message_filters.Subscriber('base_pose_ground_truth', Odometry)
        self.laser_sub = message_filters.Subscriber('base_scan', LaserScan)
        self.ts = message_filters.TimeSynchronizer([self.odom_sub, self.laser_sub], 10) # syncronize topics
        self.ts.registerCallback(self.odomLaserCb)
        

    def buildLaserMsg(self, old_laser, new_laser):
        new_scan = LaserScan()
        new_scan = old_laser # take the lasers configs
        new_scan.ranges = new_laser # with ranges transformed
        return new_scan
    

    def odomLaserCb(self, odom_data, laser_data):
        scans = np.empty(len(laser_data.ranges), dtype=np.float32)
        updated = np.zeros(len(laser_data.ranges), dtype=np.int8)

        # avoid processing when robot is not moving
        if not(odom_data.twist.twist.linear.x == 0 and
            odom_data.twist.twist.linear.y == 0 and
            odom_data.twist.twist.angular.z == 0):

            # TODO: put the drifts in parameters
            drift_x = odom_data.twist.twist.linear.x * 0.1
            drift_y = odom_data.twist.twist.linear.y * 0.0
            drift_theta = odom_data.twist.twist.angular.z * 0.1
            
            for i in range(0, len(laser_data.ranges)):
                # convert to cartesian
                x_laser = laser_data.ranges[i] * np.cos(i*laser_data.angle_increment)
                y_laser = laser_data.ranges[i] * np.sin(i*laser_data.angle_increment)

                # do the filtering
                x = x_laser*np.cos(drift_theta) - y_laser*np.sin(drift_theta)
                y = x_laser*np.sin(drift_theta) + y_laser*np.cos(drift_theta)
                x += drift_x
                y += drift_y

                # go back to polar
                ang = math.atan2(y, x)
                # if ang < 0: # negative radians
                #     ang += 2*math.pi
                idx = ang*(len(scans))/(laser_data.angle_max - laser_data.angle_min)  # convert angle into index
                
                new_idx = (int(round(idx)) % len(scans)) # circular array

                new_range = math.sqrt(math.pow(x, 2) + math.pow(y, 2))

                if new_range > laser_data.range_max-0.1: # fine adjust to remove "numerial error"
                    new_range = laser_data.range_max
                
                scans[new_idx] = new_range
                updated[new_idx] = 1 # mark the updated indexes

            for j in range(0, len(updated)): # deal with rays not updated
                if updated[j] == 0:
                    if j == 0:
                        scans[j] = (scans[j+1] + scans[len(updated)-1])/2
                    elif j == (len(updated)-1):
                        scans[j] = (scans[0] + scans[j-1])/2
                    else:
                        scans[j] = (scans[j+1] + scans[j-1])/2

        else: # do nothing
            scans = laser_data.ranges
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