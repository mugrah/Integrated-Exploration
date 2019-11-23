#!/usr/bin/env python
import rospy
import sys
import math
from nav_msgs.msg import Odometry

class TraveledDistance:

    def __init__(self):
        self.step = rospy.get_param('~step', 0.5)
        self.file = open(rospy.get_param('~file_name', 'dist.txt'), "w")
        self.prevX = 0.0
        self.prevY = 0.0

        self.vel_sub = rospy.Subscriber('odom', Odometry, self.odomCb)

    def odomCb(self, odom):
        newX = odom.pose.pose.position.x
        newY = odom.pose.pose.position.y
        dist = math.sqrt((newX-self.prevX) * (newX-self.prevX) + (newY-self.prevY) * (newY-self.prevY))
        if dist > self.step:
            self.file.write(str(newX) + " " + str(newY) + "\n")
            self.prevX = newX
            self.prevY = newY

def main(args):
    rospy.init_node('traveled_distance', anonymous=False)
    t_d = TraveledDistance()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main(sys.argv)