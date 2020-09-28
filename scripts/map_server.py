#!/usr/bin/env python3
import rospy
import sys
import copy
import numpy as np

from pioneer3at.msg import OccMap
from stage_ros.msg import fiducials
from nav_msgs.msg import OccupancyGrid


class MapServer:

    def __init__(self):
        self.ns_prefix = rospy.get_param('~ns_prefix', '')
        print(self.ns_prefix)

        self.first = True

        self.occ_map_pub = rospy.Publisher('occ_map', OccMap, queue_size=1)
        self.map_pub = rospy.Publisher('map', OccupancyGrid, queue_size=1)
        
        self.map_sub = rospy.Subscriber('occ_gmapping', OccMap, self.gmappingMap)

        self.fiducial_sub = rospy.Subscriber('fiducials', fiducials, self.fiducialChecker)

        self.fiducials = {}
        self.maps = {}

    def gmappingMap(self, gmap):
        self.maps[self.ns_prefix] = gmap
        pub_map = self.mergeMaps()
        # pub_map = self.maps[self.ns_prefix]
        self.occ_map_pub.publish(pub_map)
        self.map_pub.publish(pub_map.map)


    def mergeMaps(self):
        aux = self.resizeMaps()
        pub_map = aux[self.ns_prefix]
        for item in aux:
            if item != self.ns_prefix:
                pub_map = self.stichMaps(pub_map, aux[item])
        return pub_map
        
    def stichMaps(self, pub_map, other_map):        
        height = pub_map.map.info.height
        width = pub_map.map.info.width
        
        for y in range(0,height):
            for x in range(0,width):
                t = x + (height - y - 1) * width
                if pub_map.map.data[t] == -1:
                    pub_map.map.data[t] = other_map.map.data[t]
                    pub_map.data[t] = other_map.data[t]
        return pub_map

    def resizeMaps(self):
        map_size = 1001
        aux = copy.deepcopy(self.maps)
        for item in self.maps:
            aux[item] = self.resizeMap(aux[item], map_size)
        return aux

    def getMapSize(self):
        map_size = 0
        for item in self.maps:
            height = self.maps[item].map.info.height
            width = self.maps[item].map.info.width
            if height > map_size:
                map_size = height
            if width > map_size:
                map_size = width
        return map_size+1


    def resizeMap(self, new_map, map_size):
        o_height = new_map.map.info.height
        o_width = new_map.map.info.width
        resolution = new_map.map.info.resolution
        origin_y = new_map.map.info.origin.position.y
        origin_x = new_map.map.info.origin.position.x 

        aux = OccMap()

        aux.map.header = new_map.map.header
        aux.map.info = new_map.map.info
        aux.map.info.height = map_size
        aux.map.info.width = map_size
        aux.map.info.origin.position.y = -map_size*resolution/2
        aux.map.info.origin.position.x = -map_size*resolution/2
        for i in range(0, map_size*map_size):
            aux.map.data.append(-1)
            aux.data.append(0.5)
        
        for i in range(0, o_height):
            for e in range(0, o_width):
                t = (o_height - (o_height -i))*o_width + e
                g = int((map_size - (map_size/2 - origin_y/resolution - i)- 1)*map_size + (map_size + origin_x/resolution + e))
                aux.map.data[g] = new_map.map.data[t]
                aux.data[g] = new_map.data[t]
        
        return aux

    def fiducialChecker(self, fiducials):
        for fiducial in fiducials.observations:
            robot_prefix = '/robot_' + str(fiducial.id - 1)
            self.fiducials[robot_prefix] = fiducial
            self.maps[robot_prefix] = rospy.wait_for_message(robot_prefix + '/occ_gmapping', OccMap)

def main(args):
    rospy.init_node('map_server', anonymous=False, log_level=rospy.INFO)
    map_server = MapServer()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main(sys.argv)

    