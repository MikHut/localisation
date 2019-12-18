#!/usr/bin/env python

import rospy

import matplotlib.pyplot as plt
from shapely.geometry import Point, Polygon
import utm

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool



class GNSS_fence:

    def __init__(self):


        self.gnss_fence_coords = rospy.get_param("~gnss_fence_coords")

        self.create_polygon()


        # Publishers
        self.fence_status_pub = rospy.Publisher('/gnss_fence/status_within', Bool, queue_size=1)

        # Subscribers
        self.navsat_subscriber = rospy.Subscriber('/gps/fix', NavSatFix, self.navsat_callback, queue_size = 10)



    def navsat_callback(self, msg):

        # TODO - Handle different zones
        easting, northing, _, _ = utm.from_latlon(msg.latitude, msg.longitude) # _, _ : zone_num, zone_let
        utm_coord = Point(easting, northing)

        status_msg = Bool()
        status_msg.data = utm_coord.within(self.fence)
        
        self.fence_status_pub.publish(status_msg)



    def create_polygon(self):
        print 'creating polygon'
        print  self.gnss_fence_coords
        utm_coords = []
        for i in self.gnss_fence_coords:
            print i
            easting, northing, _, _ = utm.from_latlon(i[0], i[1]) # TODO - Handle different zones
            utm_coords.append([easting, northing])

        print 'utm coords: ', utm_coords

        self.fence = Polygon(utm_coords)




if __name__ == '__main__':
    rospy.init_node('gnss_fence')
    server = GNSS_fence()
    rospy.spin()