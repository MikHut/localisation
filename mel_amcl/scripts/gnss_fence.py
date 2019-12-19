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
        self.site_datum = rospy.get_param("navsat_transform_node/datum")

        self.visualise_site_on_google = rospy.get_param("~visualise_site_on_google", False)

        # Check that we are in a single UTM zone
        self.error = False
        self.utm_zone_nums = []
        self.utm_zone_lets = []


        self.create_polygon()


        # Publishers
        self.fence_status_pub = rospy.Publisher('/gnss_fence/status_within', Bool, queue_size=1)


        # Subscribers
        self.navsat_subscriber = rospy.Subscriber('/gps/fix', NavSatFix, self.navsat_callback, queue_size = 10)



    def navsat_callback(self, msg):

        status_msg = Bool()

        # TODO - Handle different zones
        easting, northing, zone_num, zone_let = utm.from_latlon(msg.latitude, msg.longitude) # _, _ : zone_num, zone_let
        utm_coord = Point(easting, northing)

        if not self.error:
            status_msg.data = utm_coord.within(self.fence)  
        else:
            status_msg.data = False
            rospy.logerr("The field crosses a utm zone! Zones are: %s %s \n Current Robot position is in zone: %s %s ", self.utm_zone_nums, self.utm_zone_lets, zone_num, zone_let)

        
        self.fence_status_pub.publish(status_msg)



    def create_polygon(self):
        print 'creating polygon'
        print  self.gnss_fence_coords
        utm_coords = []
        latitude_list = []
        longitude_list = []
        for i in self.gnss_fence_coords:
            print i
            easting, northing, zone_num, zone_let = utm.from_latlon(i[0], i[1]) # TODO - Handle different zones
            utm_coords.append([easting, northing])
            self.utm_zone_nums.append(zone_num)
            self.utm_zone_lets.append(zone_let)
            latitude_list.append(i[0])
            longitude_list.append(i[1])

        print 'utm coords: ', utm_coords
        
        self.fence = Polygon(utm_coords)

        # Check entire field is in the same zone before continuing
        if not all(i == self.utm_zone_lets[0] for i in self.utm_zone_lets) and all(i == self.utm_zone_nums[0] for i in self.utm_zone_nums):
            self.error = True
            rospy.logerr("The field crosses a utm zone!!! Zones are: %s %s ", self.utm_zone_nums, self.utm_zone_lets)


        if self.visualise_site_on_google:
        
            import gmplot 
            import webbrowser
            import os
            
            gmap = gmplot.GoogleMapPlotter(self.site_datum[0], 
                                self.site_datum[1], 18, apikey="AIzaSyCPlq11AxZ9LYGbKL97YsQOTp-90A97Tko")      
            gmap.polygon(latitude_list, longitude_list, color = 'red')
            x,y = self.fence.exterior.xy 
            lat = []
            lon = []
            for i in range(len(x)):
                lat_i, lon_i = utm.to_latlon(x[i], y[i], self.utm_zone_nums[0], self.utm_zone_lets[0])
                lat.append(lat_i)
                lon.append(lon_i)
            gmap.plot(lat, lon, 'blue', edge_width=10)
            gmap.scatter( latitude_list, longitude_list, '# FF0000', size = 5, marker = False) 
            gmap.marker(self.site_datum[0], self.site_datum[1], 'red')

            file_name = os.getenv("HOME") + '/gmap.html'
            gmap.draw(file_name) 
            webbrowser.open('file://' + os.path.realpath(file_name))




if __name__ == '__main__':
    rospy.init_node('gnss_fence')
    server = GNSS_fence()
    rospy.spin()