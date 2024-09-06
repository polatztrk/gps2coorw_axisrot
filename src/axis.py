#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
import math

# Global variable for count
count = 0

def callback(data):
    global count  # Declare count as global to modify the global variable
    base_lat = 39.89655995
    base_long = 32.77759353

    rospy.loginfo("Received GPS Fix:")
    rospy.loginfo("Latitude: %f", data.latitude)
    rospy.loginfo("Longitude: %f", data.longitude)
    rospy.loginfo("Altitude: %f", data.altitude)

    cur_lat = data.latitude
    cur_long = data.longitude

    if count == 0:
        global husky_lat_init, husky_long_init  # Declare as global to modify them
        husky_lat_init = cur_lat
        husky_long_init = cur_long

        # Avoid division by zero if husky_long_init is zero
        
        count = count + 1
    theta = math.atan2(husky_long_init, husky_lat_init)
    coor_x = (cur_long - base_long) * 111111
    coor_y = (cur_lat - base_lat) * 111111

    e = math.tan(theta)*coor_x
    q = x/math.cos(theta)
    b_e = coor_y-e
    p = math.sin(theta)*b_e
    x_t = p + q
    y_t = b_e*math.cos(theta)
    
    rospy.loginfo("Received point: x=%f, y=%f", x_t, y_t)
    
    coor_point = Point()
    coor_point.x = x_t
    coor_point.y = y_t
    coor_point.z = 0.0

    pub.publish(coor_point)

    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "gps_points"
    marker.id = 1
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = x_t
    marker.pose.position.y = y_t
    marker.pose.position.z = 0.0
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.a = 1.0
    morker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0

    marker_pub.publish(marker)

def gps_subscriber_node():
    global pub, marker_pub
    # Initialize the ROS node
    rospy.init_node('gps_subscriber', anonymous=True)

    # Create a publisher for 'coordinate' topic with Point messages
    pub = rospy.Publisher('coordinate', Point, queue_size=10)
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

    # Subscribe to the "/fix" topic with NavSatFix messages
    rospy.Subscriber('/fix', NavSatFix, callback)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        gps_subscriber_node()
    except rospy.ROSInterruptException:
        pass
