#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point
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

def gps_subscriber_node():
    global pub
    # Initialize the ROS node
    rospy.init_node('gps_subscriber', anonymous=True)

    # Create a publisher for 'coordinate' topic with Point messages
    pub = rospy.Publisher('coordinate', Point, queue_size=10)

    # Subscribe to the "/fix" topic with NavSatFix messages
    rospy.Subscriber('/fix', NavSatFix, callback)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        gps_subscriber_node()
    except rospy.ROSInterruptException:
        pass
