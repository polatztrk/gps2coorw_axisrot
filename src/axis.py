#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from gps2coorw_axisrot.msg import GPSData  # Replace 'custom_msgs' with the actual package name for GPSData.msg
import math

# Global variables for husky's initial GPS data
husky_lat_init = None
husky_long_init = None

def gps_topic_callback(data):
    """
    Callback function to store the initial GPS data from the gps_topic.
    This function is called only once to get the initial latitude and longitude.
    """
    rospy.logwarn("first")
    global husky_lat_init, husky_long_init
    if husky_lat_init is None and husky_long_init is None:
        husky_lat_init = data.latitude
        husky_long_init = data.longitude
        rospy.loginfo("Initialized husky's GPS data from gps_topic: Lat = %f, Long = %f", husky_lat_init, husky_long_init)

def callback(data):
    """
    Callback for /fix topic to process the current GPS data and apply transformations.
    """
    global husky_lat_init, husky_long_init

    if husky_lat_init is None or husky_long_init is None:
        rospy.logwarn("Waiting for initial GPS data from gps_topic...")
        return

    base_lat = 39.89653978
    base_long = 32.77760805

    rospy.loginfo("Received GPS Fix:")
    rospy.loginfo("Latitude: %f", data.latitude)
    rospy.loginfo("Longitude: %f", data.longitude)
    rospy.loginfo("Altitude: %f", data.altitude)

    cur_lat = data.latitude
    cur_long = data.longitude

    # Compute transformation angle based on initial husky GPS data
    theta = math.atan2(husky_long_init, husky_lat_init)
    coor_x = (cur_long - base_long) * 111111
    coor_y = (cur_lat - base_lat) * 111111

    e = math.tan(theta) * coor_x
    q = coor_x / math.cos(theta)
    b_e = coor_y - e
    p = math.sin(theta) * b_e
    x_t = p + q
    y_t = b_e * math.cos(theta)
    
    rospy.loginfo("Transformed point: x=%f, y=%f", x_t, y_t)

    # Publish Point message
    coor_point = Point()
    coor_point.x = x_t
    coor_point.y = y_t
    coor_point.z = 0.0
    point_pub.publish(coor_point)

    # Publish markers for axes
    publish_axes(0, 0, 0, 1, 0, 0, 0, 1.0, 0, "Original X-axis", 12)
    publish_axes(0, 0, 0, 0, 1, 0, 0, 1.0, 0, "Original Y-axis", 71)

    # Rotated X and Y axes
    rotated_x = math.cos(theta)
    rotated_y = math.sin(theta)
    publish_axes(0, 0, 0, rotated_x, rotated_y, 0, 0, 1.0, 1.0, "Rotated X-axis", 7)
    publish_axes(0, 0, 0, -rotated_y, rotated_x, 0, 0, 1.0, 1.0, "Rotated Y-axis", 9)

    # Publish the point as a sphere in RViz
    publish_point_as_marker(x_t, y_t)

def publish_axes(start_x, start_y, start_z, end_x, end_y, end_z, r, g, b, ns, asd):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = ns
    marker.id = 2007 + asd
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.scale.x = 0.025
    marker.scale.y = 0.025
    marker.scale.z = 0.025
    marker.color.a = 1.0
    marker.color.r = r
    marker.color.g = g
    marker.color.b = b
    marker.pose.orientation.w = 1.0
    marker.points.append(Point(start_x, start_y, start_z))
    marker.points.append(Point(end_x, end_y, end_z))
    axis_marker_pub.publish(marker)

def publish_point_as_marker(x, y):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "transformed_point"
    marker.id = 2003
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0.0
    marker.scale.x = 0.2
    marker.scale.y = 0.2
    marker.scale.z = 0.2
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    point_marker_pub.publish(marker)

def gps_subscriber_node():
    global point_pub, axis_marker_pub, point_marker_pub
    rospy.init_node('gps_subscriber', anonymous=True)

    # Create publishers
    point_pub = rospy.Publisher('coordinate', Point, queue_size=10)
    axis_marker_pub = rospy.Publisher('visualization_marker/axes', Marker, queue_size=10)
    point_marker_pub = rospy.Publisher('visualization_marker/points', Marker, queue_size=10)

    # Subscribe to the gps_topic to get initial GPS data
    rospy.Subscriber('/gps_data', GPSData, gps_topic_callback)

    # Subscribe to the /fix topic to process current GPS data
    rospy.Subscriber('/fix', NavSatFix, callback)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        gps_subscriber_node()
    except rospy.ROSInterruptException:
        pass
