import rospy
import ros_numpy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point

import numpy as np

most_recent_pc = np.zeros()

def callback_ptcloud(ptcloud_data):
    most_recent_pc = ros_numpy.numpify(ptcloud_data)

def callback_coordinput(coord):
    pnt = ros_numpy.numpify(coord)
    pnt_msg = Point(x=most_recent_pc['x', pnt[0]], y=most_recent_pc['y', pnt[1]], z=most_recent_pc['z', pnt[2]])
    coord_pub.publish(pnt_msg)

if __name__ == '__main__':
    rospy.init_node("cam_to_depthcoords", anonymous=True)
    rate = rospy.Rate(7)
    rospy.loginfo("Cam to Depth node started")
    
    rospy.Subscriber("/camera/depth/color/points", PointCloud2, callback_ptcloud)
    rospy.Subscriber("input_coords", Point, callback_coordinput)

    coord_pub = rospy.Publisher("pc_coords", Point, queue_size=10)

    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()
