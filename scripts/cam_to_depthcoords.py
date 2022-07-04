import rospy
import ros_numpy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point

import numpy as np

def callback_ptcloud(ptcloud_data):
    pc = ros_numpy.numpify(ptcloud_data)
    points=np.zeros((pc.shape[0],3))
    points[:,0] = pc['x']
    points[:,1] = pc['y']
    points[:,2] = pc['z']
    pnt = Point(x=points[0, 1], y=points[1, 1], z=points[2, 1])
    coord_pub.publish(pnt)


if __name__ == '__main__':
    rospy.init_node("cam_to_depthcoords", anonymous=True)
    rate = rospy.Rate(7)
    rospy.loginfo("Cam to Depth node started")
    
    rospy.Subscriber("/camera/depth/color/points", PointCloud2, callback_ptcloud)

    coord_pub = rospy.Publisher("/fish/pc_coords", Point)

    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()
