import rospy
import ros_numpy
from sensor_msgs.msg import PointCloud2

import numpy as np

def callback_ptcloud(ptcloud_data):
    pc = ros_numpy.numpify(ptcloud_data)
    points=np.zeros((pc.shape[0],3))
    points[:,0] = pc['x']
    points[:,1] = pc['y']
    points[:,2] = pc['z']
    rospy.logerr(pc)


if __name__ == '__main__':
    rospy.init_node("cam_to_depthcoords", anonymous=True)
    rate = rospy.Rate(7)
    rospy.loginfo("Cam to Depth node started")
    
    rospy.Subscriber("/camera/depth/color/points", PointCloud2, callback_ptcloud)

    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()
