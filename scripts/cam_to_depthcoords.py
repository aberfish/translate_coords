import rospy
import ros_numpy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point

import numpy as np

most_recent_pc = None

def callback_ptcloud(depth_msg):
    global most_recent_pc

    bridge = CvBridge()
     # Use cv_bridge() to convert the ROS image to OpenCV format
    try:
     #Convert the depth image using the default passthrough encoding
        depth_image = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        most_recent_pc = np.array(depth_image, dtype=np.float32)
        center_idx = np.array(most_recent_pc.shape) / 2
        print ('center depth:', most_recent_pc[center_idx[0], center_idx[1]])

    except CvBridgeError as e:
        print(e)


def callback_coordinput(coord):
    if most_recent_pc is None:
        rospy.logwarn("No pointcloud data recieved yet")
        return
    pnt = ros_numpy.numpify(coord)
    pnt_msg = Point(x=pnt[0], y=pnt[1], z=most_recent_pc[pnt[0], pnt[1]])
    coord_pub.publish(pnt_msg)

if __name__ == '__main__':
    rospy.init_node("cam_to_depthcoords", anonymous=True)
    rate = rospy.Rate(30)
    rospy.loginfo("Cam to Depth node started")
    
    rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, callback=callback_ptcloud)
    rospy.Subscriber("input_coords", Point, callback_coordinput)

    coord_pub = rospy.Publisher("pc_coords", Point, queue_size=10)

    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()