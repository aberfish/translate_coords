"""PKG: translatecoords     NODE: cam_to_depth_coords.py
Converts 2d pixel coordinates from depth camera colour images to realworld 3d coordinates measured in metres, where 0,0,0 is WHERE IS 0,0,0 I DONT KNOW ITS SOME PLACE IN THE ETHERRRRRR
"""

import rospy
import ros_numpy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image

import numpy as np
import pyrealsense2 as rs2

# to publish test input coordinate:
# rostopic pub -1 /input_coords geometry_msgs/Point "250" "250" "0"
# max coords = 480, 640

latest_depth_img = None
intrinsics = None

def callback_depthimg(depth_msg):
    global latest_depth_img

    bridge = CvBridge()
     # Use cv_bridge() to convert the ROS image to OpenCV format
    try:
        #Convert the depth image using the default passthrough encoding
        depth_image = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        latest_depth_img = np.array(depth_image, dtype=np.float32)

    except CvBridgeError as e:
        print(e)


def callback_coordinput(coord):
    global intrinsics, latest_depth_img

    if latest_depth_img is None:
        rospy.logwarn("No pointcloud data recieved yet")
        return

    if intrinsics is None:
        rospy.logwarn("Camera intrinsics info not recieved yet")
        return

    # read depth at point
    pnt = ros_numpy.numpify(coord)
    depth = latest_depth_img[int(pnt[0]), int(pnt[1])]

    # convert depth from mm to metres
    depth = depth/1000

    # deproject to realworld coords in metres
    realworld_pnt = rs2.rs2_deproject_pixel_to_point(intrinsics, [pnt[0], pnt[1]], depth)

    # publish result
    pnt_msg = Point(x=realworld_pnt[0], y=realworld_pnt[1], z=realworld_pnt[2])
    coord_pub.publish(pnt_msg)


def callback_caminfo(cameraInfo):
    global intrinsics

    try:
        if intrinsics:
            return
        intrinsics = rs2.intrinsics()
        intrinsics.width = cameraInfo.width
        intrinsics.height = cameraInfo.height
        intrinsics.ppx = cameraInfo.K[2]
        intrinsics.ppy = cameraInfo.K[5]
        intrinsics.fx = cameraInfo.K[0]
        intrinsics.fy = cameraInfo.K[4]
        if cameraInfo.distortion_model == 'plumb_bob':
            intrinsics.model = rs2.distortion.brown_conrady
        elif cameraInfo.distortion_model == 'equidistant':
            intrinsics.model = rs2.distortion.kannala_brandt4
        intrinsics.coeffs = [i for i in cameraInfo.D]

    except CvBridgeError as e:
        print(e)

if __name__ == '__main__':
    rospy.init_node("cam_to_depthcoords", anonymous=True)
    rate = rospy.Rate(30)
    rospy.loginfo("Cam to Depth node started")

    rospy.Subscriber('/camera/depth/camera_info', CameraInfo, callback_caminfo)   
    rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, callback_depthimg)
    rospy.Subscriber("input_coords", Point, callback_coordinput)

    coord_pub = rospy.Publisher("realworld_coords", Point, queue_size=10)

    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()