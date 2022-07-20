"""PKG: translatecoords     NODE: cam_to_depth_coords.py
Converts 2d pixel coordinates from depth camera colour images to realworld 3d coordinates measured in metres, where 0,0,0 is ≈ (240, 320, 0)
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

latest_depth_img = None
intrinsics = None

COORDINATE_SCALING = 0.69 # applied to x and y coordinates after deprojection

# Function is called each time image data (a frame) is publsihed from the camera to the "/camera/aligned_depth_to_color/image_raw" topic.
# The function uses the CvBridge package to convert the image data from a ros image to an openCV image and assigns it to depth_image. 
# depth_image is converted to a np array of type float32 and assigned to the global latest_depth_img variable.
def callback_depthimg(depth_msg):
    global latest_depth_img

    bridge = CvBridge()
     # Use cv_bridge() to convert the ROS image to OpenCV format
    try:
        #Convert the depth image using the default passthrough encoding
        depth_image = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        #Convert the openCV image (which is a np array) to a np array of data type float
        latest_depth_img = np.array(depth_image, dtype=np.float32) 

    except CvBridgeError as e:
        print(e)

# function is called each time data is published to "input_coords" topic. Makes use of the data in the intrinsics and latest_depth_img variables provided 
# by the callback_caminfo and callback_depthimg functions (respectively). If there is no data in these variables then an error message is printed.
# The inputted coords from the "input_coords" topic is used to locate a point on the latest_depth_image and find the depth at that point. 
# A method in the rs2 package coverts the inputted coords to real world coords using the 'intrinics' as a parameter. It stores the full 3d real world 
# coords in the realworld_pnt variable. The individual real world x, y, z coords are published to the 'realworld_coords' topic.
def callback_coordinput(coord):
    global intrinsics, latest_depth_img

    if latest_depth_img is None:
        rospy.logwarn("No pointcloud data recieved yet")
        return

    if intrinsics is None:
        rospy.logwarn("Camera intrinsics info not recieved yet")
        return

    # converts ros Point message type (coord) to a numpy object
    pnt = ros_numpy.numpify(coord) 
    # locates the depth of the point in the latest_depth_img where the point is specified by the data from the input_coords topic.
    depth = latest_depth_img[int(pnt[1]), int(pnt[0])]

    # convert depth from mm to metres
    depth = depth/1000

    # deproject to realworld coords in metres
    realworld_pnt = rs2.rs2_deproject_pixel_to_point(intrinsics, [pnt[1], pnt[0]], depth)

    # BUG x and y axis of point needs scaling inorder to get it correct. unkown why
    realworld_pnt[0] *= COORDINATE_SCALING
    realworld_pnt[1] *= COORDINATE_SCALING

    # publish result
    pnt_msg = Point(x=realworld_pnt[0], y=realworld_pnt[1], z=realworld_pnt[2])
    coord_pub.publish(pnt_msg)

# Each time data is published to the '/camera/depth/camera_info' topic this function is called
# and the published data is passed as a parameter. The data is of CameraInfo type and contains info about the camera.
# The intrinsic camera parameters are assigned to the global 'intrinsics' variable.  
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

# main function that initialises the node, declares the topics the node publishes to, 
# decalres topics the node subscribes to and which callback functions the data is sent to. 
if __name__ == '__main__':
    rospy.init_node("cam_to_depthcoords", anonymous=True)
    rate = rospy.Rate(30)
    rospy.loginfo("Cam to Depth node started")

    rospy.Subscriber('/camera/depth/camera_info', CameraInfo, callback_caminfo) 
    rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, callback_depthimg)
    rospy.Subscriber("input_coords", Point, callback_coordinput)

    coord_pub = rospy.Publisher("depth_coords", Point, queue_size=10)

    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()