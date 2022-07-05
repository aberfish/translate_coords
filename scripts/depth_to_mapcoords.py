import rospy
from sensor_msgs.msg import Image as msg_Image
from geometry_msgs.msg import Point
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError

import pyrealsense2 as rs2

intrinsics = None
latest_depth_image = None
bridge = CvBridge()

def depthPointCallback(pnt):
    global latest_depth_image, intrinsics

    if intrinsics:
        depth = pnt[2]

        # convert depth from mm to metres
        depth = depth/1000

        # deproject to realworld coords in metres
        result = rs2.rs2_deproject_pixel_to_point(intrinsics, [pnt[0], pnt[1]], depth)

        # publish result
        pnt_msg = Point(x=result[0], y=result[1], z=result[2])
        depth_coords_realworld_pub.publish(pnt_msg)

    else:
        rospy.logwarn("Camera intrinsics not received yet")

def imageDepthCallback(data):
    global latest_depth_image

    try:
        cv_image = bridge.imgmsg_to_cv2(data, data.encoding)
        latest_depth_image = cv_image

    except CvBridgeError as e:
        print(e)
        return

def imageDepthInfoCallback(cameraInfo):
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
        return


if __name__ == '__main__':
    rospy.init_node("depth_to_mapcoords")

    depth_img_sub = rospy.Subscriber("/camera/depth/image_rect_raw", msg_Image, imageDepthCallback)
    cam_info_sub = rospy.Subscriber('/camera/depth/camera_info', CameraInfo, imageDepthInfoCallback)
    input_coords_sub = rospy.Subscriber('pc_coords', Point, depthPointCallback)

    depth_coords_realworld_pub = rospy.Publisher('realworld_depth_coords', Point, queue_size=10)

    rospy.spin()
