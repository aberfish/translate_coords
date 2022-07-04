import rospy
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import pyrealsense2 as rs2

intrinsics = None
bridge = CvBridge()

def imageDepthCallback(data):
    global intrinsics
    try:
        cv_image = bridge.imgmsg_to_cv2(data, data.encoding)
        pix = (int(data.width/2), int(data.height/2)
        print('Depth at center(%d, %d): %f(mm)\r' % (pix[0], pix[1], cv_image[int(pix[1]), pix[0]]))

        if intrinsics:
            depth = cv_image[pix[1], pix[0]]
            result = rs2.rs2_deproject_pixel_to_point(intrinsics, [pix[0], pix[1]], depth)
            print('result:', result)

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

    sub = rospy.Subscriber("/camera/depth/image_rect_raw", msg_Image, imageDepthCallback)
    sub_info = rospy.Subscriber('/camera/depth/camera_info', CameraInfo, imageDepthInfoCallback)

    rospy.spin()