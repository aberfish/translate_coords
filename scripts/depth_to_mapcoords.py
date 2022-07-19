from email import header
from py_compile import main
from time import time
import rospy
import tf2_ros
from geometry_msgs.msg import Point
from tf2_geometry_msgs import PointStamped

tfbuffer = None
tflistener = None

def callback_coordinput(coord):
    # convert point to stamped point in camera frame
    pnt = PointStamped()
    pnt.header.stamp = rospy.Time.now() 
    pnt.header.frame_id = cam_frame_name
    pnt.point = coord

    try:
        # tf = tfbuffer.lookup_transform(cam_frame_name, world_frame_name, rospy.Time(), tftimeout)

        resultcoord = tfbuffer.transform(pnt, world_frame_name, timeout=rospy.Duration(tftimeout))

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logwarn('Failed to transform point from camera to world frame \n', e)
        # return

    # publish transformed coord without timestamp
    coord_pub.publish(resultcoord)

if __name__ == "__main__":
    rospy.init_node('depth_to_mapcoords', anonymous=True)
    rate = rospy.Rate(30)

    world_frame_name = rospy.get_param('~world_frame', default='map')
    cam_frame_name = rospy.get_param('~cam_frame', default="_link")
    tftimeout = rospy.get_param('~tf_timeout', default=1.0) # in secs

    tfbuffer = tf2_ros.Buffer()
    tflistener = tf2_ros.TransformListener(tfbuffer)

    rospy.loginfo("Depth to Map node started")


    rospy.Subscriber("depth_coords", Point, callback_coordinput)

    coord_pub = rospy.Publisher("realworld_coords", PointStamped, queue_size=10)

    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()