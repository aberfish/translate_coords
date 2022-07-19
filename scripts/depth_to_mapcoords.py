from email import header
from py_compile import main
from time import time
import rospy
import tf2_ros
from geometry_msgs.msg import Point
from tf2_geometry_msgs import PointStamped

tfbuffer = tf2_ros.Buffer()
tflistener = tf2_ros.TransformListener(tfbuffer)

def callback_coordinput(coord):
    #convert point to stamped point in camera frame. This information is required to calculate the transform.
    pnt = PointStamped()
    pnt.header.stamp = rospy.Time()
    pnt.header.frame_id = cam_frame_name
    pnt.point = coord

    try:
        #returns the transform between the two frames?
        # tf = tfbuffer.lookup_transform(cam_frame_name, world_frame_name, rospy.Time(), tftimeout)
        
        #this returns the transformed point rather than the transform (above commented). Returns the point in the map frame. 
        resultcoord = tfbuffer.transform(pnt, world_frame_name, timeout=rospy.Duration(tftimeout))
    
    #If the point cannot be transformed (the target frame is not available in 1 second) an error is thrown.
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logwarn('Failed to transform point from camera to world frame \n', e)
        return

    #publish transformed coord without timestamp
    coord_pub.publish(resultcoord.point)

#main function. 
if __name__ == "__main__":

    #Initialises depth_to_mapcoords node.
    rospy.init_node('depth_to_mapcoords', anonymous=True)
    rate = rospy.Rate(30)

    #Initialises the variables with values from the launchfile (via parameter server)
    world_frame_name = rospy.get_param('~world_frame', default='map')
    cam_frame_name = rospy.get_param('~cam_frame', default="_link")
    tftimeout = rospy.get_param('~tf_timeout', default=1.0) # in secs

    rospy.loginfo("Depth to Map node started")

    #subscribes to depth_coords topic, when data is published to this topic the callback_coordinput function is called.
    rospy.Subscriber("depth_coords", Point, callback_coordinput)

    #rospy.Publisher object initiated
    coord_pub = rospy.Publisher("world_coords", PointStamped, queue_size=10)

    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()