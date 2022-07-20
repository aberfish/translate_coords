from time import sleep
import rospy
import tf2_ros as tf2
import tf
from geometry_msgs.msg import TransformStamped

#instantiate a StaticTransformBroadcaster object
br = tf2.StaticTransformBroadcaster()

#Function instantiates a TransformStamped object and broadcasts it. TransformStamped class has transform, header and child_frame_id fields. transform holds info about 
#rotation (quaternions) and translation (direction vector). header and child_frame_id hold info about the frames between which the transform describes.
#The TransformStamped object is broadcast using br.sendTransform.
def broadcast_tf(tf_position, tf_rotation, world_frame, cam_frame):
    static_transformStamped = TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()

    static_transformStamped.header.frame_id = world_frame #describe transform from this frame (the value of this is 'map', should we change name of var to map?)
    static_transformStamped.child_frame_id = cam_frame #describes tranform to this frame 

    static_transformStamped.transform.translation.x = float(tf_position[0])
    static_transformStamped.transform.translation.y = float(tf_position[1])
    static_transformStamped.transform.translation.z = float(tf_position[2])
    
    quat = tf.transformations.quaternion_from_euler(float(tf_rotation[0]), float(tf_rotation[1]), float(tf_rotation[1]))
    static_transformStamped.transform.rotation.x = quat[0]
    static_transformStamped.transform.rotation.y = quat[1]
    static_transformStamped.transform.rotation.z = quat[2]
    static_transformStamped.transform.rotation.w = quat[3]

    br.sendTransform(static_transformStamped)

#main function. Initialises cam_tf_broadcaster node. Initialises 4 variables with data from the launch file (via the parameter server).
#once the variables are initialised the broadcast_tf function is called. 
if __name__ == '__main__':
    rospy.init_node('cam_tf_broadcaster')

    cam_tf_pos = rospy.get_param('~tf_pos', default=(0.0, 0.0, 0.0))
    cam_tf_rot = rospy.get_param('~tf_rot', default=(0.0, 0.0, 0.0, 0.0))
    world_frame_name = rospy.get_param('~world_frame', default='map')
    cam_frame_name = rospy.get_param('~cam_frame', default="_link")

    # START RQT FIRST TO VIEW

    broadcast_tf(cam_tf_pos, cam_tf_rot, world_frame_name, cam_frame_name)

    rospy.loginfo(f"Camera transform has been broadcast: pos={cam_tf_pos} rot={cam_tf_rot}")
    rospy.spin()