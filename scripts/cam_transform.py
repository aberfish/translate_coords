from time import sleep
import rospy
import tf2_ros as tf2
import tf
from geometry_msgs.msg import TransformStamped

br = tf2.StaticTransformBroadcaster()

def broadcast_tf(tf_position, tf_rotation, world_frame, cam_frame):
    static_transformStamped = TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = world_frame
    static_transformStamped.child_frame_id = cam_frame

    static_transformStamped.transform.translation.x = float(tf_position[0])
    static_transformStamped.transform.translation.y = float(tf_position[1])
    static_transformStamped.transform.translation.z = float(tf_position[2])

    #quat = tf.transformations.quaternion_from_euler(float(tf_rotation[0]), float(tf_rotation[1]), float(tf_rotation[2]))
    static_transformStamped.transform.rotation.x = float(tf_rotation[0])
    static_transformStamped.transform.rotation.y = float(tf_rotation[1])
    static_transformStamped.transform.rotation.z = float(tf_rotation[2])
    static_transformStamped.transform.rotation.w = float(tf_rotation[3])

    br.sendTransform(static_transformStamped)

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