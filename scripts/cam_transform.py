import rospy
import tf

br = tf.TransformBroadcaster()

def broadcast_tf(tf_position, tf_rotation, world_frame, cam_frame):
    br.sendTransform(tf_position,
                     tf_rotation,
                     rospy.Time.now(),
                     cam_frame,
                     world_frame)

if __name__ == '__main__':
    rospy.init_node('cam_tf_broadcaster')
    cam_tf_pos = rospy.get_param('~tf_pos', default=(0, 0, 0))
    cam_tf_rot = rospy.get_param('~tf_rot', default=tf.transformations.quaternion_from_euler(0, 0, 0))
    world_frame_name = rospy.get_param('~world_frame')
    cam_frame_name = rospy.get_param('~cam_fram', default="_link")

    broadcast_tf(cam_tf_pos, cam_tf_rot, world_frame_name, cam_frame_name)

    rospy.spin()