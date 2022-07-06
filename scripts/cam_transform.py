from time import sleep
import rospy
import tf

br = tf.TransformBroadcaster()

def broadcast_tf(tf_position, tf_rotation, world_frame, cam_frame):
    tf_rot_quat = tf.transformations.quaternion_from_euler(tf_rotation[0], tf_rotation[1], tf_rotation[2])

    br.sendTransform(tf_position,
                     tf_rot_quat,
                     rospy.Time.now(),
                     cam_frame,
                     world_frame)

if __name__ == '__main__':
    rospy.init_node('cam_tf_broadcaster')
    rate = rospy.Rate(0.1)
    cam_tf_pos = rospy.get_param('~tf_pos', default=(0.0, 0.0, 0.0))
    cam_tf_rot = rospy.get_param('~tf_rot', default=(0.0, 0.0, 0.0))
    world_frame_name = rospy.get_param('~world_frame')
    cam_frame_name = rospy.get_param('~cam_frame', default="_link")

    # START RQT FIRST TO VIEW

    broadcast_tf(cam_tf_pos, cam_tf_rot, world_frame_name, cam_frame_name)
    rospy.loginfo(f"Camera transform has been broadcast: pos={cam_tf_pos} rot={cam_tf_rot}")