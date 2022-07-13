import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler

client = None
frame_name = ""

def active_callback():
    rospy.loginfo("Goal pose is now being processed by the Action Server...")

def feedback_callback(feedback):
    rospy.loginfo("Feedback for goal pose received")
    # feedback contains current position info

def done_callback(status, result):
    # status contains status code
    # result is always empty
    if status == 2:
        rospy.loginfo("Goal pose received a cancel request after it started executing, completed execution!")

    if status == 3:
        rospy.loginfo("Goal pose reached") 

    if status == 4:
        rospy.loginfo("Goal pose was aborted by the Action Server")
        rospy.signal_shutdown("Goal pose aborted, shutting down!")
        return

    if status == 5:
        rospy.loginfo("Goal pose has been rejected by the Action Server")
        rospy.signal_shutdown("Goal pose rejected, shutting down!")
        return

    if status == 8:
        rospy.loginfo("Goal pose received a cancel request before it started executing, successfully cancelled!")


def send_goal(goal_pnt):

    qt = quaternion_from_euler(0, 0, 0)
    goal_pose = Pose(goal_pnt, Quaternion(*qt))

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = frame_name
    goal.target_pose.header.stamp = rospy.Time.now() 
    goal.target_pose.pose = goal_pose
    rospy.loginfo(str(goal_pose))
    client.send_goal(goal, done_callback, active_callback, feedback_callback)

if __name__ == '__main__':
    rospy.init_node("goal_server")
    frame_name = rospy.get_param('~map_frame', default="map")
    rospy.loginfo("Goal Server node started")

    # connect to movebase action server for sending goals
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    wait = client.wait_for_server(rospy.Duration(5.0))
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
        quit()

    rospy.loginfo("Connected to move base server")

    # subscribe to goal coords topic
    rospy.Subscriber("realworld_coords", Point, send_goal)

    rospy.spin()

#   EXAMPLE POSE COORD FRAME CONVERSION
# tfBuffer = tf2_ros.Buffer()
# listener = tf2_ros.TransformListener(tfBuffer)
# def convert_pose(pose, from_frame, to_frame):
#     """
#     Convert a pose or transform between frames using tf.
#         pose            -> A geometry_msgs.msg/Pose that defines the robots position and orientation in a reference_frame
#         from_frame      -> A string that defines the original reference_frame of the robot
#         to_frame        -> A string that defines the desired reference_frame of the robot to convert to
#     """
#     global tfBuffer, listener

#     # Create Listener objet to recieve and buffer transformations
#     trans = None

#     try:
#         trans = tfBuffer.lookup_transform(to_frame, from_frame, rospy.Time(0), rospy.Duration(1.0))
#     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException), e:
#         print(e)
#         rospy.logerr('FAILED TO GET TRANSFORM FROM %s to %s' % (to_frame, from_frame))
#         return None

#     spose = gmsg.PoseStamped()
#     spose.pose = pose
#     spose.header.stamp = rospy.Time().now
#     spose.header.frame_id = from_frame

#     p2 = tf2_geometry_msgs.do_transform_pose(spose, trans)

#     return p2.pose