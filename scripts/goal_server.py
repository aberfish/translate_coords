import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PointStamped, Quaternion
from tf.transformations import quaternion_from_euler

client = None
frame_name = ""

def active_callback():
    rospy.loginfo("Goal pose is now being processed by the Action Server...")

def feedback_callback(feedback):
    #rospy.loginfo("Feedback for goal pose received")
    # feedback contains current position info
    pass

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
    if type(goal_pnt) == PointStamped:
        goal_pnt = goal_pnt.point

    qt = quaternion_from_euler(0, 0, 0)
    goal_pose = Pose(goal_pnt, Quaternion(*qt))

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = frame_name
    goal.target_pose.header.stamp = rospy.Time.now() 
    goal.target_pose.pose = goal_pose
    #rospy.loginfo('New goal is [', str(goal_pnt[0]), str(goal_pnt[1]), str(goal_pnt[2]), ']')
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

    rospy.loginfo("Connected to action server")

    # subscribe to goal coords topic
    rospy.Subscriber("goal_coords", PointStamped, send_goal)

    rospy.spin()