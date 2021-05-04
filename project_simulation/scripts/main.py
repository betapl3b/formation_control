#!/usr/bin/python
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PoseStamped
import tf
import threading
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
import angles

goHome = 0
targetPose = PoseStamped()
targetPose.pose.orientation.w = 1
targetPose.pose.position.x = 1.66
targetPose.pose.position.z = -1


def alvarCallback(marker):
    global targetPose
    global targetMarker
    if int(targetMarker) == 1:
        targetPose.header.frame_id = 'ar_marker_1'
    elif int(targetMarker) == 2:
        targetPose.header.frame_id = 'ar_marker_4'
    elif int(targetMarker) == 3:
        targetPose.header.frame_id = 'ar_marker_2'
    elif int(targetMarker) == 4:
        targetPose.header.frame_id = 'ar_marker_0'
    elif int(targetMarker) == 0:
        print('Confirm that nobody needs my help by clapping')
        targetPose.pose.position.x = 5
        targetPose.pose.position.y = 4
        targetPose.pose.position.z = 0
        targetPose.header.frame_id = 'world'
        global goHome
        ar_tracker.unregister()
        goHome = 1
    else:
        print('Wrong number')
        targetMarker = int(raw_input('Which table needs my help? (1-4) '))


if __name__ == "__main__":
    rospy.init_node('controller_node')
    while goHome == 0:
        targetMarker = int(raw_input('Which table needs my help? (1-4) '))
        coords = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        ar_tracker = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, alvarCallback, queue_size=10)
        coords.publish(targetPose)
    print('end')

    rospy.spin()
