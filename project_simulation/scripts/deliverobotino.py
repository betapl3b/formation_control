#!/usr/bin/python
import rospy
from geometry_msgs.msg import PoseStamped
def goHome():
    targetPose.pose.position.x = 5
    targetPose.pose.position.y = 4
    targetPose.pose.position.z = 0
    targetPose.header.frame_id = 'world'
    coords.publish(targetPose)
def deliverobotino():
    global tasks
    while tasks < 5:
        targetMarker = int(raw_input('Who needs my help now? (1-4; 0 - nobody)\n'))
        targetPose.pose.orientation.w = 0.5
        targetPose.pose.orientation.z = 0.5
        targetPose.pose.position.x = 1.66
        targetPose.pose.position.z = -1
        if int(targetMarker) == 1:
            targetPose.header.frame_id = 'ar_marker_1'
            tasks+=1
        elif int(targetMarker) == 2:
            targetPose.header.frame_id = 'ar_marker_2'
            targetPose.pose.position.x = -1
            targetPose.pose.position.z = -1
            tasks+=1
        elif int(targetMarker) == 3:
            targetPose.header.frame_id = 'ar_marker_4'
            targetPose.pose.position.x = -1
            targetPose.pose.position.z = -1
            tasks+=1
        elif int(targetMarker) == 4:
            targetPose.header.frame_id = 'ar_marker_0'
            tasks+=1
        elif int(targetMarker) == 0:
            print('Confirm that nobody needs my help by clapping. \n')
            clapTest = raw_input('Press any key to continue... \n')
            tasks = 1337
            goHome()
            continue
        else:
            print('Wrong number')
            continue
        coords.publish(targetPose)

rospy.init_node('controller_node')
coords = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
oneMore='test'
while not oneMore == 'quit':
    tasks = 0
    targetPose = PoseStamped()
    deliverobotino()
    print('Type anything if u need my help and "quit" if u do not \n')
    oneMore = raw_input('...')
