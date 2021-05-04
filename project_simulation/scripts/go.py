#!/usr/bin/python
import rospy
from geometry_msgs.msg import PoseStamped
from audio_common_msgs.msg import AudioData
def goHome():
    global targetPose
    targetPose.pose.orientation.w = 0
    targetPose.pose.orientation.z = 1
    targetPose.pose.position.z = 0
    targetPose.pose.position.x = 4
    targetPose.pose.position.y = 5
    targetPose.header.frame_id = 'map'
    coords.publish(targetPose)

def audioCaptureCallback(audioArray):
    global clapTest
    clapTest = len(audioArray.data)

def deliverobotino():
    global clapTest
    clapTest = 0
    goNext = 0
    while goNext == 0:
        global targetMarker
        targetMarker = int(raw_input('Who needs my help now? (1-4)\n'))
        targetPose.pose.orientation.w = 0.5
        targetPose.pose.orientation.z = 0.5
        targetPose.pose.position.x = 2
        targetPose.pose.position.z = -1
        targetPose.pose.position.y = -0.86
        if int(targetMarker) == 1:
            targetPose.header.frame_id = 'ar_marker_1'
            goNext+=1
        elif int(targetMarker) == 2:
            targetPose.header.frame_id = 'ar_marker_2'
            targetPose.pose.position.x = -2
            targetPose.pose.position.z = -1
            goNext += 1
        elif int(targetMarker) == 3:
            targetPose.header.frame_id = 'ar_marker_4'
            targetPose.pose.position.x = -2
            targetPose.pose.position.z = -1
            goNext += 1
        elif int(targetMarker) == 4:
            targetPose.header.frame_id = 'ar_marker_0'
            goNext += 1
        else:
            print('Wrong number')
    coords.publish(targetPose)
    print('Clap when I can go. \n')
    while not clapTest > 500:
        rospy.loginfo('Microphone volume= %d (Clap for 500 VOLUME to go home)', clapTest)
    rospy.logwarn('I heard the clap! Going home now.')
    goHome()

rospy.init_node('controller_node')
rospy.Subscriber('/audio/audio', AudioData, audioCaptureCallback, queue_size=10)
coords = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
oneMore='test'
while not oneMore == 'quit':
    targetPose = PoseStamped()
    deliverobotino()
    print('Type anything if u need my help and "quit" if u do not \n')
    oneMore = raw_input('...')
