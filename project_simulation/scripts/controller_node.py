#!/usr/bin/python3
import rospy
import control_utils
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist
import numpy as np
import math
import threading

timestamp = 0
LAMBDA0 = 0.8
SIGMA0 = 0.1
E = 0.6
LOG_COMPENSATOR = False


def time_callback(clock_response):
    global timestamp
    timestamp = float(clock_response.clock.secs) + float(clock_response.clock.nsecs) * 0.000000001
    if timestamp % 1 == 0:
        print(timestamp)


def node_creator(robot):
    global timestamp
    if '3' in robot.name:
        robot.debug = True
    if '0' in robot.name:
        robot.controlled = False
    # robot.debug = True
    # rospy.init_node('{}_controller'.format(robot.name))
    compensators_x = []
    compensators_y = []
    velocity = Twist()
    for index, neighbour in enumerate(iter(robot.neighbours_dict.values())):
        compensators_x.append(
            control_utils.Compensator(LAMBDA0, SIGMA0, E, timestamp,
                                      name='{} to {} X compensator'.format(robot.name, neighbour.name), parent=robot))
        compensators_y.append(
            control_utils.Compensator(LAMBDA0, SIGMA0, E, timestamp,
                                      name='{} to {} Y compensator'.format(robot.name, neighbour.name), parent=robot))

    while True:
        control_x = 0
        control_y = 0
        for index, neighbour in enumerate(iter(robot.neighbours_dict.values())):
            errors_i = robot.get_error(neighbour)
            control_x += compensators_x[index].update_state(errors_i[0], timestamp)# * (neighbour.position_x - robot.position_x)
            control_y += compensators_y[index].update_state(errors_i[1], timestamp)# * (neighbour.position_y - robot.position_y)
        velocity.linear.x = control_utils.limit_robot_speed(-control_x)
        velocity.linear.y = control_utils.limit_robot_speed(-control_y)
        robot.set_vel(velocity)
        rospy.sleep(0.05)


def controller(robots):
    global timestamp
    # rospy.init_node('{}_controller'.format(robot.name))
    pubs = []
    comp_x = []
    comp_y = []
    vels = []
    robot_list = []
    for robot in iter(robots.values()):
        robot_list.append(robot)
        pubs.append(rospy.Publisher('/{}/cmd_vel'.format(robot.name), Twist, queue_size=5))
        comp_x.append(control_utils.Compensator(lambda0=LAMBDA0, sigma0=SIGMA0, e=E,
                                                init_time=timestamp, name='X', parent=robot))
        comp_y.append(control_utils.Compensator(lambda0=LAMBDA0, sigma0=SIGMA0, e=E,
                                                init_time=timestamp, name='Y', parent=robot))
        vels.append(Twist())

    while True:
        for i in range(len(robot_list)):
            error_x, error_y = robot_list[i].get_errors()
            print('{} errs: '.format(robot_list[i].name), error_x, error_y)
            vels[i].linear.x = control_utils.limit_robot_speed(comp_x[i].update_state(error_x, timestamp))
            print('{} speedx '.format(robot_list[i].name), vels[i].linear.x)
            vels[i].linear.y = control_utils.limit_robot_speed(comp_y[i].update_state(error_y, timestamp))
            print('{} speedy '.format(robot_list[i].name), vels[i].linear.y)
            pubs[i].publish(vels[i])
        rospy.sleep(0.01)


diag = math.sqrt(2)
rospy.init_node('main_controller')
formation = np.array([
    [0, 2, 2],
    [2, 0, 2],
    [2, 2, 0],
])
# formation = np.array([
#     [0, 2],
#     [2, 0],
# ])
# zero = [0, 0]
# full = [1, 1]
# only_x = [1, 0]
# only_y = [0, 1]
# formation = np.array([
#     [zero, only_x, full, only_y],
#     [only_x, zero, only_y, zero],
#     [full, only_y, zero, only_x],
#     [only_y, zero, only_x, zero]
# ])
# formation = np.array([
#     [zero, only_x, full, only_y],
#     [only_x, zero, only_y, zero],
#     [full, zero, zero, only_x],
#     [only_y, zero, zero, zero]
# ])
# formation = formation*2
# print(formation)
N = formation.shape[0]
robots = {'robot{}'.format(bot): control_utils.Robot(bot, formation) for bot in range(N)}
for robot in iter(robots.values()):
    robot.link_neighbours(robots)
rospy.Subscriber('/clock', Clock, time_callback, queue_size=5)

# controller(robots)
for robot in iter(robots.values()):
    print('starting thread with ', robot.name)
    x = threading.Thread(target=node_creator, args=(robot,))
    x.start()
    rospy.sleep(0.1)

rospy.spin()
