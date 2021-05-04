#!/usr/bin/python3
import rospy
import control_utils
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist
import sys

timestamp = 0
NUMBER = int(sys.argv[1])
DISTANCES = list(map(float, str(sys.argv[2]).strip('][').split(', ')))
LAMBDA0 = float(sys.argv[3])
SIGMA0 = float(sys.argv[4])
E = float(sys.argv[5])
LOG_COMPENSATOR = bool(sys.argv[6])
rospy.loginfo(f'Params: {NUMBER}, {DISTANCES}, {LAMBDA0}, {SIGMA0}, {E}, {LOG_COMPENSATOR}')


def time_callback(clock_response):
    global timestamp
    timestamp = float(clock_response.clock.secs) + float(clock_response.clock.nsecs) * 0.000000001
    if timestamp % 1 == 0:
        print(timestamp)


def node_creator(robot):
    global timestamp
    timestamp = rospy.get_time()
    compensators_x = []
    compensators_y = []
    velocity = Twist()
    neighbours = {}
    for index in [i for i, d in enumerate(DISTANCES) if d != 0]:

        compensator_x = control_utils.Compensator(LAMBDA0, SIGMA0, E, timestamp, parent=robot,
                                                  name=f'{NUMBER} to {index} X compensator')
        compensator_y = control_utils.Compensator(LAMBDA0, SIGMA0, E, timestamp, parent=robot,
                                                  name=f'{NUMBER} to {index} Y compensator')

        neighbours.update({index: (compensator_x, compensator_y)})

    while True:
        timestamp = rospy.get_time()
        control_x = 0
        control_y = 0
        for index, comp_tuple in iter(neighbours.items()):
            errors_i = robot.get_neighbour_error(index)
            control_x += comp_tuple[0].update_state(errors_i[0],
                                                    timestamp)  # * (neighbour.position_x - robot.position_x)
            control_y += comp_tuple[1].update_state(errors_i[1],
                                                    timestamp)  # * (neighbour.position_y - robot.position_y)
        velocity.linear.x = control_utils.limit_robot_speed(-control_x)
        velocity.linear.y = control_utils.limit_robot_speed(-control_y)
        robot.set_vel(velocity)
        rospy.sleep(0.05)


# diag = math.sqrt(2)
# rospy.Subscriber('/clock', Clock, time_callback, queue_size=5)

if __name__ == "__main__":
    rospy.init_node(f'robot_{NUMBER}_controller')
    robot = control_utils.Robot(NUMBER, DISTANCES, only_neigbours=True, controlled=LOG_COMPENSATOR)
    node_creator(robot)
    rospy.spin()
