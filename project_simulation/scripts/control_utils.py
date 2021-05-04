#!/usr/bin/python3
import control

import rospy
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from numpy import array as arr, linalg
import math

# ROS = False
ROS = True
LOG_COMP = False


class Integrator(object):
    def __init__(self, init_cond, init_time, parent=None, name=None):
        self.name = name
        self.parent = parent
        self.prev_out = init_cond
        self.prev_time = init_time
        self.tf = control.tf([1], [1, 0])
        self.history = []
        # if parent:
            # print(parent.parent.name, ' Integrator ', self.name, ' for ', parent.name, ' created!')

    def update_state(self, inp, t):
        time, out = control.forced_response(self.tf, U=inp, T=[self.prev_time, t], X0=self.prev_out)
        self.prev_time = time[-1]
        self.prev_out = out[-1]
        self.history.append(self.prev_out)
        return self.prev_out

    def get_state(self):
        return self.prev_out


class Compensator(object):
    def __init__(self, lambda0, sigma0, e, init_time, name=None, debug=False, parent=None):
        self.name = name
        self.parent = parent
        self.xi_int = Integrator(0, init_time, parent=self, name='xi')
        self.k_int = Integrator(0, init_time, parent=self, name='k')
        self.lambda0 = lambda0
        self.sigma0 = sigma0
        self.e = e
        self.prev_out = 0
        self.history = []
        self.k_history = []
        self.e_history = []
        self.timeseries = []
        self.debug = parent.debug and LOG_COMP if parent.debug else debug

    def update_state(self, inp, t):
        dk = self.lambda0 if abs(inp) >= self.e else 0
        k = self.k_int.update_state(dk, t)
        dxi = k * k * self.sigma0 * (inp - self.xi_int.prev_out)
        xi = self.xi_int.update_state(dxi, t)
        self.prev_out = -(k * xi) - ((dxi * k) + (dk * xi))

        if self.debug:
            self.timeseries.append(t)
            self.history.append(self.prev_out)
            if dk != 0:
                print(self.name, ' k: ', k)
                print(self.name, ' dxi: ', dxi)
            self.k_history.append(k)
            self.e_history.append(inp)
        return self.prev_out

    def get_state(self):
        return self.prev_out


class Robot(object):
    def __init__(self, robot_id, laplacian=None, debug=False, controlled=True, only_neigbours=False):

        # Name
        self.robot_id = robot_id
        self.name = 'robot{}'.format(self.robot_id)
        self.neighbours_dict = {}
        self.debug = debug
        self.publisher = rospy.Publisher('/{}/cmd_vel'.format(self.name), Twist, queue_size=2)
        self.position_x = 0
        self.position_y = 0
        self.orientation = 0

        self.subscriber = rospy.Subscriber('/{}/odom'.format(self.name), Odometry, self.update_state_odom, queue_size=1)
        self.controlled = controlled
        self.speed_integrator_x = Integrator(0, rospy.get_time(), parent=self, name='x_vel')
        self.speed_integrator_y = Integrator(0, rospy.get_time(), parent=self, name='y_vel')

        # Service
        if not only_neigbours:
            self.distances = laplacian[robot_id] if laplacian is not None else None
        else:
            self.distances = laplacian

    def update_state_odom(self, response):
        self.position_x = response.pose.pose.position.x
        self.position_y = response.pose.pose.position.y
        self.orientation = response.pose.pose.orientation.z
        if self.debug:
            # print(self.name, ' position: ', self.position_x, '; ', self.position_y)
            pass

    def print_goal(self):
        if self.distances is None:
            print('{} has no goal'.format(self.name))
        else:
            for index, distance in enumerate(self.distances):
                if distance != 0:
                    print('{} tries to keep distance {} with robot{}'.format(self.name, distance, index))

    def link_neighbours(self, robots_dict):
        for index, distance in enumerate(self.distances):
            if distance != 0:
                name = 'robot{}'.format(index)
                self.neighbours_dict[name] = robots_dict[name]

    def get_error(self, neighbour):
        X = ([0, self.position_x]) - arr([0, neighbour.position_x])
        Y = ([0, self.position_y]) - arr([0, neighbour.position_y])
        err_x = (linalg.norm(X) ** 2 - self.distances[neighbour.robot_id] ** 2) * (neighbour.position_x - self.position_x)
        err_y = (linalg.norm(Y) ** 2 - self.distances[neighbour.robot_id] ** 2) * (neighbour.position_y - self.position_y)
        if self.debug:
            if err_x > 0.05 or err_y > 0.05:
                print(self.name, ' errors to ', neighbour.name, 'is : ', err_x, '; ', err_y)

        return err_x, err_y

    def get_neighbour_error(self, neighbour_index):
        npos = rospy.wait_for_message(f'/robot{neighbour_index}/odom', Odometry)
        npos_x = npos.pose.pose.position.x
        npos_y = npos.pose.pose.position.y

        X = ([0, self.position_x]) - arr([0, npos_x])
        Y = ([0, self.position_y]) - arr([0, npos_y])
        err_x = (linalg.norm(X) ** 2 - self.distances[neighbour_index] ** 2) * (npos_x - self.position_x)
        err_y = (linalg.norm(Y) ** 2 - self.distances[neighbour_index] ** 2) * (npos_y - self.position_y)
        if self.debug:
            if err_x > 0.05 or err_y > 0.05:
                print(self.name, ' errors to ', neighbour_index, 'is : ', err_x, '; ', err_y)

        return err_x, err_y

    def get_complex_error(self, neighbour_index):
        npos = rospy.wait_for_message(f'/robot{neighbour_index}/odom', Odometry)
        npos_arr = arr([npos.pose.pose.position.x, npos.pose.pose.position.y])
        pos_arr = arr([self.position_x, self.position_y])
        cur_dis = linalg.norm(pos_arr - npos_arr)
        err = cur_dis - self.distances[neighbour_index]
        if self.debug:
            if err > 0.05:
                print(self.name, ' error to ', neighbour_index, 'is : ', err)

        return err

    def set_vel(self, velocity):
        if self.debug:
            print(self.name, ' - setting velocity to: ', velocity.linear.x, '; ', velocity.linear.y)
        if self.controlled and self.robot_id !=0:
            # velocity.linear.x = self.speed_integrator_x.update_state(velocity.linear.x * math.cos(self.orientation), rospy.get_time())
            # velocity.linear.y = self.speed_integrator_y.update_state(velocity.linear.y * math.sin(self.orientation), rospy.get_time())
            # velocity.linear.x *= math.sin(self.orientation)
            # velocity.linear.y *= math.cos(self.orientation)
            self.publisher.publish(velocity)


def limit_robot_speed(speed, max_speed=1.0):
    if abs(speed) > max_speed:
        speed = max_speed if speed > 0 else -max_speed
    return speed
