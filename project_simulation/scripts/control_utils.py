#!/usr/bin/python3
import control

import rospy
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
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
        return limit_value(self.prev_out, 1000000000)

    def get_state(self):
        return self.prev_out


class Compensator(object):
    def __init__(self, lambda0, sigma0, e, init_time, name=None, debug=False, parent=None):
        self.name = name
        self.parent = parent
        self.xi_int = Integrator(0, init_time, parent=self, name='xi')
        self.k_int = Integrator(0, init_time, parent=self, name='k')

        self.k_publisher = rospy.Publisher(f'/{parent.name}/{name}_k', Float64, queue_size=2)
        self.xi_publisher = rospy.Publisher(f'/{parent.name}/{name}_xi', Float64, queue_size=2)

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
        self.err_pubs = []
        for i in range(len(laplacian)):
            if i != 0 or True:
                self.err_pubs.append((rospy.Publisher(f'/{self.name}/{i}_x_err', Float64, queue_size=2),
                                      rospy.Publisher(f'/{self.name}/{i}_y_err', Float64, queue_size=2)))
            else:
                self.err_pubs.append((None, None))

        self.subscriber = rospy.Subscriber('/{}/odom'.format(self.name), Odometry, self.update_state_odom, queue_size=1)
        self.controlled = controlled
        self.speed_integrator_x = RobotModel(0, rospy.get_time(), parent=self, name='x_vel')
        self.speed_integrator_y = RobotModel(0, rospy.get_time(), parent=self, name='y_vel')

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

    def get_complex_error(self, neighbour_index):
        npos = rospy.wait_for_message(f'/robot{neighbour_index}/odom', Odometry)
        npos_arr = arr([npos.pose.pose.position.x, npos.pose.pose.position.y])
        pos_arr = arr([self.position_x, self.position_y])
        cur_dis = linalg.norm(pos_arr - npos_arr)
        err = (cur_dis ** 2 - self.distances[neighbour_index] ** 2) * (npos_arr - pos_arr)
        self.err_pubs[neighbour_index][0].publish(err[0])
        self.err_pubs[neighbour_index][1].publish(err[1])
        if self.debug:
            if err > 0.05:
                print(self.name, ' error to ', neighbour_index, 'is : ', err)

        return err

    def set_vel(self, velocity):
        if self.debug:
            print(self.name, ' - setting velocity to: ', velocity.linear.x, '; ', velocity.linear.y)
        if self.controlled and self.robot_id != 0:
            # velocity.linear.x = limit_value(velocity.linear.x, 0.5)
            # velocity.linear.y = limit_value(velocity.linear.y, 0.5)
            velocity.linear.x = self.speed_integrator_x.update_state(
                velocity.linear.x, rospy.get_time())
            velocity.linear.y = self.speed_integrator_y.update_state(
                velocity.linear.y, rospy.get_time())
            self.publisher.publish(velocity)


def limit_value(value, max_value=1.0):
    if abs(value) > max_value:
        value = max_value if value > 0 else -max_value
    return value


class RobotModel(object):
    def __init__(self, init_cond, init_time, parent=None, name=None):
        self.name = name
        self.parent = parent
        self.prev_out = init_cond
        self.prev_time = init_time
        self.tf = control.tf([1], [0.1, 1])
        self.history = []
        # if parent:
        # print(parent.parent.name, ' Integrator ', self.name, ' for ', parent.name, ' created!')

    def update_state(self, inp, t):
        time, out = control.forced_response(self.tf, U=inp, T=[self.prev_time, t], X0=self.prev_out)
        self.prev_time = time[-1]
        self.prev_out = out[-1]
        self.history.append(self.prev_out)
        return limit_value(self.prev_out, 1.0)

    def get_state(self):
        return self.prev_out
