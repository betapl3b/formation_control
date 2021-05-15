#!/usr/bin/python3
import logging

import control

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from numpy import array as arr, linalg
import math

from transformations import euler_from_quaternion

LOG_PATH = '/home/beta/catkin_ws/src/project_simulation/logs/'


class Integrator(object):
    def __init__(self, init_cond, init_time, parent=None, name=None):
        self.name = name
        self.parent = parent
        self.prev_out = init_cond
        self.prev_time = init_time
        self.tf = control.tf([1], [1, 0])

    def update_state(self, inp, t):
        time, out = control.forced_response(self.tf, U=inp, T=[self.prev_time, t], X0=self.prev_out)
        self.prev_time = time[-1]
        self.prev_out = out[-1]
        return limit_value(self.prev_out, 10000000)

    def get_state(self):
        return self.prev_out


class Compensator(object):
    def __init__(self, lambda0, sigma0, e, init_time, name=None, parent=None):
        self.name = name
        self.parent = parent
        self.xi_int = Integrator(0, init_time, parent=self, name='xi')
        self.k_int = Integrator(0, init_time, parent=self, name='k')

        self.lambda0 = lambda0
        self.sigma0 = sigma0
        self.e = e
        self.prev_out = 0
        self.logger = setup_logger(self.name, f'{LOG_PATH}{self.name}.csv')
        self.logger.debug('time,dk,k,dxi,xi,out')

    def update_state(self, inp, t):
        dk = self.lambda0 if abs(inp) >= self.e else 0
        k = self.k_int.update_state(dk, t)
        dxi = k * k * self.sigma0 * (inp - self.xi_int.prev_out)
        xi = self.xi_int.update_state(dxi, t)
        self.prev_out = -(k * xi) - ((dxi * k) + (dk * xi))
        self.logger.debug(f'{t},{dk},{k},{dxi},{xi},{self.prev_out}')

        return self.prev_out

    def get_state(self):
        return self.prev_out


class Robot(object):
    def __init__(self, robot_id, laplacian=None, ext=False):
        """Robot object
        laplacian: list - Part of Laplacian. Row for specified robot ID
        ext: bool - External indignation in model
        robot_id: int - ID
        """
        # Robot's ID
        self.robot_id = robot_id
        # Robot's name
        self.name = 'robot{}'.format(self.robot_id)
        # External indignation True/False
        self.ext = ext
        # Velocity publisher
        self.publisher = rospy.Publisher('/{}/cmd_vel'.format(self.name), Twist, queue_size=2)
        # Robot's position
        self.position_x = 0
        self.position_y = 0
        self.orientation = 0
        # Postition logging
        self.pos_logger = setup_logger(f'{self.name}_pos', f'{LOG_PATH}{self.name}_pos.csv')
        self.pos_logger.debug('time,pos_x,pos_y')
        # Velocity logging
        self.vel_logger = setup_logger(f'{self.name}_vel', f'{LOG_PATH}{self.name}_vel.csv')
        self.vel_logger.debug('time,vel_x,vel_y')
        # Ext logging
        if self.ext:
            self.ext_logger = setup_logger(f'{self.name}_ext', f'{LOG_PATH}{self.name}_ext.csv')
            self.ext_logger.debug('time,ext')

        # Position subscriber
        self.subscriber = rospy.Subscriber('/{}/odom'.format(self.name), Odometry, self.update_state_odom, queue_size=1)

        # Robot's model declaration
        self.speed_model_x = None
        self.speed_model_y = None

        # Logging and neighbour position dictionaries
        self.logger_dict = {}
        self.sub_dict = {}
        self.npos_dict = {}

        # Desired positions list
        self.distances = laplacian
        for i, d in enumerate(self.distances):
            if d != 0:
                # Error logger
                self.logger_dict.update(
                    {i: setup_logger(f'{self.name}_err_{i}', f'{LOG_PATH}{self.name}_err{i}.csv')}
                )
                # Fill position dictionaries
                npos = rospy.wait_for_message(f'/robot{i}/odom', Odometry)
                self.npos_dict[i] = [npos.pose.pose.position.x, npos.pose.pose.position.y]
                # Fill position subscriber dictionaries
                self.sub_dict.update(
                    {i: rospy.Subscriber(f'/robot{i}/odom', Odometry, self.update_npos, (i), queue_size=1)}
                )
                self.logger_dict[i].debug("time,err")

    def init_model(self, time):
        """Initialize robot x,y - models"""
        self.speed_model_x = RobotModel(0, time, parent=self, name=f'{self.name}_x_vel')
        self.speed_model_y = RobotModel(0, time, parent=self, name=f'{self.name}_y_vel')

    def update_state_odom(self, response):
        """Update robot's position from subscriber callback"""
        self.position_x = response.pose.pose.position.x if abs(response.pose.pose.position.x) > 0.0001 else 0
        self.position_y = response.pose.pose.position.y if abs(response.pose.pose.position.y) > 0.0001 else 0
        self.orientation = [response.pose.pose.orientation.x, response.pose.pose.orientation.y,
                            response.pose.pose.orientation.z, response.pose.pose.orientation.w]
        self.pos_logger.debug(f'{rospy.get_time()},{self.position_x},{self.position_y}')

    def update_npos(self, response, args):
        """Update neighbour's position"""
        self.npos_dict[args] = [response.pose.pose.position.x if abs(response.pose.pose.position.x) > 0.0001 else 0,
                                response.pose.pose.position.y if abs(response.pose.pose.position.y) > 0.0001 else 0]

    def get_complex_error(self, neighbour_index):
        """Get error value from temprorary subscriber"""
        npos = rospy.wait_for_message(f'/robot{neighbour_index}/odom', Odometry)
        npos_arr = arr([npos.pose.pose.position.x, npos.pose.pose.position.y])
        pos_arr = arr([self.position_x, self.position_y])
        cur_dis = linalg.norm(pos_arr - npos_arr)
        err = (cur_dis ** 2 - self.distances[neighbour_index] ** 2) * (npos_arr - pos_arr)
        self.logger_dict[neighbour_index].debug(f'{rospy.get_time()},{cur_dis - self.distances[neighbour_index]}')

        return err

    def get_complex_error_sub(self, neighbour_index):
        """Get error value from constant subscriber"""
        npos_arr = arr([self.npos_dict[neighbour_index][0], self.npos_dict[neighbour_index][1]])
        pos_arr = arr([self.position_x, self.position_y])
        cur_dis = linalg.norm(pos_arr - npos_arr)
        err = (cur_dis ** 2 - self.distances[neighbour_index] ** 2) * (npos_arr - pos_arr)
        self.logger_dict[neighbour_index].debug(f'{rospy.get_time()},{cur_dis - self.distances[neighbour_index]}')

        return err

    def set_vel(self, velocity):
        """Publish robot's velocity"""
        if self.robot_id != 0:
            # External
            if self.ext:
                t = rospy.get_time()
                ext = 0.1*math.sin(t*self.robot_id-self.robot_id)
                self.ext_logger.debug(f'{rospy.get_time()},{ext}')
                velocity.linear.x += ext
                velocity.linear.y += ext

            # Send control to dynamic model
            velocity.linear.x = self.speed_model_x.update_state(
                velocity.linear.x, rospy.get_time())
            velocity.linear.y = self.speed_model_y.update_state(
                velocity.linear.y, rospy.get_time())

            # Orientation correction
            (roll, pitch, yaw) = euler_from_quaternion(self.orientation)
            velocity.linear.x = velocity.linear.x * math.cos(-roll)
            velocity.linear.y = velocity.linear.y * math.sin(-roll + math.pi / 2)

            # Publish velocity
            self.publisher.publish(velocity)
            self.vel_logger.debug(f'{rospy.get_time()},{velocity.linear.x},{velocity.linear.y}')


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
        self.T = 0.1
        self.tf = control.tf([1], [self.T, 1])

    def update_state(self, inp, t):
        time, out = control.forced_response(self.tf, U=inp, T=[self.prev_time, t], X0=self.prev_out * self.T)
        self.prev_time = time[-1]
        self.prev_out = out[-1]
        return limit_value(self.prev_out, 1.0)

    def get_state(self):
        return self.prev_out


def setup_logger(name, log_file, level=logging.DEBUG):
    """To setup as many loggers as you want"""

    handler = logging.FileHandler(log_file, 'w')
    handler.setFormatter(logging.Formatter('%(message)s'))

    logger = logging.getLogger(name)
    logger.setLevel(level)
    logger.addHandler(handler)

    return logger


global_logger = setup_logger('vel_pos_comp_robot_logger', f'{LOG_PATH}main.csv')
