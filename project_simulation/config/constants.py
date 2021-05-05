#!/usr/bin/python3



class Formation:
    LAPLACIANS = {
        "3": [
            [0, 2, 2],
            [2, 0, 2],
            [2, 2, 0],
        ],
        "2": [
            [0, 2],
            [2, 0],
        ]
    }


class Controller:
    LAMBDA0 = 0.8
    SIGMA0 = 0.1
    E = 0.6


class Path:
    GAZEBO_WORLD = "/opt/ros/melodic/share/gazebo_ros/launch/empty_world.launch"
    SPAWNER_LAUNCH = '/home/beta/catkin_ws/src/project_simulation/launch/spawner.launch'
