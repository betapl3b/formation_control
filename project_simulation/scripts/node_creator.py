#!/usr/bin/python3
import roslaunch
import time, sys, subprocess
import rospy
from random import sample

CORE = True
LAPLACIANS = {
    "3": [
        [0, 2, 2],
        [2, 0, 2],
        [2, 2, 0],
    ],
    "2": [
        [0, 2],
        [2, 0],
    ],
    "0": []
}
LAMBDA0 = 0.5
SIGMA0 = 0.03
E = 1
GAZEBO_PATH = "/opt/ros/melodic/share/gazebo_ros/launch/empty_world.launch"
WORLD_PATH = "/home/beta/catkin_ws/src/project_simulation/worlds/ml_plugin_room.world"
SPAWNER_LAUNCH = '/home/beta/catkin_ws/src/project_simulation/launch/spawner.launch'

process_running = True


class ProcessListener(roslaunch.pmon.ProcessListener):
    global process_running

    def process_died(self, name, exit_code):
        global process_running
        process_running = False
        rospy.logwarn("%s died with code %s", name, exit_code)


def start_nodes():
    if CORE:
        print("Starting roslaunch Python script")
        print("Starting roscore")
        roscore_popen_file = open("roscore_popen.log", "w+")
        roscore_popen_err_file = open("roscore_popen_err.log", "w+")
        roscore = subprocess.Popen('roscore', stdout=roscore_popen_file, stderr=roscore_popen_err_file)
        time.sleep(2)  # wait a bit to be sure the roscore is really launched

    print("Starting roslaunch")
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    # Creating launch
    launch = roslaunch.scriptapi.ROSLaunch()
    # firstly run gazebo.launch file
    run_gazebo_cli = [f'{GAZEBO_PATH}', f'world_name:={WORLD_PATH}', 'paused:=false', 'use_sim_time:=true']
    gazebo_args = run_gazebo_cli[1:]
    roslaunch_gazebo = [(roslaunch.rlutil.resolve_launch_arguments(run_gazebo_cli)[0], gazebo_args)]
    launch.parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_gazebo)

    print("Starting gazebo")
    launch.start()
    time.sleep(4)  #
    # wait a bit to be sure the roscore is really launched
    print("Reading arguments")
    try:
        form_number = str(sys.argv[1])
    except IndexError as e:
        print('==========Wrong number of arguments!=======')
        raise e
    except Exception as e:
        raise e
    laplacian = LAPLACIANS.get(form_number)
    number_of_nodes = len(laplacian)

    print("Creating {} nodes...".format(number_of_nodes))
    rospy.init_node('The_creator', anonymous=True)
    coords = list(zip(sample(range(0,number_of_nodes*2,1), number_of_nodes), sample(range(0,number_of_nodes*2,1), number_of_nodes)))
    # coords = [[1, 1], [0, 1], [1, 0], [2, 1],]
    spawners = []

    for n in range(number_of_nodes):
        spawners.append((SPAWNER_LAUNCH, [f"x:={coords[n][0]}", f"y:={coords[n][1]}", f"name:=robot{str(n)}"]))

    launch.parent = roslaunch.parent.ROSLaunchParent(uuid, spawners)
    launch.start()
    time.sleep(5)
    for n in range(number_of_nodes):
        controller = roslaunch.core.Node(
            'project_simulation', 'controller_creator.py',
            name='controller', namespace=f'robot{n}',
            args=f'"{n}" "{laplacian[n]}" "{LAMBDA0}" "{SIGMA0}" "{E}" "{True}"',
        )
        launch.launch(controller)

    print("All done. Stopping roslaunch.")
    while process_running:
        continue

    launch.shutdown()


if __name__ == "__main__":
    start_nodes()
