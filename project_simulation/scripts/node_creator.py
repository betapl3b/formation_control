#!/usr/bin/python3
import roslaunch
import time, sys, subprocess
import rospy

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
LAMBDA0 = 0.8
SIGMA0 = 0.1
E = 0.6
GAZEBO_PATH = "/opt/ros/melodic/share/gazebo_ros/launch/empty_world.launch"
SPAWNER_LAUNCH = '/home/beta/catkin_ws/src/project_simulation/launch/spawner.launch'


def startNodes():
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
    launch.parent = roslaunch.parent.ROSLaunchParent(uuid, [f"{GAZEBO_PATH}"])

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

    coords = [[1, 1], [0, 1], [1, 0], [2, 1]]
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
    try:
        while True:
            continue
    except Exception as e:
        print(f"Got error: {e}")
        launch.stop()
        print("Stopping roscore")
        roscore.terminate()


if __name__ == "__main__":
    startNodes()
