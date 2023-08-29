import time
import argparse
import subprocess
import os
from os.path import join
import shutil

import numpy as np
import rospy
import rospkg

from gazebo_simulation import GazeboSimulation

# below are set dependent on whether it is BARN or DynaBARN
# INIT_POSITION = [-2, 3, 1.57]  # in world frame
# GOAL_POSITION = [0, 10]  # relative to the initial position

def compute_distance(p1, p2):
    return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5

def path_coord_to_gazebo_coord(x, y):
        RADIUS = 0.075
        r_shift = -RADIUS - (30 * RADIUS * 2)
        c_shift = RADIUS + 5

        gazebo_x = x * (RADIUS * 2) + r_shift
        gazebo_y = y * (RADIUS * 2) + c_shift

        return (gazebo_x, gazebo_y)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description = 'test BARN navigation challenge')
    parser.add_argument('--world_idx', type=int, default=0)
    parser.add_argument('--gui', default=False, action="store_true")
    parser.add_argument('--out', type=str, default="out.txt")
    parser.add_argument('--rviz', default=False, action="store_true")
    parser.add_argument('--algo', type=str, default="dyna_lflh", choices=["dwa", "lflh", "dyna_lflh"])
    parser.add_argument('--record', default=False, help="record gazebo simulation (doesnt require gui)", action="store_true")
    args = parser.parse_args()
    
    ##########################################################################################
    ## 0. Launch Gazebo Simulation
    ##########################################################################################
    is_static_barn = None
    if args.world_idx == -1: # empty world
        world_name = "/usr/share/gazebo-9/worlds/empty.world"
        INIT_POSITION = [11, 0, 3.14]  # in world frame
        GOAL_POSITION = [-20, 0]  # relative to the initial position
        is_static_barn = False
    elif args.world_idx < 300:  # static environment from 0-299
        world_name = "BARN/world_%d.world" %(args.world_idx)
        INIT_POSITION = [-2.25, 3, 1.57]  # in world frame
        GOAL_POSITION = [0, 10]  # relative to the initial position
        is_static_barn = True
    elif args.world_idx < 360:  # Dynamic environment from 300-359
        world_name = "DynaBARN/world_%d.world" %(args.world_idx - 300)
        INIT_POSITION = [11, 0, 3.14]  # in world frame
        GOAL_POSITION = [-20, 0]  # relative to the initial position
        is_static_barn = False
    elif args.world_idx < 420:  # Dynamic extended environment from 400-419
        # DB_extended params: --seed 1 --min_object 5 --max_object 10 --n_worlds 10 --min_speed 0.25 --max_speed 0.75 --min_std 0.1 --max_std 0.25 --min_order 1 --max_order 2
        # DB_05 params:
        world_name = "DB_05/world_%d.world" %(args.world_idx - 400)
        INIT_POSITION = [11, 0, 3.14]  # in world frame
        GOAL_POSITION = [-20, 0]  # relative to the initial position
        is_static_barn = False
    else:
        raise ValueError("World index %d does not exist" %args.world_idx)

    if is_static_barn:
        os.environ["JACKAL_LASER"] = "1"
        os.environ["JACKAL_LASER_MODEL"] = "ust10"
        os.environ["JACKAL_LASER_OFFSET"] = "-0.065 0 0.01"
    
    # world_name = "BARN/world_%d.world" %(args.world_idx)
    print(">>>>>>>>>>>>>>>>>> Loading Gazebo Simulation with %s <<<<<<<<<<<<<<<<<<" %(world_name))   
    rospack = rospkg.RosPack()
    base_path = rospack.get_path('jackal_helper')

    if not is_static_barn:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.path.join(base_path, "plugins")
        print("GAZEBO_PLUGIN_PATH:", os.environ['GAZEBO_PLUGIN_PATH'])

    launch_file = join(base_path, 'launch', 'gazebo_launch.launch')
    world_name = join(base_path, "worlds", world_name)
    
    record_db = "BARN" if is_static_barn else "DynaBARN"
    run_id = time.strftime("%Y-%m-%d_%H-%M-%S", time.gmtime())
    record_path = join("/home/saadaghani/recordings", record_db, args.algo,run_id)
    if not os.path.exists(record_path):
        os.makedirs(record_path)
    
    gazebo_process = subprocess.Popen([
        'roslaunch',
        launch_file,
        'front_laser:='+ ("true" if not is_static_barn else "false"),
        'world_name:=' + world_name,
        'gui:=' + ("true" if args.gui else "false"),
        'rviz:=' + ("true" if args.rviz else "false"),
        'recording:='+("true" if args.record else "false"),
        'extra_gazebo_args:='+("--record_path "+record_path if args.record else "")
    ])
    time.sleep(5)
    # GazeboSimulation provides useful interface to communicate with gazebo  
    gazebo_sim = GazeboSimulation(init_position=INIT_POSITION)
    gazebo_sim.pause()
    # print("gazebo/rviz launched. waiting 30 seconds")
    time.sleep(30)  # sleep to wait until the gazebo being created
    
    rospy.init_node('gym', anonymous=True) #, log_level=rospy.FATAL)
    rospy.set_param('/use_sim_time', True)
    
    
    
    
    init_coor = (INIT_POSITION[0], INIT_POSITION[1])
    goal_coor = (INIT_POSITION[0] + GOAL_POSITION[0], INIT_POSITION[1] + GOAL_POSITION[1])
    
    pos = gazebo_sim.get_model_state().pose.position
    # print(f"initial pos (from gazebo): {pos}")
    # print(f"INIT_POSITION: {INIT_POSITION}")
    # print(f"GOAL POSITION: {GOAL_POSITION}")
    # print(f"init_coor: {init_coor}")
    # print(f"goal_coor: {goal_coor}")


    curr_coor = (pos.x, pos.y)
    collided = True
    
    # check whether the robot is reset, the collision is False
    while compute_distance(init_coor, curr_coor) > 0.1 or collided:
        gazebo_sim.reset() # Reset jackal to the initial position
        pos = gazebo_sim.get_model_state().pose.position
        curr_coor = (pos.x, pos.y)
        collided = gazebo_sim.get_hard_collision()
        time.sleep(1)

    # start recording 
    # todo: start gazebo recording now. rather than before.
    #       can use "gz log -d 1" to start, or gazebo_sim.unpause()
    # issue with gazebo_sim.unpause() is that gazebo log file does not track jackal model if it starts in a paused state
    gazebo_sim.unpause()
    if args.record:
        recorder_process = subprocess.Popen([
            'roslaunch',
            join(base_path, 'launch', 'recorder_launch.launch'),
            'save_dir:=' + record_path,
        ])

    ##########################################################################################
    ## 1. Launch your navigation stack
    ## (Customize this block to add your own navigation stack)
    ##########################################################################################
    
    if args.algo == "dwa":
        launch_file = join(base_path, 'launch', 'move_base_DWA.launch')
    elif args.algo == "lflh":
        launch_file = "../hallucination/nav_competition_icra2022.launch"
    elif args.algo == "dyna_lflh":
        launch_file = "../dynamic_hallucination/nav_competition_icra2022.launch"
    else:
        raise Exception(f"incorrect algo option: {args.algo}")
    
    if is_static_barn:
        nav_stack_process = subprocess.Popen([
            'roslaunch',
            launch_file
        ])
    else:
        # for dyna barn - DWA initially
        # launch_file = join(base_path, 'launch', 'move_base_DWA.launch')
        move_base_process = subprocess.Popen([
            'roslaunch',
            launch_file
        ])

    # the following lines are needed for DWA Move base to work
    # Make sure your navigation stack recives a goal of (0, 10, 0), which is 10 meters away
    # along postive y-axis.
    import actionlib
    from geometry_msgs.msg import Quaternion
    from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
    nav_as = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    mb_goal = MoveBaseGoal()
    mb_goal.target_pose.header.frame_id = 'odom'
    mb_goal.target_pose.pose.position.x = GOAL_POSITION[0]
    mb_goal.target_pose.pose.position.y = GOAL_POSITION[1]
    mb_goal.target_pose.pose.position.z = 0
    mb_goal.target_pose.pose.orientation = Quaternion(0, 0, 0, 1)

    nav_as.wait_for_server()
    nav_as.send_goal(mb_goal)

    # if not is_static_barn:
    #     nav_stack_process = subprocess.Popen([
    #         "python3",
    #         "navigation_stack.py"
    #     ])

    # print("mb_goal: ", mb_goal)

    ##########################################################################################
    ## 2. Start navigation
    ##########################################################################################
    
    curr_time = rospy.get_time()
    pos = gazebo_sim.get_model_state().pose.position
    curr_coor = (pos.x, pos.y)

    
    # check whether the robot started to move
    while compute_distance(init_coor, curr_coor) < 0.1:
        curr_time = rospy.get_time()
        pos = gazebo_sim.get_model_state().pose.position
        curr_coor = (pos.x, pos.y)
        # print(f"robot moving? curr_coor: {curr_coor}")
        time.sleep(0.01)
    
    # start navigation, check position, time and collision
    start_time = curr_time
    start_time_cpu = time.time()
    collided = False
    
    while compute_distance(goal_coor, curr_coor) > 3 and not collided and curr_time - start_time < 30:
        curr_time = rospy.get_time()
        pos = gazebo_sim.get_model_state().pose.position
        curr_coor = (pos.x, pos.y)
        # print("Time: %.2f (s), x: %.2f (m), y: %.2f (m)" %(curr_time - start_time, *curr_coor), end="\r")
        collided = gazebo_sim.get_hard_collision()
        while rospy.get_time() - curr_time < 0.1:
            time.sleep(0.01)


    
    
    ##########################################################################################
    ## 3. Report metrics and generate log
    ##########################################################################################
    
    print(">>>>>>>>>>>>>>>>>> Test finished! <<<<<<<<<<<<<<<<<<")
    success = False
    if collided:
        status = "collided"
    elif curr_time - start_time >= 100:
        status = "timeout"
    else:
        status = "succeeded"
        success = True
    print("Navigation %s with time %.4f (s)" %(status, curr_time - start_time))
    

    # if success: # delete recording files
    #     shutil.rmtree(record_path)

    # no navigation metric with dynaBARN so gotta comment out all these
    if is_static_barn:
        path_file_name = join(base_path, "worlds/BARN/path_files", "path_%d.npy" %args.world_idx)
        path_array = np.load(path_file_name)
        path_array = [path_coord_to_gazebo_coord(*p) for p in path_array]
        path_array = np.insert(path_array, 0, (INIT_POSITION[0], INIT_POSITION[1]), axis=0)
        path_array = np.insert(path_array, len(path_array), (INIT_POSITION[0] + GOAL_POSITION[0], INIT_POSITION[1] + GOAL_POSITION[1]), axis=0)
        path_length = 0
        for p1, p2 in zip(path_array[:-1], path_array[1:]):
            path_length += compute_distance(p1, p2)
        
        # Navigation metric: 1_success *  optimal_time / clip(actual_time, 4 * optimal_time, 8 * optimal_time)
        optimal_time = path_length / 2
        actual_time = curr_time - start_time
        nav_metric = int(success) * optimal_time / np.clip(actual_time, 4 * optimal_time, 8 * optimal_time)
        print("Navigation metric: %.4f" %(nav_metric))
    
    if args.algo == "dwa":
        algo_code = 1
    elif args.algo == "lflh":
        algo_code = 2
    elif args.algo == "dyna_lflh":
        algo_code = 3
    else:
        raise Exception("not implemented yet!")

    # for static barn:
    if is_static_barn:
        with open(args.out, "a") as f:
            f.write("%d %d %d %d %.4f %.4f %d %s\n" %(args.world_idx, success, collided, (curr_time - start_time)>=100, curr_time - start_time, nav_metric, algo_code, run_id))
    else:
        with open(args.out, "a") as f:
            f.write("%d %d %d %d %.4f %d %s\n" %(args.world_idx, success, collided, (curr_time - start_time)>=100, curr_time - start_time, algo_code, run_id))
    

    gazebo_process.terminate()
    gazebo_process.wait()
    if is_static_barn:
        nav_stack_process.terminate()
        nav_stack_process.wait()
    if not is_static_barn:
        move_base_process.terminate()
        move_base_process.wait()

    if args.record:
        recorder_process.terminate()
        recorder_process.wait()