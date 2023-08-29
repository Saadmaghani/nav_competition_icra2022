import argparse
import os
import os.path as path
import time
import subprocess

import rospkg
import rospy
from std_msgs.msg import Bool
from std_srvs.srv import Empty, SetBool

RECORDING_BASE_PATH = path.join(path.expanduser("~"),"recordings")


class Service():

    def __init__(self, no_gazebo=False, no_rviz=False, rosbag_name="player"):
        self.no_gazebo = no_gazebo
        self.no_rviz = no_rviz
        if not no_gazebo:
            self._pause_gazebo = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
            self._unpause_gazebo = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        
        if not no_rviz:
            self._rosbag_player_name=rosbag_name
            self._pause_rosbag = rospy.ServiceProxy('/'+rosbag_name+'/pause_playback', SetBool)
     
    def pause(self):
        if not self.no_gazebo:
            rospy.wait_for_service('/gazebo/pause_physics')
        if not self.no_rviz:
            rospy.wait_for_service('/'+self._rosbag_player_name+'/pause_playback')
        try:
            if not self.no_gazebo:
                self._pause_gazebo()
            if not self.no_rviz:
                self._pause_rosbag(True)
        except rospy.ServiceException:
            print ("/gazebo/pause_physics service call failed")

    def play(self):
        if not self.no_gazebo:
            rospy.wait_for_service('/gazebo/unpause_physics')
        if not self.no_rviz:
            rospy.wait_for_service('/'+self._rosbag_player_name+'/pause_playback')
        try:
            if not self.no_gazebo:
                self._unpause_gazebo()
            if not self.no_rviz:
                self._pause_rosbag(False)
        except rospy.ServiceException:
            print ("/gazebo/unpause_physics service call failed")

    # def restart_playback(self):
    #     """
    #     /gazebo/reset_world or /gazebo/reset_simulation will
    #     destroy the world setting, here we used set model state
    #     to put the model back to the origin
    #     """
    #     rospy.wait_for_service("/gazebo/set_model_state")
    #     try:
    #         self._reset(self._init_model_state)
    #     except (rospy.ServiceException):
    #         rospy.logwarn("/gazebo/set_model_state service call failed")

def parse_arguments():
    parser = argparse.ArgumentParser(description = '')
    parser.add_argument('-t', "--timestamp", type=str, help="timestamp of the recording", required=True)
    parser.add_argument('-r', "--rate", type=float, help="rate at which rosbag is played", default=0.5)
    parser.add_argument('--no_gazebo', default=False, action="store_true")
    parser.add_argument("--no_rviz", default=False, action="store_true")
    parser.add_argument("--testbed_algo", nargs=2, default=["DynaBARN", "lflh"])
    args = parser.parse_args()

    _path = path.join(RECORDING_BASE_PATH, "/".join(args.testbed_algo), args.timestamp)
    assert path.exists(_path), "path: "+str(_path)+" does not exist"
    return args


if __name__ == "__main__":
    args = parse_arguments()

    rospack = rospkg.RosPack()
    base_path = rospack.get_path('jackal_helper')

    launch_file = path.join(base_path, 'launch', 'recording_reviewer.launch')
    
    recording_path = path.join(RECORDING_BASE_PATH, "/".join(args.testbed_algo), args.timestamp)
    log_file = path.join(recording_path, "state.log")
    bag_file = [path.join(recording_path, f) for f in os.listdir(recording_path) if ".bag" in f][0]
    
    assert path.exists(log_file)
    assert path.exists(bag_file)
    
    reviewer_process = subprocess.Popen([
        'roslaunch',
        launch_file,
        'no_gazebo:=' + ("true" if args.no_gazebo else "false"),
        'no_rviz:=' + ("true" if args.no_rviz else "false"),
        'bag_file:='+bag_file,
        'log_file:='+log_file,
        'rate:='+str(args.rate)
    ])
    time.sleep(15)

    try:
        service_caller = Service(no_gazebo=args.no_gazebo, no_rviz=args.no_rviz)
        service_caller.play()

    finally:

        time.sleep(60)

        reviewer_process.terminate()
        reviewer_process.wait()


# TODO:
# replace time.sleep(60) with 5sec + max(bag.time, gazebo_sim.time)