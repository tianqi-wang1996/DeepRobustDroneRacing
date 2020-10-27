#/usr/bin/env python
import rospy
import os
import time

from std_msgs.msg import Empty
from scipy.spatial.transform import Rotation as R
import random

class TestRacing(object):
    def __init__(self):

        self.crash_sub = rospy.Subscriber("/crashed", Empty,
                                                self.callback_crashed, queue_size=1)
        self.passed_gate_sub = rospy.Subscriber("/passed_gate", Empty,
                                                self.callback_passed_gate, queue_size=1)
        self.crashed = False
        self.passed_gates = 0
        # Where to log test data
        self.folder_idx = 5000

    def callback_crashed(self, data):
        self.crashed = True

    def callback_passed_gate(self, data):
        self.passed_gates = self.passed_gates + 1

    def run(self, num_iterations):
        # This function is only for testing, no data generation!
        self.crashed = False

        num_passed_gates = []
        time_before_crash = []

        for i in range(1, num_iterations + 1, 1):
            used_time = 0
            trails = 0
            used_time_threshold = 40


            while used_time < used_time_threshold and trails <= 4:
                os.system("timeout 1s rostopic pub /hummingbird/bridge/arm std_msgs/Bool 'False'")
                os.system("timeout 1s rostopic pub /hummingbird/state_change std_msgs/Bool 'False'")

                reverse_track = random.randint(0,1)
                if reverse_track == 1:
                    rospy.set_param("/reverse_track", True)
                    print('python Right Circle!!!!!!!!!!\n')
                else:
                    rospy.set_param("/reverse_track", False)
                    print('python Left Circle!!!!!!!!!!\n')

                trails += 1
                # set up simulation scenario
                print('Replacing quad for new run...')
                os.system('timeout 1s rostopic pub /hummingbird/autopilot/off std_msgs/Empty')

                # Network only
                os.system("timeout 1s rostopic pub /hummingbird/only_network std_msgs/Bool 'True'")   

                # start quadrotor
                print("Start quadrotor")
                os.system("timeout 1s rostopic pub /hummingbird/bridge/arm std_msgs/Bool 'True'")
                os.system("timeout 1s rostopic pub /hummingbird/autopilot/start std_msgs/Empty")

                # # Network enabled
                os.system("timeout 1s rostopic pub /hummingbird/state_change std_msgs/Bool 'True'")

                # # setup environment and start
                os.system("timeout 1s rostopic pub /hummingbird/setup_environment std_msgs/Empty")

                print("Start Testing...")
                time_start=time.time()
                while used_time + (time.time() - time_start) <= used_time_threshold:
                    if self.crashed == True:
                        print('crash@@@@@@@@@@@@@')
                        # stop algorithm
                        self.crashed = False
                        print("Stop collecting data")
                        os.system("timeout 2s rostopic pub /hummingbird/hard_stop std_msgs/Empty") 
                        break
                
                print("Passed Gates are {}".format(self.passed_gates))
                num_passed_gates.append(self.passed_gates)
                time_before_crash.append(time.time() - time_start)
                print('This trail last %f s'%(time.time() - time_start))

                used_time += time.time() - time_start
                print('accumulated used time: %d s'%(used_time))
            
                self.crashed = False
                self.passed_gates = 0

            os.system("pkill -9 rviz; pkill -9 gzserver")
        # endfor
        return num_passed_gates, time_before_crash