#!/usr/bin/env python
import cv2
import rospy
import tensorflow as tf
import numpy as np

from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TwistStamped

from ddr_learner.models.base_learner import TrajectoryLearner

from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
import time
from math import sin, cos

bridge = CvBridge()


class Network(object):
    def __init__(self, config):

        self.config = config
        self.pub = rospy.Publisher("cnn_predictions", TwistStamped, queue_size=1)
        self.feedthrough_sub = rospy.Subscriber("state_change", Bool,
                                                self.callback_feedthrough, queue_size=1)
        self.checkpoint_sub = rospy.Subscriber("/checkpoint", String,
                                               self.callback_checkpoint, queue_size=1)

        self.gamma_sub = rospy.Subscriber("/gamma", String,
                                          self.callback_gamma, queue_size=1)

        self.learner = TrajectoryLearner()
        self.learner.setup_inference(config, mode='prediction')

        self.saver = tf.train.Saver([var for var in tf.trainable_variables()])
        self.use_network_out = False
        self.smoothed_pred = np.zeros((3,))
        self.alpha = 1.0
        self.checkpoint = ""
        self.gamma = ""

        self.state_estimate_sub = rospy.Subscriber("state_estimate", Odometry, self.callback_state_estimate, queue_size=1)
                
        # state dimension
        self.state_input = np.zeros(33)

        self.state_estimate_prev_prev = []
        self.state_estimate_prev = []
        self.state_estimate = []



    def callback_feedthrough(self, data):
        self.use_network_out = data.data
        if self.use_network_out:

            if self.config.ckpt_file:
                checkpoint = self.config.ckpt_file
            else:
                checkpoint = tf.train.latest_checkpoint(self.config.checkpoint_dir)
            self.saver.restore(self.sess, checkpoint)
            print("--------------------------------------------------")
            print("Restored checkpoint file {}".format(checkpoint))
            print("--------------------------------------------------")

    def callback_gamma(self, data):
        self.gamma = data.data

    def callback_checkpoint(self, data):
        self.config.ckpt_file = self.config.checkpoint_dir + self.gamma + "/" + data.data

    def callback_state_estimate(self, msg):
        self.state_estimate_prev_prev = self.state_estimate_prev
        self.state_estimate_prev = self.state_estimate
        self.state_estimate = msg

        if self.state_estimate_prev_prev != [] and self.state_estimate_prev != [] and self.state_estimate != []:
            r1 = R.from_quat([self.state_estimate_prev_prev.pose.pose.orientation.x, self.state_estimate_prev_prev.pose.pose.orientation.y, self.state_estimate_prev_prev.pose.pose.orientation.z, self.state_estimate_prev_prev.pose.pose.orientation.w])
            angle_prev_prev = r1.as_euler('zxy', degrees=False)

            r2 = R.from_quat([self.state_estimate_prev.pose.pose.orientation.x, self.state_estimate_prev.pose.pose.orientation.y, self.state_estimate_prev.pose.pose.orientation.z, self.state_estimate_prev.pose.pose.orientation.w])
            angle_prev = r2.as_euler('zxy', degrees=False)

            r3 = R.from_quat([self.state_estimate.pose.pose.orientation.x, self.state_estimate.pose.pose.orientation.y, self.state_estimate.pose.pose.orientation.z, self.state_estimate.pose.pose.orientation.w])
            angle = r3.as_euler('zxy', degrees=False)

            self.state_input  = np.array([angle_prev_prev[1], angle_prev_prev[2], angle_prev_prev[0], angle_prev[1], angle_prev[2], angle_prev[0], angle[1], angle[2], angle[0], \
                                        self.state_estimate_prev_prev.twist.twist.angular.x, self.state_estimate_prev_prev.twist.twist.angular.y, self.state_estimate_prev_prev.twist.twist.angular.z, \
                                        self.state_estimate_prev.twist.twist.angular.x, self.state_estimate_prev.twist.twist.angular.y, self.state_estimate_prev.twist.twist.angular.z, \
                                        self.state_estimate.twist.twist.angular.x, self.state_estimate.twist.twist.angular.y, self.state_estimate.twist.twist.angular.z, \
                                        (self.state_estimate_prev_prev.pose.pose.position.x - self.state_estimate_prev.pose.pose.position.x), (self.state_estimate_prev_prev.pose.pose.position.y - self.state_estimate_prev.pose.pose.position.y), \
                                        (self.state_estimate_prev_prev.pose.pose.position.z - self.state_estimate_prev.pose.pose.position.z), \
                                        (self.state_estimate_prev.pose.pose.position.x - self.state_estimate.pose.pose.position.x), (self.state_estimate_prev.pose.pose.position.y - self.state_estimate.pose.pose.position.y), \
                                        (self.state_estimate_prev.pose.pose.position.z - self.state_estimate.pose.pose.position.z), \
                                        self.state_estimate_prev_prev.twist.twist.linear.x, self.state_estimate_prev_prev.twist.twist.linear.y, self.state_estimate_prev_prev.twist.twist.linear.z, \
                                        self.state_estimate_prev.twist.twist.linear.x, self.state_estimate_prev.twist.twist.linear.y, self.state_estimate_prev.twist.twist.linear.z, \
                                        self.state_estimate.twist.twist.linear.x, self.state_estimate.twist.twist.linear.y, self.state_estimate.twist.twist.linear.z])
            
            # state dimension
            # self.max_vel = rospy.get_param("/hummingbird/drone_racing_node/max_velocity", 4.0)
            # self.state_input  = np.array([self.state_estimate.twist.twist.linear.x, self.state_estimate.twist.twist.linear.y, self.state_estimate.twist.twist.linear.z]) / self.max_vel              
            # self.state_input  = np.array([0.0,0.0,0.0])            


    def run(self, sess):
        self.sess = sess
        while not rospy.is_shutdown():
            data_camera = None

            while data_camera is None:
                try:
                    data_camera = rospy.wait_for_message("camera",
                                                         Image)

                except:
                    print("could not aquire image data")
                    break

            # Reading image and processing it through the network
            try:
                cv_input_image = bridge.imgmsg_to_cv2(data_camera)

            except CvBridgeError as e:
                print(e)
                continue

            inputs = {}

            cv_input_image = cv2.resize(cv_input_image, (300, 200),
                                 interpolation=cv2.INTER_LINEAR)

            inputs['images'] = cv_input_image[None]
 
            
            inputs['states'] = np.expand_dims(self.state_input, axis=0)

            results = self.learner.inference(inputs, sess)
            gate_predictions = np.squeeze(results['predictions'])
            final_output = np.squeeze(results['vel'])
            msg = TwistStamped()
            msg.header.stamp = rospy.Time.now()



            msg.twist.linear.x = max(min(self.alpha * final_output[0] + (1 - self.alpha) * self.smoothed_pred[0], 1.0), -1.0)
            msg.twist.linear.y = max(min(self.alpha * final_output[1] + (1 - self.alpha) * self.smoothed_pred[1], 1.0), -1.0)
            msg.twist.linear.z = max(min(self.alpha * final_output[2] + (1 - self.alpha) * self.smoothed_pred[2], 1.0), 0.0)

            # uncomment the below code to switch to another behavior which always heads to the next gate
            # msg.twist.linear.x = max(min(self.alpha * gate_predictions[0] + (1 - self.alpha) * self.smoothed_pred[0], 1.0), -1.0)
            # msg.twist.linear.y = max(min(self.alpha * gate_predictions[1] + (1 - self.alpha) * self.smoothed_pred[1], 1.0), -1.0)

            self.pub.publish(msg)
            self.smoothed_pred = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]
