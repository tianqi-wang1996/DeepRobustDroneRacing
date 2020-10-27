#!/usr/bin/env python

import rospy
from Network import Network
import os, datetime
import tensorflow as tf
import sys
import gflags
from ddr_learner.common_flags import FLAGS

os.environ["CUDA_DEVICE_ORDER"]="PCI_BUS_ID"
os.environ["CUDA_VISIBLE_DEVICES"]="0"

def run_network():

    rospy.init_node('deep_drone_racing_learned_traj', anonymous=True)

    config = tf.ConfigProto()
    config.gpu_options.allow_growth=True

    # RUN NETWORK
    with tf.Session(config=config) as sess:
        network = Network.Network(FLAGS)
        sess.run(tf.global_variables_initializer())
        network.run(sess)

def parse_flags(argv):
    # Utility main to load flags
    try:
      argv = FLAGS(argv)  # parse flags
    except gflags.FlagsError:
      print ('Usage: %s ARGS\\n%s' % (sys.argv[0], FLAGS))
      sys.exit(1)

if __name__ == "__main__":
    parse_flags(sys.argv)
    run_network()
