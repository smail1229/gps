import copy
import time
import numpy as np

import rospy

import sys

sys.path.append('..')
from gps.agent.agent import Agent
from gps.agent.agent_utils import generate_noise, setup
from gps.agent.config import AGENT_ROS
from gps.agent.ros.ros_utils import ServiceEmulator, msg_to_sample, \
        policy_to_msg, tf_policy_to_action_msg, tf_obs_msg_to_numpy
from gps_agent_pkg.msg import TrialCommand, SampleResult, PositionCommand, \
        RelaxCommand, DataRequest

def controller_report_callback(data):
    baxter_pub = rospy.Publisher("baxter_sample_report", SampleResult)
    baxter_pub.publish(data)
    rospy.loginfo("I talk to you")

def gps_baxter_listener():

    rospy.init_node('gps_baxter_node')
    rospy.Subscriber("gps_controller_report", SampleResult, controller_report_callback)
    rospy.spin()

if __name__ == '__main__':
    gps_baxter_listener()