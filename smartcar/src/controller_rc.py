#!/usr/bin/env python

# ---------------------------------------------------------------------------
# Licensing Information: You are free to use or extend these projects for
# education or reserach purposes provided that (1) you retain this notice
# and (2) you provide clear attribution to UC Berkeley, including a link
# to http://barc-project.com
#
# Attibution Information: The barc project ROS code-base was developed at UC
# Berkeley in the Model Predictive Control (MPC) lab by Jon Gonzales
# (jon.gonzales@berkeley.edu) and Greg Marcil (grmarcil@berkeley.edu). The cloud
# services integation with ROS was developed by Kiet Lam
# (kiet.lam@berkeley.edu). The web-server app Dator was based on an open source
# project by Bruce Wootton
# ---------------------------------------------------------------------------

# README: This node serves as an outgoing messaging bus from odroid to arduino
# Subscribes: steering and motor commands on 'ecu'
# Publishes: combined ecu commands as 'ecu_pwm'

from rospy import init_node, Subscriber, Publisher, get_param
from rospy import Rate, is_shutdown, ROSInterruptException, spin, on_shutdown
from smartcar.msg import ECU_PWM
from numpy import pi
import rospy

def rc_inputs_callback(data):
    global throttle, steering
    throttle = data.motor
    steering = data.servo

def main_auto():
    global throttle, steering

    # initialize the ROS node
    init_node('manual_control', anonymous=True)
    Subscriber('rc_inputs', ECU_PWM, rc_inputs_callback)
    nh = Publisher('ecu_pwm', ECU_PWM, queue_size = 1)

    # set node rate
    rateHz = 50
    dt = 1.0/rateHz
    rate = Rate(rateHz)

    throttle = 1500
    steering = 1500

    # main loop
    while not is_shutdown():
        ecu_cmd = ECU_PWM(throttle, steering)
        nh.publish(ecu_cmd)
        rospy.loginfo('ecu_pwm_pub send [%s]' % str(ecu_cmd))
        rate.sleep()

#############################################################
if __name__ == '__main__':
    try:
        main_auto()
    except ROSInterruptException:
        pass
