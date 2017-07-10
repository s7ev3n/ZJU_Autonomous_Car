#!/usr/bin/env python
#encoding:utf-8
import datetime
import rospy
from smartcar.msg import Subtask, SubtaskCmd, ECU, Correction, Feedback
from math import pi
from numpy import zeros, array
from numpy import unwrap
import numpy as np
from speedCalc import SpeedCalc
from pid import PID


class Translator:

    def __init__(self):
        self.__correction_cmd = Correction(0, 0)
        # initialize ROS node
        rospy.init_node('translator', log_level=rospy.DEBUG)
        self.__ecu_pub = rospy.Publisher('ecu', ECU, queue_size=500)
        self.__feedback_pub = rospy.Publisher('feedback', Feedback, queue_size=500)
        rospy.Subscriber('imu/data', Imu, self.imu_callback)
        rospy.Subscriber('cv_bridge_demo', Correction, self.correction_callback)#type may should be changed(zlj)
        rospy.Subscriber('subtask', Subtask, self.subtask_callback)
        rospy.loginfo('init translator down')

    def __del__(self):
        self.neutralize()


    def se_callback(self, data):
        pass

    def correction_callback(self, data):
        print data, type(data)
	self.__correction_cmd.v=data.v
	self.__correction_cmd.steer=-data.steer
 	print  "real",self.__correction_cmd.v,self.__correction_cmd.steer
	

    def neutralize(self):
        motor, servo = 0, 0
        self.update_ecu(motor, servo)

    def update_ecu(self, motor, servo):
        ecu_cmd = ECU(motor, servo)
        self.__ecu_pub.publish(ecu_cmd)
        # rospy.loginfo('ecu_pub send:\n%s' % ecu_cmd)

    def subtask_callback(self, subtask):
        if subtask.index == 0:
            self.neutralize()
        else:
            r = rospy.Rate(20)  # 20hz
            # todo: use config file to read p, i, d and dt
            # pid = PID(P=0.0, I=40.0, D=0.0)
            # dt = 0.1
            for subtask_cmd in subtask.subtaskcmd_list:
                # curspeed = SpeedCalc()
                # v, steer = pid.update(curspeed, subtask_cmd.v, dt), subtask_cmd.steer + self.__correction_cmd.steer
		v = subtask_cmd.v + self.__correction_cmd.v
		steer = subtask_cmd.steer + self.__correction_cmd.steer
		#v, steer = subtask_cmd.v, subtask_cmd.steer +  self.__correction_cmd.v, self.__correction_cmd.steer
                
		motor, servo = v, steer
        self.update_ecu(motor, servo)
        r.sleep()
        self.task_feedback(subtask.index)

    def task_feedback(self, index):
        feedback = Feedback()
        feedback.finish_taskId = index
        self.__feedback_pub.publish(feedback)
        # rospy.loginfo('send feedback taskId: %s' % str(index))


if __name__ == '__main__':
    try:
        translator = Translator()
        rospy.spin()
    except rospy.ROSInterruptException, e:
        rospy.loginfo(e)
