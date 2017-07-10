#!/usr/bin/env python
#encoding:utf-8
import rospy
from smartcar.msg import ECU_PWM,ECU
from math import pi

class ControllerLowLevel:
    def __init__(self):
        # !!todo: use configuration file
        # max turning angle of 1/5 car
        self.__str_ang_max = 25
        self.__str_ang_min = -25
        # p is the params for different roads, this params should derive from experiments on different roads
        # p = 1.0 means its wheel is unloaded
	    # p = 2.x means its wheel is on the ground 
	
        self.__p = 2.5
	#steer neutral should be 1500, but our rc car always drive right, so neutral may be a little less than 1500
	self.neutral_steer = 1478
	#self.__p = 2.8
        # initialize ROS node
        rospy.init_node('arduino_interface', log_level=rospy.DEBUG)
        self.__ecu_pwm_pub = rospy.Publisher('ecu_pwm', ECU_PWM, queue_size=10)
        rospy.Subscriber('ecu', ECU, self.pwm_converter_callback, queue_size=10)

        # Set motor to neutral on shutdown
        rospy.on_shutdown(self.neutralize)
        rospy.loginfo('init controller_low_level down')

    # convert steer degree to pwm
    def steerPWM(self, steer_angle):
        # max pwm for servo is 1900 from experiment
        servo_pwm = self.neutral_steer + steer_angle * ((1900.0 - 1500.0) / self.__str_ang_max)
        return int(servo_pwm)
    # convert speed(m/s) to pwm
    def motorPWM(self, motor):
        if motor < 0.3 / self.__p:
            motor_pwm = 1500
        elif motor >= 0.3/self.__p and motor <= 0.5/self.__p:
            motor_pwm = 1550
        elif motor > 0.5/self.__p and motor <= 1.24 / self.__p:
            motor_pwm = 1600
        elif (motor >= 1.24/self.__p) and motor <= (2.11/self.__p):
            motor_pwm = 1605
        elif motor > (2.01/self.__p) and motor <= (2.78/self.__p):
            motor_pwm = 1620
        elif motor > 2.78/self.__p and motor <= 3.45/self.__p:
            motor_pwm = 1630
        elif motor > 3.45/self.__p and motor <= 4.03/self.__p:
            motor_pwm = 1640
        elif motor > 4.03/self.__p and motor <= 4.70/self.__p:
            motor_pwm = 1650
        elif motor > 4.70/self.__p and motor <= 5.37/self.__p:
            motor_pwm = 1660
        elif motor > 5.37/self.__p:
            motor_pwm = 1705
        return int(motor_pwm)
    def pwm_converter_callback(self, msg):
        # rospy.loginfo('receive ecu message: %s' % str(msg))

        servo_pwm = self.steerPWM(msg.servo)

        motor_pwm = self.motorPWM(msg.motor)

        self.update_arduino(motor_pwm, servo_pwm)

    def neutralize(self):
        motor_pwm = 1500
        servo_pwm = 1500
        self.update_arduino(motor_pwm, servo_pwm)

    def update_arduino(self, motor_pwm, servo_pwm):
        ecu_cmd = ECU_PWM(motor_pwm, servo_pwm)
        self.__ecu_pwm_pub.publish(ecu_cmd)
        # rospy.loginfo('ecu_pwm_pub send [%s]' % str(ecu_cmd))


#############################################################
if __name__ == '__main__':
    try:
        ControllerLowLevel()
        rospy.spin()
    except rospy.ROSInterruptException, e:
        rospy.loginfo(e)
