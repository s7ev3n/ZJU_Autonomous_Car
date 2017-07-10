#!/usr/bin/env python
#encoding:utf-8
import rospy
import rosbag
import math
import time
from smartcar.msg import Encoder


class SpeedCalc:

    dt_v_enc = 0.8  # time interval for computing speed
    t0 = time.time()
    v = 0

    n_FL = 0  # counts in the front left tire
    #n_FR = 0  # counts in the front right tire
    #n_BL = 0  # counts in the back left tire
    #n_BR = 0  # counts in the back right tire
    n_FL_prev = 0
    #n_FR_prev = 0
    #n_BL_prev = 0
    #n_BR_prev = 0

    N = 8.0   # N is the number of magnets
    r_tire = 0.1    # r of wheel
    dx_qrt = 2.0 * math.pi * r_tire / N

    # record speed data
    #speed_bag = rosbag.Bag('speed.bag', 'w')

    def __init__(self):
        rospy.init_node('speedCalc', log_level=rospy.DEBUG)
        rospy.Subscriber('encoder', Encoder, self.enc_callback, queue_size=10)


    def enc_callback(self, data):
        # global v, t0, dt_v_enc, v_meas
        # global n_FL, n_FR, n_FL_prev, n_FR_prev
        # global n_BL, n_BR, n_BL_prev, n_BR_prev

        self.n_FL = data.FL
        # self.n_FR = data.FR
        # self.n_BL = data.BL
        # self.n_BR = data.BR

        # compute time elapsed
        tf = time.time()
        dt = tf - self.t0
        #print 'dt is: ', dt
        # if enough time elapse has elapsed, estimate v_x
        if dt >= self.dt_v_enc:
            # compute speed :  speed = distance / time
            v_FL = (self.n_FL - self.n_FL_prev) * self.dx_qrt / dt
            #v_FR = float(self.n_FR - self.n_FR_prev) * self.dx_qrt / dt
            #v_BL = float(n_BL - n_BL_prev) * self.dx_qrt / dt
            #v_BR = float(n_BR - n_BR_prev) * self.dx_qrt / dt

            # Uncomment/modify according to your encoder setup
            #self.v_means    = (v_FL + v_FR)/2.0
            # Modification for 3 working encoders
            #v_meas = (v_FL + v_BL + v_BR) / 3.0
            # Modification for bench testing (driven wheels only)
            # v = (v_BL + v_BR)/2.0
            self.v = v_FL

            # update old data
            self.n_FL_prev = self.n_FL
            #self.n_FR_prev = n_FR
            #n_BL_prev = n_BL
            #n_BR_prev = n_BR
            self.t0 = time.time()
            rospy.loginfo('speed is %f' % self.v)
            #self.speed_bag.write('speed', self.v)
        return self.v
    
#######################################################################
if __name__ == '__main__':
    try:
        speedCaclc = SpeedCalc()
        rospy.spin()
    except rospy.ROSInterruptException, e:
        rospy.loginfo(e)