#!/usr/bin/env python
#encoding:utf-8
import numpy as np
from scipy import interpolate
import ConfigParser
import os
import rospy
from smartcar.msg import Task, TaskCmd, Subtask, SubtaskCmd


class Subtasker:

    def __init__(self, cf):
        self.cloud_timestep = cf.getfloat("timeparams", "cloud_timestep")
        self.ctrl_timestep = cf.getfloat("timeparams", "ctrl_timestep")
        self.wheelbase = cf.getfloat("carparams", "wheelbase5")
        self.minR = cf.getfloat("carparams", "minTurnR5")
        self.floatZero = 0.00001

        # initialize ROS node
        rospy.init_node('subtasker', log_level=rospy.DEBUG)
        self.__subtask_pub = rospy.Publisher('subtask', Subtask, queue_size=20)
        rospy.Subscriber('task', Task, self.task_callback)
        rospy.loginfo('init subtasker down')

    def getTaskPoints(self, taskList):
        listX, listY, listTheta, listV, listR, listTime = [], [], [], [], [], []

        for taskcmd in taskList:
            listX.append(taskcmd.x)
            listY.append(taskcmd.y)
            listTheta.append(taskcmd.theta)
            listV.append(taskcmd.v)
            listR.append(taskcmd.r)
            listTime.append(taskcmd.deltaTime)

        return listX, listY, listTheta, listV, listR, listTime


    # Convert global coordinate to car coordinate
    def convert2car(self, xpoints, ypoints, thetapoints):
        carX = [0, ]
        carY = [0, ]
        OX = xpoints[0]
        OY = ypoints[0]
        THETA = np.degrees(np.arctan(thetapoints[0]))
        if THETA >= 0:
            for x, y in zip(xpoints, ypoints)[1:]:
                carX.append((x - OX) * np.cos(np.radians(THETA - 90)) + (y - OY) * np.sin(np.radians(THETA - 90)))
                carY.append(-(x - OX) * np.sin(np.radians(THETA - 90)) + (y - OY) * np.cos(np.radians(THETA - 90)))
        else:
            for x, y in zip(xpoints, ypoints)[1:]:
                carX.append((x - OX) * np.cos(np.radians(THETA + 90)) + (y - OY) * np.sin(np.radians(THETA + 90)))
                carY.append(-(x - OX) * np.sin(np.radians(THETA + 90)) + (y - OY) * np.cos(np.radians(THETA + 90)))
        return carX, carY

    # Decide Turn Left or Right
    def isDirection(self, xpoints, ypoints, thetapoints):
        RIGHT = 1
        LEFT = -1
        #boundry
        b = 0.1
        carX, carY = self.convert2car(xpoints, ypoints, thetapoints)

        if carX[len(carX) // 2] <= 0 and carX[len(carX) - 1] <= 0 and abs(carX[len(carX) - 1]) > b:
            return LEFT
        else:
            return RIGHT

    #create time axis from timelist in task
    def createTimeaxis(self, listTime):
        cloud_axis = []
        sum = 0
        for t in listTime:
            cloud_axis.append(sum + t)
            sum += t
        return cloud_axis
    # curvature function against time
    # def xt(self, t):
    #     return interpolate.splev(t, self.__tck_xt, der=0)

    # def yt(self, t):
    #     return interpolate.splev(t, self.__tck_yt, der=0)

    # def thetat(self, t):
    #     return interpolate.splev(t, self.__tck_thetat, der=0)

    def vt(self, t):
        return interpolate.splev(t, self.__tck_vt, der=0)    
    def rt(self,t):
        return interpolate.splev(t, self.__tck_rt, der=0)

    # helper fucntion
    def calcR(self, t):
        if abs(self.thetat(t)) < self.floatZero:
            return float('Inf')
        if abs(self.thetat(t + self.cloud_timestep)) < self.floatZero:
            return float('Inf')
        k1 = -1/self.thetat(t)
        b1 = self.yt(t) - k1 * self.xt(t)
        k2 = -1/self.thetat(t + self.cloud_timestep)
        b2 = self.yt(t + self.cloud_timestep) - k2 * self.xt(t + self.cloud_timestep)
        if abs((k1 - k2)) < self.floatZero:
            return float('Inf')
        xc = (b2 - b1)/(k1 - k2)
        yc = k1 * xc + b1

        R = np.sqrt(np.power(xc - self.xt(t), 2) + np.power(yc - self.yt(t), 2))
        Rmax = 1000
        if R > Rmax:
            return Rmax
        else:
            return R

    def getR(self, t, cloud_axis):
        N = len(cloud_axis)

        if self.rt(t) < self.minR:
            return self.minR
        else:
            return self.rt(t)

    def calcG(self, wheelbase, R):

        if abs(R)<self.floatZero:
            return round(np.degrees(np.arctan(1.0/self.floatZero)), 1)
        else:
            return round(np.degrees(np.arctan(wheelbase/R)), 1)

    def exception(self, taskList):
        x_points, y_points, theta_points, v_points, r_points, time_points = self.getTaskPoints(taskList)
        #rospy.loginfo('receive x points %s'%str(x_points))
        Nx, Ny, Ntheta, Nv, Nr, Nt = len(x_points), len(y_points), len(theta_points), len(v_points), len(r_points), len(time_points)
        assert Nx == Ny and Ny == Ntheta and Ntheta == Nv and Nv == Nr and Nr == Nt
        rospy.loginfo('This task total time %s' %str(sum(time_points)))
        assert sum(time_points) > 0


    def taskList_to_subtaskList(self, taskList, index):
        # Malfunction Check
        try:
            self.exception(taskList)
        except rospy.ROSInterruptException, e:
            rospy.loginfo(e)

        #Get task points
        x_points, y_points, theta_points, v_points, r_points, time_points = self.getTaskPoints(taskList)
        N = len(x_points)
        rospy.loginfo("Receive %s Task Points" % str(N))
	rospy.loginfo('Receive R Points; ' %r_points)
        # set time axis
        cloud_axis = self.createTimeaxis(time_points)
        totalTime = sum(time_points)
        ctrl_axis = np.arange(0, totalTime, self.ctrl_timestep)

        # fit curvature function params
        # rospy.loginfo("__cloud_axis: %s, __x_points: %s" % (__cloud_axis, __x_points))
        #self.__tck_xt = interpolate.splrep(cloud_axis, x_points)
        #self.__tck_yt = interpolate.splrep(cloud_axis, y_points)
        #self.__tck_thetat = interpolate.splrep(cloud_axis, theta_points)
        #self.__tck_vt = interpolate.splrep(cloud_axis, v_points)
        #self.__tck_rt = interpolate.splrep(cloud_axis, r_points)

        # generate total time control sequence in form of [(steer,v)......]
        #self.ctrlSequence = []
        #direction = self.isDirection(x_points, y_points, theta_points)
        # for t in ctrl_axis:
        #     r = self.getR(t)
        #     steer = direction * self.calcG(self.wheelbase, r)
        #     v = self.vt(t)
        #     self.ctrlSequence.append(SubtaskCmd(v, steer))

        # self.subtaskList = []
        # for i in range(index, index + N):
        #     subtask = Subtask()
        #     subtask.index = i
        #     if i == index + N - 1:
        #         subtask.isLastSubtask = True
        #     else:
        #         subtask.isLastSubtask = False
        #     for t in cloud_axis[0:(N - 1)]:
        #         subtask.subtaskcmd_list = (self.ctrlSequence[(int)(t / self.ctrl_timestep):(int)((t + self.cloud_timestep) / self.ctrl_timestep)])
        #     self.subtaskList.append(subtask)
        # return self.subtaskList

        self.subtaskList = []
        for i in range(index, index + N - 1):
            subtask = Subtask()
            subtask.index = i
            

            for t in np.arange(cloud_axis[i - index], cloud_axis[i - index + 1], self.ctrl_timestep):
                v = v_points[i - index + 1]
                r = r_points[i - index + 1]
		rospy.loginfo('r is: %s' % str(r))
                # if r > 0 turn left, if < 0 turn right, which is the opposite of servo on the car
                # steer > 0 turn right, < 0 turn left
		
                steer = - self.calcG(self.wheelbase, r)
                subtask.subtaskcmd_list.append(SubtaskCmd(v, steer))
            self.subtaskList.append(subtask)
        return self.subtaskList
    ####################################################################################
    def task_callback(self, task):
        index = task.index
        if index != 0:
            taskcmd_list = task.taskcmd_list
            subtask_list = self.taskList_to_subtaskList(taskcmd_list, index)
            for i in subtask_list:
                self.__subtask_pub.publish(i)
                rospy.loginfo('subtask is: %s' % str(i))
        else:
            subtask = Subtask()
            subtask.index = 0
            self.__subtask_pub.publish(subtask)

if __name__ == '__main__':
    try:
        cf = ConfigParser.ConfigParser()
        cf.read(os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir, 'config/config.ini')))
        subtasker = Subtasker(cf)
        rospy.spin()
    except rospy.ROSInterruptException, e:
        rospy.loginfo(e)
