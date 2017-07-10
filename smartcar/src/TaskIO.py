#!/usr/bin/env python
#encoding:utf-8

import rospy
import socket
import ConfigParser
import os
import Queue
import time
import datetime
import json
import threading
from smartcar.msg import Task, TaskCmd, Feedback


class TaskIO:
    def __init__(self, cf):
        self.__cur_task, self.__next_task = None, None
        self.__mutex = threading.Lock()

        rospy.init_node('TaskIO', log_level=rospy.DEBUG)
        self.__task_pub = rospy.Publisher('task', Task, queue_size=5)
        rospy.Subscriber('feedback', Feedback, self.send_finish_taskId)

        socket_tuple = (cf.get("tcpip", "tasksocket_host"), cf.getint("tcpip", "tasksocket_port"))
        self.__task_socket = socket.socket()
        rospy.loginfo('socket connecting: %s' % str(socket_tuple))
        self.__task_socket.connect(socket_tuple)
        self.send_registration()
        rospy.loginfo('init taskio down')

    def __del__(self):
        self.__task_socket.close()

    def set_cur_task(self, task):
        self.__mutex.acquire()
        self.__cur_task = task
        self.__mutex.release()

    def set_next_task(self, task):
        self.__mutex.acquire()
        self.__next_task = task
        self.__mutex.release()

    def get_cur_task(self):
        self.__mutex.acquire()
        task = self.__cur_task
        self.__mutex.release()
        return task

    def get_next_task(self):
        self.__mutex.acquire()
        task = self.__next_task
        self.__mutex.release()
        return task

    def send_registration(self):
        msg = dict()
        msg['carId'] = 1
        msg['timeStamp'] = time.time()
        self.__task_socket.send(json.dumps(msg) + '\r\n')

    def send_finish_taskId(self, feedback):
        if feedback.finish_taskId != 0:
            msg = dict()
            msg['timeStamp'] = time.time()
            msg['finish_taskId'] = feedback.finish_taskId
            self.__task_socket.send(json.dumps(msg) + '\r\n')
            rospy.loginfo('send finish taskId: %s' % str(msg))

            # 异步请求next task
            cur_task, next_task = self.get_cur_task(), self.get_next_task()
            if cur_task is not None and feedback.finish_taskId == cur_task.index:
                if next_task is not None:
                    self.send_next_taskId(next_task.index + len(next_task.taskcmd_list) - 1)
        else:
            rospy.logerr("receive index 0 feedback:%s" % str(feedback))

    def send_next_taskId(self, index):
        if index <= 0:
            rospy.logerr("receive index<=0 next taskId:%s" % str(index))
            index = 1
        msg = dict()
        msg['timeStamp'] = time.time()
        msg['next_taskId'] = index
        self.__task_socket.send(json.dumps(msg) + '\r\n')
        rospy.loginfo('send next taskId: %s' % str(msg))

    def do_task(self, task):
        if task.index == 0:
            self.set_cur_task(None)
        else:
            self.set_cur_task(task)
        rospy.loginfo('start do task')
        self.__task_pub.publish(task)
        rospy.loginfo('task_pub send:\n%s' % task)

    def neutralize(self):
        task = Task()
        task.index = 0 # self-control command index
        self.do_task(task)

    def construct_task(self, msg):
        task = Task()
        task.index = msg['index']
        for i in msg['taskList']:
            taskcmd = TaskCmd(i['point']['x'], i['point']['y'], i['speed'], i['thetaTan'], i['r'],
                              i['deltaTime'])
            task.taskcmd_list.append(taskcmd)
        assert len(task.taskcmd_list) > 1
        return task

    def _decode_msg(self, msg):
        ret = False
        try:
            msg = json.loads(msg)
            if msg['index'] == 0:
                if self.get_next_task() is not None:
                    self.do_task(self.get_next_task())
                    self.set_next_task(None)
            else:
                task = self.construct_task(msg)
                if self.get_next_task() is None:
                    self.set_next_task(task)
                    next_task = self.get_next_task()
                    self.send_next_taskId(next_task.index + len(next_task.taskcmd_list) - 1)
                else:
                    self.do_task(self.get_next_task())
                    self.set_next_task(task)
                ret = True
        except Exception, e:
            rospy.loginfo('catch a exception: %s' % e)
        return ret

    def run(self):
        wait_msg = ""
        while True:
            rospy.loginfo("waiting for task...")
            recv_msg = self.__task_socket.recv(4096)
	    rospy.loginfo("receive msg:%s" % recv_msg)

            if len(recv_msg) == 0:
                rospy.loginfo("neutralize and flush task")
                break
            else:
                # solve TCP problem
                wait_msg += recv_msg
                if wait_msg.find('\r\n') == -1:
                    rospy.loginfo("not find ctl end flag")
                    continue
                single_msg, wait_msg = wait_msg.split('\r\n', 1)[0], wait_msg.split('\r\n', 1)[1]
                if self._decode_msg(single_msg) is False:
                    break
        self.neutralize()

        rospy.loginfo('mission complete')

if __name__ == '__main__':
    try:
        cf = ConfigParser.ConfigParser()
        cf.read(os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir, 'config/config.ini')))

        # continue to run forever
        taskIO = TaskIO(cf)
        taskIO.run()
        rospy.spin()

    except rospy.ROSInterruptException, e:
        rospy.loginfo(e)

