#!/usr/bin/env python
import rospy
import time
import sys
from naoqi import ALProxy
from balling_nao.srv import HandControl


class NaoController(object):

    def __init__(self, ip, port):

        self.nao_proxy = ALProxy("ALMotion", ip, port)
        rospy.init_node('move_joints_server')
        rospy.Service("set_hands_server", HandControl, self.process_request)

    def process_request(self, req):

        # kill task call
        task_list = self.nao_proxy.getTaskList()
        for task in task_list:
            self.nao_proxy.killTask(task[1])

        if req.action_type == 0 :
            print "Opening Hand"
            self.nao_proxy.openHand(req.hand_name)
            self.set_stiffness(req.hand_name, 0.0)

        elif req.action_type == 1:
            print "Closing Hand"
            self.set_stiffness(req.hand_name, 1.0)
            self.nao_proxy.closeHand(req.hand_name)

        else:
            raise InvalidArgumentError("Received invalid argument for req.action_type: {}".format(req.action_type))

        return True

    def set_stiffness(self, name, val):
        for i in xrange(20):
            self.nao_proxy.setStiffnesses(name, val)

if __name__ == '__main__':

    robotIP=str(sys.argv[1])
    PORT=int(sys.argv[2])
    print sys.argv[2]
    controller = NaoController(robotIP, PORT)
    rospy.spin()
