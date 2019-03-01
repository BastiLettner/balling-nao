#!/usr/bin/env python
# Joint service node

import rospy
import time
import almath
import sys
from naoqi import ALProxy
from balling_nao.srv import MoveJoints, MoveJointsResponse


class NaoController(object):

    """ Service that moves Naos joints on request
        The request contains names, angles, speed and the response contains the new angles
    """

    def __init__(self, ip, port):
        """

        Args:
            ip: NAO_IP
            port: NAO_PORT
        """
        self.nao_proxy = ALProxy("ALMotion", ip, port)
        rospy.init_node('move_joints_server')
        rospy.Service("set_joints_server", MoveJoints, self.process_request)  # create the server

    def process_request(self, req):
        """

        Args:
            req: The request containing the names, angles and the speed.

        Returns:
            response: Contains the new angles
        """

        self.set_stiffness(req, 1.0)  # disable stiffness for joints on request

        # kill task call
        task_list = self.nao_proxy.getTaskList()
        for task in task_list:
            self.nao_proxy.killTask(task[1])

        req.angles = list(req.angles)  # set the angles to a list

        self.normal_movement(req)  # perform movement

        useSensors = True
        sensorAngles = self.nao_proxy.getAngles(req.names, useSensors)
        response = MoveJointsResponse()  # create the response
        response.new_angles = sensorAngles  # set the sensed angles

        # print the changed joints
        for name, angle, new_angle in zip(req.names, req.angles, response.new_angles):
            print "Performed angle change of {} to {}".format(name, angle)
            print "New angles {}: {}".format(name, new_angle)

        return response  # return the new angles

    def set_stiffness(self, req, val):
        for name in req.names:
            for _ in xrange(20):
                self.nao_proxy.setStiffnesses(name, val)

    def normal_movement(self, req):

        # perform the movement(s)
        self.nao_proxy.setAngles(req.names, req.angles, req.fractionMaxSpeed)


if __name__ == '__main__':

    robotIP=str(sys.argv[1])
    PORT=int(sys.argv[2])
    print sys.argv[2]
    controller = NaoController(robotIP, PORT)
    rospy.spin()
