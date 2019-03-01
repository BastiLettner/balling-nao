#!/usr/bin/env python
# Position (posture and walking) node
import rospy
import time
import motion
import numpy
import almath
import sys

import tf
from naoqi import ALProxy
from balling_nao.srv import GetTransform, \
    GetTransformResponse,\
    GetPosition,\
    GetPositionResponse,\
    Movement,\
    MovementResponse, \
    GoToPosture, \
    GoToPostureResponse, \
    MoveToPosition, \
    MoveToPositionResponse

FRAME_TORSO = 0
FRAME_WORLD = 1
FRAME_ROBOT = 2


class NaoController(object):

    """ Implements the following services:
            - Move into to a specified posture
            - Walk to position
    """

    def __init__(self, ip, port):
        """

        Args:
            ip: NAO_IP
            port: NAO_PORT
        """
        self.nao_proxy = ALProxy("ALMotion", ip, port)
        self.posture_proxy = ALProxy("ALRobotPosture", ip, port)
        self.autonomous_proxy = ALProxy("ALAutonomousMoves", ip, port)
        self.autonomous_proxy.setExpressiveListeningEnabled(False)  # disable aut. live
        rospy.init_node('move_joints_server')

        rospy.Service("go_to_posture_server", GoToPosture, self.handle_posture_request)  # create service
        rospy.Service("move_to_position_server", MoveToPosition, self.handle_move_to_position_request)  # create service

    def handle_posture_request(self, request):
        """

        Args:
            request: The request which contains the posture name

        Returns:
            response: True if success else false
        """

        response = GoToPostureResponse()
        posture_name = request.posture_name
        speed = request.speed

        self.posture_proxy.goToPosture(posture_name, speed)

        return response

    def handle_move_to_position_request(self, request):
        """

        Args:
            request: Request containing the position nao should walk to.

        Returns:
            response: True if success else false

        """
        response = MoveToPositionResponse()
        x = request.x
        y = request.y
        theta = request.theta

        self.nao_proxy.moveTo(x, y, theta)

        return response


if __name__ == '__main__':

    robotIP = str(sys.argv[1])
    PORT = int(sys.argv[2])
    print sys.argv[2]
    controller = NaoController(robotIP, PORT)
    rospy.spin()

