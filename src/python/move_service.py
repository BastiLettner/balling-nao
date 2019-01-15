#!/usr/bin/env python
import rospy
import time
import motion
import numpy
import almath
import sys

import tf
from naoqi import ALProxy
from control_2_basti.srv import GetTransform, \
    GetTransformResponse,\
    GetPosition,\
    GetPositionResponse,\
    Movement,\
    MovementResponse

FRAME_TORSO = 0
FRAME_WORLD = 1
FRAME_ROBOT = 2


class NaoController(object):

    def __init__(self, ip, port):

        self.nao_proxy = ALProxy("ALMotion", ip, port)
        rospy.init_node('move_joints_server')
        self.listener = tf.TransformListener()
        rospy.Service("get_position_server", GetPosition, self.handle_position_request)
        rospy.Service("set_position_server", Movement, self.handle_movement_request)
        rospy.Service("get_transformation", GetTransform, self.handle_transform_request)

    def handle_position_request(self, request):

        response = GetPositionResponse()
        name = request.joint_name
        response.coordinates_6D = self.nao_proxy.getPosition(name, FRAME_TORSO, True)

        return response

    def handle_movement_request(self, request):

        response = MovementResponse()
        name = request.name
        coordinates = request.coordinates
        speed = request.speed
        time = request.time
        is_abs = 1
        frame_torso = FRAME_TORSO

        for i in xrange(10):
            self.nao_proxy.setStiffnesses(name, 1.0)

        if len(coordinates) == 3:
            axis_mask = 7
            coordinates = coordinates + (0, 0, 0)
        else:
            axis_mask = 63

        if request.type == 0:
            self.nao_proxy.setPositions(name, FRAME_TORSO, coordinates, speed, axis_mask)

        elif request.type == 1:
            if request.use_both_arms:
                print "Moving bot arms"
                name = ["LArm", "RArm"]
                coordinates = [coordinates, coordinates]
                axis_mask = [axis_mask] * 2
                time = [time] * 2
                print "Changing ", name, " position to ", coordinates
                self.nao_proxy.positionInterpolations(name, frame_torso, coordinates, axis_mask, time, is_abs)
                name = "LArm"
            else:
                self.nao_proxy.positionInterpolation(name, frame_torso, coordinates, axis_mask, time, is_abs)


        else:
            raise Exception("Invalied Request type")

        response.coordinates = self.nao_proxy.getPosition(name, FRAME_TORSO, True)

        for i in xrange(10):
            self.nao_proxy.setStiffnesses(name, 0.0)
            
        return response

    def handle_transform_request(self, request):

        name = request.name
        space = request.space
        use_sensors = True
        result = self.nao_proxy.getTransform(name, space, use_sensors)

        response = GetTransformResponse()
        response.H = result
        return response


if __name__ == '__main__':

    robotIP = str(sys.argv[1])
    PORT = int(sys.argv[2])
    print sys.argv[2]
    controller = NaoController(robotIP, PORT)
    rospy.spin()

