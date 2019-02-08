#!/usr/bin/env python
import rospy
import sys
import numpy
import cv2
from naoqi import ALProxy
from balling_nao.srv import GetMarkerRotation, \
    GetMarkerRotationResponse



class NaoController(object):

    def __init__(self, ip, port):

        self.nao_proxy = ALProxy("ALMotion", ip, port)
        rospy.init_node('move_joints_server') #better name is needed
        rospy.Service("rotation_server", GetMarkerRotation, self.get_yaw)

    def get_yaw(self, req):

        Rvec = numpy.array(req.Rvec)
        rotation_matrix, rotation_aruco_jacob = cv2.Rodrigues(Rvec)
        response = GetMarkerRotationResponse()
        response.yaw = numpy.arctan2(rotation_matrix[1][0], rotation_matrix[0][0])
        return response


if __name__ == '__main__':

    robotIP=str(sys.argv[1])
    PORT=int(sys.argv[2])
    print sys.argv[2]
    controller = NaoController(robotIP, PORT)
    rospy.spin()
