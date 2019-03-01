#!/usr/bin/env python
# Rotation service node, extracts yaw from a rotation matrix
import rospy
import sys
import numpy
import cv2
from naoqi import ALProxy
from balling_nao.srv import GetMarkerRotation, \
    GetMarkerRotationResponse


class NaoController(object):

    """
    Service to get the rotation of an aruco marker
    """

    def __init__(self, ip, port):
        """

        Args:
            ip: NAO_IP
            port: NAO_PORT
        """

        self.nao_proxy = ALProxy("ALMotion", ip, port)
        rospy.init_node('rotation_server_node')
        rospy.Service("rotation_server", GetMarkerRotation, self.get_yaw)

    def get_yaw(self, req):
        """

        Args:
            req: The request containing the Rvec.

        Returns:
            response containing the yaw
        """

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
