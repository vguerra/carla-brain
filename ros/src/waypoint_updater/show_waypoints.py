#!/usr/bin/python
# -*- coding: utf-8 -*-

"""
ZetCode PyQt4 tutorial

In the example, we draw randomly 1000 red points
on the window.

author: Jan Bodnar
website: zetcode.com
last edited: September 2011
"""

import sys, random, math
from PyQt5 import QtGui, QtCore, QtWidgets
from PyQt5.QtGui import QPainter, QColor, QPen
from PyQt5.QtCore import Qt, QPointF

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped, PoseStamped
from styx_msgs.msg import Lane
import threading


class Example(QtWidgets.QWidget):
    def __init__(self):
        super(Example, self).__init__()
        rospy.init_node('show_waypoints')
        self.lock = threading.Lock()

        self.waypoints = None
        self.base_waypoints = None
        self.steering = 0
        rospy.Subscriber('/final_waypoints',
                         Lane,
                         self.waypoints_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.base_waypoints_cb)
        rospy.Subscriber('/vehicle/steering_cmd',
                         SteeringCmd, self.steering_cb)
        self.initUI()


    def initUI(self):
        self.setGeometry(100, 100, 1500, 1000)
        self.setWindowTitle('Points')
        self.show()

    def paintEvent(self, e):
        qp = QtGui.QPainter()
        qp.begin(self)
        self.drawPoints(qp)
        qp.end()

    def drawPoints(self, qp):
        pen = QPen()
        pen.setWidth(4)
        pen.setColor(Qt.black)
        qp.setPen(pen)
        if self.base_waypoints:
            for waypoint in self.base_waypoints:
                x = waypoint.pose.pose.position.x/1000*400
                y = waypoint.pose.pose.position.y/1000*400-400
                qp.drawPoint(x, y)
        pen = QPen()
        pen.setWidth(6)
        pen.setColor(Qt.red)
        qp.setPen(pen)
        if self.waypoints:
            for waypoint in self.waypoints:
                x = waypoint.pose.pose.position.x/1000*400
                y = waypoint.pose.pose.position.y/1000*400-400
                qp.drawPoint(x, y)

        cx = 500
        cy = 500
        r = 100.0
        pen = QPen()
        pen.setWidth(3)
        pen.setColor(Qt.black)
        qp.setPen(pen)
        qp.drawEllipse(QPointF(cx,cy),r,r)

        pen = QPen()
        pen.setWidth(10)
        pen.setColor(Qt.red)
        qp.setPen(pen)
        x = cx + r * math.cos(-math.pi/2+self.steering*-1)
        y = cy + r * math.sin(-math.pi/2+self.steering*-1)
        qp.drawLine(QPointF(cx,cy), QPointF(x,y))

    def waypoints_cb(self, msg):
        self.waypoints = msg.waypoints
        with self.lock:
            self.repaint()

    def steering_cb(self, msg):
        self.steering = msg.steering_wheel_angle_cmd
        with self.lock:
            self.repaint()

    def base_waypoints_cb(self, msg):
        self.base_waypoints = msg.waypoints
        with self.lock:
            self.repaint()

def main():


    app = QtWidgets.QApplication(sys.argv)
    ex = Example()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()