#!/usr/bin/python

import sys, rospy, thread, time
from PyQt4 import QtGui, QtCore
from PyQt4.QtCore import *
from PyQt4.QtGui import *

from sphero_swarm_node.msg import SpheroTwist
from multi_apriltags_tracker.msg import april_tag_pos
from kris_alder_algo.msg import changeLeader
from geometry_msgs.msg import Pose2D

RADIUS = 40


class AgentSim(QtGui.QWidget):
    def __init__(self):
        super(QtGui.QWidget, self).__init__()
        self.selecting = False
        self.resize(960, 800)
        self.initUI() #TODO use to setup drawing areas.
        rospy.init_node('agent_sim', anonymous = True)
        self.cmdVelSub = rospy.Subscriber('/cmd_vel', SpheroTwist, self.cmdVelCallback)
        self.tagLocPub = rospy.Publisher('/april_tag_pos', april_tag_pos, queue_size = 1)
        self.changeLeadPub = rospy.Publisher('/change_leader',changeLeader, queue_size = 1)
        self.leader = -1;
        self.nextAgentNum = 1
        self.nextObsNum = 60
        self.started = False
        self.agentMap = {}
        self.obsMap = {}
        self.agentVel = {}
        self.blackPen = QtGui.QPen(QtCore.Qt.black, 2, QtCore.Qt.SolidLine)
        self.bluePen = QtGui.QPen(QtCore.Qt.blue, 2, QtCore.Qt.SolidLine)

    def initUI(self):
        p = self.palette()
        p.setColor(self.backgroundRole(), QtCore.Qt.white)
        self.setPalette(p)
        self.setContextMenuPolicy(QtCore.Qt.ActionsContextMenu)

        self.addAgentAction = QAction("Add Agent", self)
        self.addObsAction = QAction("Add Obstacle", self)
        self.startAction = QAction("Start", self)
        self.stopAction = QAction("Stop", self)
        self.stopAction.setEnabled(False)
        self.resetAction = QAction("Reset", self)

        self.addAgentAction.triggered.connect(self.addAgent)
        self.addObsAction.triggered.connect(self.addObs)
        self.startAction.triggered.connect(self.startSim)
        self.stopAction.triggered.connect(self.stopSim)
        self.resetAction.triggered.connect(self.reset)

        self.addAction(self.addAgentAction)
        self.addAction(self.addObsAction)
        self.addAction(self.startAction)
        self.addAction(self.stopAction)
        self.addAction(self.resetAction)
        self.show()



    def addAgent(self):
        pos = Pose2D()
        pos.theta = 0
        pos.x = self.mouseX
        pos.y = self.mouseY
        self.agentMap[self.nextAgentNum] = pos
        self.agentVel[self.nextAgentNum] = (0,0)
        self.nextAgentNum += 1
        rospy.set_param('/sphero_swarm/connected', {str(key) : str(key) for key in self.agentMap} )
        self.update()

    def addObs(self):
        pos = Pose2D()
        pos.theta = 0
        pos.x = self.mouseX
        pos.y = self.mouseY
        self.obsMap[self.nextObsNum] = pos
        self.nextObsNum += 1
        self.update()

    def move(self):
        rate = rospy.Rate(100)
        while self.started and not rospy.is_shutdown():
            for key in self.agentMap:
                self.agentMap[key].x += (self.agentVel[key][0] / 50)
                self.agentMap[key].y -= (self.agentVel[key][1] / 50)
                self.agentMap[key].x = max(0, min(960, self.agentMap[key].x))
                self.agentMap[key].y = max(0, min(800, self.agentMap[key].y))
            self.update()
            rate.sleep()



    def startSim(self):
        self.addAgentAction.setEnabled(False)
        self.addObsAction.setEnabled(False)
        self.startAction.setEnabled(False)
        self.stopAction.setEnabled(True)
        self.resetAction.setEnabled(False)
        self.started = True;
        thread.start_new_thread(self.move,())
        thread.start_new_thread(self.publishPos,())

    def publishPos(self):
        rate = rospy.Rate(50);
        while self.started and not rospy.is_shutdown():
            num=[]
            pos=[]
            for key, value in self.agentMap.iteritems():
                num.append(key)
                pos.append(value)
            for key, value in self.obsMap.iteritems():
                num.append(key)
                pos.append(value)
            msg = april_tag_pos();
            msg.id = num
            msg.pose = pos
            self.tagLocPub.publish(msg)
            rate.sleep()


    def stopSim(self):
        self.started = False;
        self.addAgentAction.setEnabled(True)
        self.addObsAction.setEnabled(True)
        self.startAction.setEnabled(True)
        self.stopAction.setEnabled(False)
        self.resetAction.setEnabled(True)
        for key in self.agentVel:
            self.agentVel[key] = (0,0)

    def reset(self):
        self.agentMap = {}
        self.obsMap = {}
        self.nextObsNum = 60
        self.nextAgentNum = 1
        self.agentVel = {}
        rospy.delete_param('/spher_swarm/connected')
        self.update()

    def paintEvent(self, event):
        painter = QtGui.QPainter();
        painter.begin(self)
        painter.setPen(self.blackPen)
        for key, value in self.obsMap.iteritems():
            rect = QRect(value.x - RADIUS/2, value.y - RADIUS/2, RADIUS, RADIUS)
            painter.drawRect(rect)
            painter.drawText(rect, QtCore.Qt.AlignCenter, str(key))

        painter.setPen(self.bluePen)
        for key, value in self.agentMap.iteritems():
            rectF = QRectF(value.x - RADIUS/2, value.y - RADIUS/2, RADIUS, RADIUS)
            painter.drawEllipse(rectF)
            painter.drawText(rectF, QtCore.Qt.AlignCenter, str(key))

        if self.leader != -1:
            value = self.agentMap[self.leader]
            rectF = QRectF(value.x - RADIUS/2, value.y - RADIUS/2, RADIUS, RADIUS)
            painter.setBrush(Qt.blue)
            painter.drawEllipse(rectF)
            painter.setPen(self.blackPen)
            painter.drawText(rectF, QtCore.Qt.AlignCenter, str(self.leader))
        painter.end()


    def cmdVelCallback(self, msg):
        if self.started:
            self.agentVel[int(msg.name)] = (msg.linear.x, msg.linear.y)
        else:
            return

    def mousePressEvent(self, QMouseEvent):
        mouse_state = QMouseEvent.button()

        self.mouseX = QMouseEvent.x()
        self.mouseY = QMouseEvent.y()

        if self.started and mouse_state == QtCore.Qt.LeftButton:
            newleader = -1;
            for key, value in self.agentMap.iteritems():
                dist = self.sqDist((value.x, value.y), (self.mouseX, self.mouseY))
                if(dist < (RADIUS * RADIUS)):
                    newleader = key
                    break
            self.setLeader(newleader, self.mouseX, self.mouseY)




    def setLeader(self, newleader, x, y):
        lid = -1
        lx = -1
        ly = -1
        if newleader == -1:

            if self.leader == -1:
                return
            lid = self.leader
            lx = x
            ly = y
        else:
            if newleader != self.leader:
                lid = newleader
                lx = self.agentMap[lid].x
                ly = self.agentMap[lid].y
                self.leader = newleader
            else:
                self.leader = -1

        l = changeLeader()
        l.id = lid
        pos = Pose2D()
        pos.theta = 0
        pos.x = lx
        pos.y = ly
        l.pose = pos
        self.changeLeadPub.publish(l)




    def sqDist(self, t1, t2):
        diffx = t1[0] - t2[0]
        diffy = t1[1] -t2[1]
        return (diffx * diffx) + (diffy * diffy)

if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    w = AgentSim()
    w.show()
    sys.exit(app.exec_())
