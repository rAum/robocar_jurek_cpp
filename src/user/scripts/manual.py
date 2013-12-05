#!/usr/bin/python

import sys
import rospy
import PySide

from PySide import QtGui, QtCore
from std_msgs.msg import Int32

def label_gen(text):
    l = QtGui.QLabel(text)
    l.setAlignment(QtCore.Qt.AlignCenter)
    return l

class TopicSub:
	def __init__(self, topic_short, topic, msg_type):
		self.topic = topic
		self.sub   = rospy.Subscriber(topic, msg_type, self.send)
		self.send_callback = None
	def send(self, value):
		print self.topic, value
		if self.send_callback != None:
			self.send_callback(self.topic, value)

	def set_callback(self, func):
		self.send_callback = func



class RosCommunication(QtCore.QObject):
    """Simple class to publish and subscribe ROS topics to 
    manual control car and see some data"""
     
    def update(self):
        #rospy.spin()
        return
        #print 'check ROS communicates - ros_spin()'

    def publish(self, what, value):
	if rospy.is_shutdown(): return
        self.pub[what].publish(Int32(value))

    def steer_changed(self, value):
        self.publish('steering_wheel', value)

    def speed_changed(self, value):
        self.publish('speed', value)

    def brake_changed(self, value):
        self.publish('brake', value)

    def throttle_changed(self, value):
       self.publish('throttle', value)

    def timerEvent(self, tim):
        self.update()

   
    def __init__(self):
        super(RosCommunication, self).__init__()
        root_topic  = '/car/'
        self.topics = ['speed', 'steering_wheel', 'brake', 'throttle']

        self.pub = { topic : rospy.Publisher(root_topic + 'set_' + topic, Int32) for topic in self.topics }
        self.sub = { topic : TopicSub(topic, root_topic + 'val_' + topic, Int32) for topic in self.topics }

        rospy.init_node('user')

        self.timer = QtCore.QBasicTimer()

    def start(self):
        self.timer.start(10, self)

    def subscribe(self, topic, function):
	self.sub[topic].set_callback(function)

class MainWindow(QtGui.QWidget):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.initUI()
        self.connect2ROS()

    def connect2ROS(self):
        self.communicator = RosCommunication()

        self.steering_display.valueChanged.connect(self.communicator.steer_changed)
        self.brake_display.valueChanged.connect(self.communicator.brake_changede
        self.throttle_display.valueChanged.connect(self.communicator.throttle_changed)
        self.speed_valueChanged.connect(self.communicator.speed_changed)

        self.communicator.start()
	self.communicator.subscribe('steering_wheel', self.valueDisplay)

    def valueDisplay(self, name, value):
	print name, value


    def initUI(self): #layout building
        self.resize(500,400)
        self.setWindowTitle('Jurek Autonomous Vehicle Manual Control')

        self.speed_label = label_gen('Predkosc [km/h]')
        self.speed_display = QtGui.QLCDNumber(self)
        self.speed_display.display(0)

        self.brake_label      = label_gen('Hamulec')
        self.brake_display    = QtGui.QProgressBar(self)
        self.brake_display.setOrientation(QtCore.Qt.Vertical)
        self.brake_display.setRange(0, 100)
	self.brake_display.setValue(0)

        self.throttle_label   = label_gen('Gaz')
        self.throttle_display = QtGui.QProgressBar(self)
        self.throttle_display.setOrientation(QtCore.Qt.Vertical)
        self.throttle_display.setRange(0, 100)
	self.throttle_display.setValue(0)

        self.steering_label   = label_gen('Kierownica')
        self.steering_display = QtGui.QDial(self)
        self.steering_display.setRange(-100, 100)


        grid = QtGui.QGridLayout()
        grid.setSpacing(5)
        
        grid.addWidget(self.steering_display, 0, 0, 8, 8)
        grid.addWidget(self.steering_label,   8, 0, 1, 8)
        grid.addWidget(self.speed_display,    0, 9, 2, 4)
        grid.addWidget(self.speed_label,      2, 9, 1, 4)
        grid.addWidget(self.brake_label,      8, 9, 1, 2)
        grid.addWidget(self.brake_display,    3, 9, 5, 1)
        grid.addWidget(self.throttle_label,   8, 11, 1, 2)
        grid.addWidget(self.throttle_display, 3, 11, 5, 1)

        self.setLayout(grid)
        self.show()

    # simulating valueChanged signal which unfortunately QLCDNumber lacks :(
    speed_valueChanged = QtCore.Signal(int)
    def speed_update(self, value):
        if value > 30 or value < 0: return
        self.speed_display.display(value)
        self.speed_valueChanged.emit(value)

    def keyPressEvent(self, event):
        key = event.key()

        if   key == QtCore.Qt.Key_W:
            self.speed_update(self.speed_display.intValue() + 1)
        elif key == QtCore.Qt.Key_S:
            self.speed_update(self.speed_display.intValue() - 1)
        elif key == QtCore.Qt.Key_A:
            self.steering_display.setValue(self.steering_display.value() - 8)
        elif key == QtCore.Qt.Key_D:
            self.steering_display.setValue(self.steering_display.value() + 8)
        elif key == QtCore.Qt.Key_Space:
            self.brake_display.setValue(self.brake_display.maximum())
	elif key == QtCore.Qt.Key_Q:
	    self.throttle_display.setValue(self.throttle_display.value()+2)
	elif key == QtCore.Qt.Key_E:
	    self.throttle_display.setValue(self.throttle_display.value()-4)

    def keyReleaseEvent(self, event):
        key = event.key()

        if key == QtCore.Qt.Key_Space:
            self.brake_display.setValue(self.brake.minimum())


if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    win = MainWindow()
    app.exec_()
