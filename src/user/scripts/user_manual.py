#echo!/usr/bin/env python

import sys
import rospy
import PySide

from PySide import QtGui, QtCore
from std_msgs.msg import Int32

def label_gen(text):
    l = QtGui.QLabel(text)
    l.setAlignment(QtCore.Qt.AlignCenter)
    return l

class RosCommunication(QtCore.QObject):
    """Simple class to publish and subscribe ROS topics to 
    manual control car and see some data"""
     
    def update(self):
        #rospy.spin()
        return
        #print 'check ROS communicates - ros_spin()'

    def publish(self, what, value):
	if rospy.is_shutdown():
		rospy.loginfo('not publish, rospy is shutdown.')
		print ':('
		return
	rospy.loginfo(what + ' < ' + str(value))
        self.pub[what].publish(Int32(value))

    def steer_changed(self, value):
        self.publish('steering_wheel', value)

    def speed_changed(self, value):
        self.publish('speed', value)

    def break_changed(self, value):
        self.publish('break', value)

    def throttle_changed(self, value):
       self.publish('throttle', value)

    def timerEvent(self, tim):
        self.update()

    def __init__(self):
        super(RosCommunication, self).__init__()
        root_topic  = '/car/'
        self.topics = ['speed', 'steering_wheel', 'break', 'throttle']

        self.pub = { topic : rospy.Publisher(root_topic + 'set_' + topic, Int32) for topic in self.topics }
        #self.sub = { topic : (root_topic + 'get_' + topic, Foo()) for topic in self.topics }

        rospy.init_node('user', disable_signals=True)

        self.timer = QtCore.QBasicTimer()

    def start(self):
        self.timer.start(10, self)

class MainWindow(QtGui.QWidget):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.initUI()
        self.connect2ROS()

    def connect2ROS(self):
        self.communicator = RosCommunication()

        self.steering_display.valueChanged.connect(self.communicator.steer_changed)
        self.break_display.valueChanged.connect(self.communicator.break_changed)
        self.throttle_display.valueChanged.connect(self.communicator.throttle_changed)
        self.speed_valueChanged.connect(self.communicator.speed_changed)

        self.communicator.start()


    def initUI(self): #layout building
        self.resize(400,300)
        self.setWindowTitle('Jurek Autonomous Vehicle Manual Control')

        self.speed_label = label_gen('Predkosc [km/h]')
        self.speed_display = QtGui.QLCDNumber(self)
        self.speed_display.display(0)

        self.break_label      = label_gen('Hamulec')
        self.break_display    = QtGui.QProgressBar(self)
        self.break_display.setOrientation(QtCore.Qt.Vertical)
        self.break_display.setRange(0, 100)

        self.throttle_label   = label_gen('Gaz')
        self.throttle_display = QtGui.QProgressBar(self)
        self.throttle_display.setOrientation(QtCore.Qt.Vertical)
        self.throttle_display.setRange(0, 100)

        self.steering_label   = label_gen('Kierownica')
        self.steering_display = QtGui.QDial(self)
        self.steering_display.setRange(-100, 100)


        grid = QtGui.QGridLayout()
        grid.setSpacing(5)
        
        grid.addWidget(self.steering_display, 0, 0, 8, 8)
        grid.addWidget(self.steering_label,   8, 0, 1, 8)
        grid.addWidget(self.speed_display,    0, 9, 2, 4)
        grid.addWidget(self.speed_label,      2, 9, 1, 4)
        grid.addWidget(self.break_label,      8, 9, 1, 2)
        grid.addWidget(self.break_display,    3, 9, 5, 1)
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
            self.break_display.setValue(self.break_display.maximum())

    def keyReleaseEvent(self, event):
        key = event.key()

        if key == QtCore.Qt.Key_Space:
            self.break_display.setValue(self.break_display.minimum())


if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    win = MainWindow()
    app.exec_()
