#!/usr/bin/env python
import sys
import rospy
import PySide

from std_msgs.msg import String
from PySide.QtGui import QApplication, QMainWindow

# class MainWindow(QMainWindow):
	# def __init__(self, parent=None):
		# super(MainWindow, self).__init__(parent)
		# #self.connect(self, SIGNAL("keyPress(QKeyEvent*)"), self.keyPressEvent)
		# # ROS init
		# self.pub = rospy.Publisher('user_keyboard', String)
		# rospy.init_node('user', disable_signals=True)


	# def keyPressEvent(self, event):
		# if rospy.is_shutdown():
			# return
		# key = event.key()
		# if   key == Qt.Key_W:
			# self.pub.publish(String('PW'))
		# elif key == Qt.Key_S:
			# self.pub.publish(String('PS'))
		# elif key == Qt.Key_A:
			# self.pub.publish(String('PA'))
		# elif key == Qt.Key_D:
			# self.pub.publish(String('PD'))			
		

def talker():
	pub = rospy.Publisher('user_keyboard', String)
	rospy.init_node('user')
	while not rospy.is_shutdown():
		str = "this is keyboard %s" % rospy.get_time()
		rospy.loginfo(str)
		pub.publish(String(str))
		rospy.sleep(1.0)

if __name__ == '__main__':
	app = QApplication(sys.argv)

	try:
		talker()
	except rospy.ROSInterruptException:
		pass

