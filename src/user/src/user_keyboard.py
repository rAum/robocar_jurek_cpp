#!/usr/bin/env python
import sys
import rospy
import PySide

from PySide.QtGui import QApplication
from PySide.QtGui import QMessageBox


app = QApplication(sys.argv)

msgBox = QMessageBox()
msgBox.setText("Hello, this is PySide " + PySide.__version__)
msgBox.exec_()

def talker():
	pub = rospy.Publisher('user_keyboard')
	rospy.init_node('user')
	while not rospy.is_shutdown():
		str = "this is keyboard %s" % rospy.get_time()
		rospy.loginfo(str)
		pub.publish(String(str))
		rospy.sleep(1.0)

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass

