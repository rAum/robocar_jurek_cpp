#!/usr/bin/python

from subprocess import check_call
import rospy
from std_msgs.msg import Int32

class ThrottleServo:
    def __init__(self, channel=1):
        self.channel      = channel
        self.MIN_THROTTLE = 3900
        self.MAX_THROTTLE = 7000
        self.RANGE        = self.MAX_THROTTLE - self.MIN_THROTTLE

        rospy.init_node('throttle_servo')
        self.sub = rospy.Subscriber('/car/set_throttle', Int32, self.set_throttle)

    def work(self):
        rospy.spin()

    def set_throttle(self, value):
        value = self.trim_to_valid(value)
        #TEMPORAR SOLUTION!!! until code will be ported.
        try:
		check_call(["./UscCmd", '--servo {0},{1}'.format(self.channel, int(value))])
	except OSError:
		rospy.logfatal("Can't find UscCmd program, run in the same directory where it is")
	except:
		rospy.logfatal("Can't set throttle servo at maestro#{0} value: {1}".format(self.channel, value))

    def trim_to_valid(self, msg):
	v = msg.data
	v = int((v * self.RANGE)/100.0 + self.MIN_THROTTLE)
        if v < self.MIN_THROTTLE: return self.MIN_THROTTLE
        if v > self.MAX_THROTTLE: return self.MAX_THROTTLE
        return v

if __name__ == '__main__':
	driver = ThrottleServo()
	driver.work()

