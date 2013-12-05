 #!/usr/bin/python

import serial
import rospy

from std_msgs.msg import Int32

#######################################################################################
# TODO: preprocess measurments, fix types (also in other nodes!!! like /user/manual.py)
#######################################################################################

class SerialDriver(object):
    """Serial driver for reading measurments such as break state nad steering wheel angle"""
    def __init__(self):
        self.sensor = { 'swheel' : 1, 'brake' : 2 }
        load_params(self)

    def load_params(self): # TODO: should load more data (and smarter!!) from Parameters Server.
        
        if rospy.has_param('/serial_driver/port'):
            self.PORT_NAME = rospy.get_param('/serial_driver/port')
        else:
            self.PORT_NAME         = "/dev/stty0"
            rospy.set_param('/serial_driver/port', self.PORT_NAME)

        if rospy.has_param('/serial_driver/freq'):
            self.FREQUENCY = rospy.get_param('/serial_driver/freq')
        else:
            self.FREQUENCY         = 100 # in Hz
            rospy.set_param('/serial_driver/freq')

        self.TICKS_TO_DIAGNOSE = 5000
        self.MSG_GET_STEERING_WHEEL_VAL       = serial.to_bytes(['1', 'P', '\n'])
        self.MSG_GET_BRAKE_VAL                = serial.to_bytes(['2', 'P', '\n'])
        self.MSG_GET_DIAGNOSIS_STEERING_WHEEL = serial.to_bytes(['1', 'D', '\n'])
        self.MSG_GET_DIAGNOSIS_BRAKE          = serial.to_bytes(['2', 'D', '\n'])
        self.MSG_GET_AMP_STEERING_WHEEL       = serial.to_bytes(['1', 'G', '\n']) # unused for now
        self.MSG_GET_AMP_BRAKE                = serial.to_bytes(['2', 'G', '\n']) # unused for now

        self.TIMEOUT_WRITE = 0.001
        self.TIMEOUT_READ  = 0.001

    def connect(self, tries = -1):
        self.ready = False
        while tries > 0 or tries == -1:
            try:
                tries -= 1
                self.port  = serial.Serial(self.PORT_NAME, 
                                           9600, 
                                           parity       = PARITY_NONE, 
                                           stopbits     = STOPBITS_ONE,
                                           timeout      = self.TIMEOUT_READ,
                                           writeTimeout = self.TIMEOUT_WRITE )
                self.ready = True
                break
            except:
                rospy.logwarn("No serial port {0}, remaining checks: {1}\n".format(self.PORT_NAME, tries))
                rospy.sleep(2.0)

        if self.ready:
            self.pub_brake          = rospy.Publisher('/car/val_brake',            Int32)
            self.pub_steering_wheel = rospy.Publisher('/car/val_steering_wheel',   Int32)
            self.pub_diag_brake     = rospy.Publisher('/serial_driver/diag_brake', Int32)
            self.pub_diag_steering_wheel = rospy.Publisher('/serial_driver/diag_steering_wheel', Int32)

            rospy.init_node('serial_driver')

            rospy.sleep(1.0)
        else:
            rospy.logfatal("Can't connect to serial port. Measurments of brake and steering wheel angle are unavailable!")

    def loop(self):
        if self.ready == False:
            return

        r = rospy.Rate(self.FREQUENCY) # 100 Hz

        diagnose_need = 0
        while not rospy.is_shutdown():
            if diagnose_need == 0:
                diagnose_need = self.TICKS_TO_DIAGNOSE
                self.do_diagnose()
            diagnose_need -= 1
            self.read_sensors()
            r.sleep()

    def read_sensors(self):
        self.read_angle(self.sensor['brake'] )
        self.read_angle(self.sensor['swheel'])

    def diagnose_element(self, what, callback):
       ask = None
       if what == 1:
           ask = self.MSG_GET_DIAGNOSIS_STEERING_WHEEL
       else:
           ask = self.MSG_GET_DIAGNOSIS_BRAKE

       if self.send_cmd(ask):
            msg = self.read_cmd(3)
            if msg != None:
                if msg[0] == 'D' and msg[2] == '\n':
                    callback(msg[1])
                elif msg[0] == 'E':
                    raise_i2c_error() 

    def push_diagnose_steering_wheel(self, msg):
        return

    def push_diagnose_brake(self, msg):
        return

    def do_diagnose(self):
        self.diagnose_element(self.sensor['swheel'], push_diagnose_steering_wheel)
        self.diagnose_element(self.sensor['brake'],  push_diagnose_brake)

    def push_diagnose_steering_wheel(self, value):
        self.pub_diag_steering_wheel(Int32(value))

    def push_diagnose_brake(self, value):
        self.pub_diag_brake.publish(Int32(value))

    def send_cmd(self, cmd):
        try:
            self.port.write(cmd)
        except:
            rospy.logwarn("sending '{0} failed.".format(cmd))
            return False
        return True

    def read_cmd(self, bytes_count=4):
        try:
            msg = self.port.read(bytes_count)
        except:
            rospy.logwarn("reading from serial port failed.")
            return None
        return msg

    def raise_i2c_error():
        rospy.logwarn("I2C transmision error. If it frequently occurs, contact someone from Electronic Team.")

    def push(self, sensor_num, value):
        if sensor_num == 1:
            self.pub_steering_wheel.publish(Int32(value))
        elif sensor_num == 2:
            self.pub_brake.publish(Int32(value))

    def read_angle(self, sensor_num):
        if sensor_num == 1:
            ask = self.MSG_GET_BRAKE_VAL
        else:
            ask = self.MSG_GET_STEERING_WHEEL_VAL

        if send_cmd(ask):
            msg = self.read_cmd()
            if msg != None:
               if msg[0] == 'A' and msg[3] == '\n':
                   value = msg[1] * 64 + msg[2]
                   self.push(sensor_num, value)
               elif msg[0] == 'E':
                    self.raise_i2c_error()


if __name__ == "__main__":
    driver = SerialDriver()
    driver.connect(100)
    driver.loop()
    rospy.loginfo("serial_driver has ended")

