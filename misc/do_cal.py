import signal
import sys

import rospy
from std_msgs.msg import Int32MultiArray

logfile = open('raw_cal.txt', 'w')
calfile = open('KSC.cal', 'w')
x_min = sys.maxint
y_min = x_min
z_min = x_min
x_max = -sys.maxint - 1
y_max = x_max
z_max = x_max

def handle_exit(signal, frame) :
    global logfile, calfile, x_min, y_min, z_min, x_max, y_max, z_max
    print ('Saving KSC.cal.')
    calfile.write('min: {{ {}, {}, {} }} max: {{ {}, {}, {} }}\n'.format(x_min, y_min, z_min, x_max, y_max, z_max))
    calfile.close()

    print ('Closing raw_cal.txt')
    logfile.close()
    
    sys.exit(0)

def callback(data):
    global logfile, x_min, y_min, z_min, x_max, y_max, z_max

    mag = data.data[0:3]
    if mag[0] < x_min :
        x_min = mag[0]
    if mag[0] > x_max :
        x_max = mag[0]

    if mag[1] < y_min :
        y_min = mag[1]
    if mag[1] > y_max :
        y_max = mag[1]

    if mag[2] < z_min :
        z_min = mag[2]
    if mag[2] > z_max :
        z_max = mag[2]

    logfile.write('{}, {}, {}, {}, {}, {}, {}, {}\n'.format(*data.data))

def listener():
    rospy.init_node('CALIBRATE')
    rospy.Subscriber("/imu_raw", Int32MultiArray, callback)
    rospy.spin()

if __name__ == '__main__' :
    signal.signal(signal.SIGINT, handle_exit)
    listener()
    
