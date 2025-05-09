#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
Control Your Robot!
---------------------------
Moving around:
   w    
a  s  d
   x    

w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity

s : stop

CTRL-C to quit
"""

moveBindings = {
    'w': (1, 0),
    'x': (-1, 0),
    'a': (0, 1),
    'd': (0, -1),
    's': (0, 0),
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('testbed_v5_teleop')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
    
    speed = 3.0
    turn = 1.0
    x = 0
    th = 0
    status = 0

    try:
        print(msg)
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][1]
            elif key == '' and (x != 0 or th != 0):
                x = 0
                th = 0
            
            twist = Twist()
            twist.linear.x = x*speed
            twist.angular.z = th*turn
            pub.publish(twist)

            if (key == '\x03'):
                break

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
