'''
MIT License

Copyright (c) 2023 Faxci-yet

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
'''


import rospy
from racecar.msg import Deviation
from geometry_msgs.msg import Twist


def doDeviation(dev):
    control_turn = 0
    control_speed = 1700
    # globals(control_turn)
    deviation = dev.deviation
    twist = Twist()
    turn_start_value = 30
    turn_bias = 0
    turn_mid = 90
    rospy.loginfo("i heard %.2f", dev.deviation)
    if deviation < 0: # <
        print('turn left')
        control_turn = (turn_start_value + turn_bias) + turn_mid
    elif deviation > 0:
        print('turn right')
        control_turn = -(turn_start_value + turn_bias) + turn_mid
    else:
        print('straight line!')
        control_turn = turn_mid

    twist.linear.x = control_speed;twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = control_turn
    # print "control_speed =",control_speed
    pub.publish(twist)

if __name__ == "__main__":
    #1.初始化节点
    rospy.init_node("carControl")
    #2.创建订阅者对象
    sub = rospy.Subscriber('/deviation',Deviation,doDeviation,queue_size=10)
    # 2.创建发布者对象
    pub = rospy.Publisher('~/car/cmd_vel', Twist, queue_size=5)
    rospy.spin() #4.循环
