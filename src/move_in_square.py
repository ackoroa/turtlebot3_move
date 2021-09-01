#! /usr/bin/env python

from enum import Enum

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from turtlebot3_move.srv import MoveInSquare, MoveInSquareResponse

SERVICE_NAME = "move_in_square"
LASER_TOPIC = "/scan"
VEL_TOPIC = "/cmd_vel"
CRASH_DISTANCE = 0.2

class MoveMode(Enum):
    STOP = "stop"
    FORWARD = "forward"
    TURN = "turn"

move_mode = MoveMode.STOP
crash_signal = False

def check_front(laser_msg):
    global move_mode
    global crash_signal

    front = laser_msg.ranges[
        int(len(laser_msg.ranges)/2)
    ]
    if move_mode == MoveMode.FORWARD and front < CRASH_DISTANCE:
        crash_signal = True
        move_mode = MoveMode.STOP
        pub.publish(Twist())

def move_in_square(request):
    global move_mode
    global crash_signal

    rospy.loginfo("Start moving in square")
    
    crash_signal = False
    vel_msg = Twist()
    for i in range(4):
        move_mode = MoveMode.FORWARD
        vel_msg.linear.x = 0.1
        vel_msg.angular.z = 0
        pub.publish(vel_msg)
        rospy.sleep(1.5)

        if crash_signal:
            break;

        move_mode = MoveMode.TURN
        vel_msg.linear.x = 0
        vel_msg.angular.z = -0.4
        pub.publish(vel_msg)
        rospy.sleep(4.25)

    move_mode = MoveMode.STOP
    pub.publish(Twist())

    response = MoveInSquareResponse()
    if crash_signal:
        response.complete = False
        rospy.logwarn("Obstacle detected, emergency stop!")
    else:    
        response.complete = True
        rospy.loginfo("Completed moving in square")

    return response

rospy.init_node(SERVICE_NAME)

sub = rospy.Subscriber(
    name=LASER_TOPIC, 
    data_class=LaserScan, 
    callback=check_front,
)

pub = rospy.Publisher(
    name=VEL_TOPIC,
    data_class=Twist,
    queue_size=1,
)
while pub.get_num_connections() < 1:
    pass

service = rospy.Service(
    f"/{SERVICE_NAME}", 
    MoveInSquare, 
    move_in_square,
)

rospy.spin()