#!/usr/bin/python

import rospy
import actionlib
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from turtlebot3_move.srv import MoveInSquare, MoveInSquareRequest
from turtlebot3_move.msg import OdomRecordAction, OdomRecordGoal

RECORD_ODOM_ACT = "record_odom"
SQUARE_SVC = "/move_in_square"
LASER_TOPIC = "/scan"
VEL_TOPIC = "/cmd_vel"

def print_dist(odom_feedback):
    rospy.loginfo(f"{odom_feedback.current_total} travelled")

def move_forward(laser_msg):
    front = laser_msg.ranges[
        int(len(laser_msg.ranges)/2)
    ]
    
    vel_msg = Twist()
    if front < 0.2:
        vel_msg.linear.x = 0
        pub.publish(vel_msg)

        odom_client.cancel_goal()
        odoms = odom_client.wait_for_result()
        rospy.loginfo(odoms)

        rospy.signal_shutdown("Task completed")
    else:
        vel_msg.linear.x = 0.1
        pub.publish(vel_msg)

rospy.init_node("turtlebot3_sub_pub")

odom_client = actionlib.SimpleActionClient(RECORD_ODOM_ACT, OdomRecordAction)
odom_client.wait_for_server()
odom_client.send_goal(OdomRecordGoal(), feedback_cb=print_dist)

rospy.wait_for_service(SQUARE_SVC)
square_service = rospy.ServiceProxy(SQUARE_SVC, MoveInSquare)
result = square_service(MoveInSquareRequest())

pub = rospy.Publisher(
    name=VEL_TOPIC,
    data_class=Twist,
    queue_size=1,
)
while pub.get_num_connections() < 1:
    pass

sub = rospy.Subscriber(
    name=LASER_TOPIC, 
    data_class=LaserScan, 
    callback=move_forward,
)

rospy.spin()