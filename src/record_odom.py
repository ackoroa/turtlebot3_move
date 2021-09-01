#! /usr/bin/env python

import math

import rospy
import actionlib
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

from turtlebot3_move.msg import (
    OdomRecordAction,
    OdomRecordFeedback,
    OdomRecordResult,
)

ODOM_TOPIC = "/odom"

current_pose = None

def read_odom(odom):
    global current_pose
    current_pose = odom.pose.pose

def record_odom(goal):
    rospy.loginfo("Start recording odom")

    r = rospy.Rate(1)
    result = OdomRecordResult()
    prev_location = None
    dist_travelled = 0
    while True:
        if server.is_preempt_requested():
            rospy.loginfo("Finish recording odom")
            server.set_succeeded(result)
            break
        
        pose = current_pose
        q = pose.orientation
        theta, _, _ = euler_from_quaternion([q.w, q.x, q.y, q.z])

        record = Point()
        record.x = pose.position.x
        record.y = pose.position.y
        record.z = theta
        result.list_of_odoms.append(record)

        if prev_location is not None:
            dist_travelled += math.sqrt(
                (record.x - prev_location.x)**2 +
                (record.y - prev_location.y)**2
            )
        prev_location = record

        feedback = OdomRecordFeedback(current_total=dist_travelled)
        server.publish_feedback(feedback)

        r.sleep()

rospy.init_node("record_odom")

sub = rospy.Subscriber(
    name=ODOM_TOPIC, 
    data_class=Odometry, 
    callback=read_odom,
)
server = actionlib.SimpleActionServer(
    "record_odom", 
    OdomRecordAction, 
    record_odom,
    False
)
server.start()
rospy.loginfo("/record_odom server started")
rospy.spin()