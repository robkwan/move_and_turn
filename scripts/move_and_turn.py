#!/usr/bin/env python3

import sys
import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger, TriggerResponse


def start_move_and_turn_callback(req):
    move_start_time = rospy.Time.now()
    current_time = rospy.Time.now()
    elapsed_time = current_time - move_start_time

    # Move straight
    while elapsed_time.to_sec() < move_duration:
        cmd_vel.linear.x = move_speed
        cmd_vel.angular.z = 0.0
        pub_cmd_vel.publish(cmd_vel)
        elapsed_time = rospy.Time.now() - move_start_time

    # Turn
    while elapsed_time.to_sec() < move_duration + turn_duration:
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = turn_speed
        pub_cmd_vel.publish(cmd_vel)
        elapsed_time = rospy.Time.now() - move_start_time

    # Move straight
    while elapsed_time.to_sec() < 2 * move_duration + turn_duration:
        cmd_vel.linear.x = move_speed
        cmd_vel.angular.z = 0.0
        pub_cmd_vel.publish(cmd_vel)
        elapsed_time = rospy.Time.now() - move_start_time

    # Stop
    cmd_vel.linear.x = 0.0
    cmd_vel.angular.z = 0.0
    pub_cmd_vel.publish(cmd_vel)

    return TriggerResponse(success=True, message="Move and Turn Started")


rospy.init_node('move_and_turn_node', anonymous=False)
pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
move_distance = 0.5  # distance in meters to move straight
turn_angle = 90.0  # angle in degrees to turn
move_speed = 0.1  # speed in meters per second to move straight
turn_speed = 0.2  # speed in radians per second to turn
move_duration = move_distance / \
    move_speed  # duration in seconds to move straight
# duration in seconds to turn
turn_duration = (turn_angle / 180.0) * \
    3.141592653589793 / turn_speed
cmd_vel = Twist()
cmd_vel.linear.x = move_speed
cmd_vel.angular.z = 0.0

# create a service server
service_server = rospy.Service(
    '/start_move_and_turn', Trigger, start_move_and_turn_callback)

try:
    rospy.spin()
except rospy.ROSInterruptException:
    sys.exit()