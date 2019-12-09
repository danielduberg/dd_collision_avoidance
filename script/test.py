#! /usr/bin/env python

from __future__ import print_function

import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import collision_avoidance.msg

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import numpy


path = Path()
pub = None
client = None

distance_converged = 0.1
yaw_converged = 0.1

current_pose = PoseStamped()


def start_callback(msg):
    print("Sending of path")
    collision_avoidance_client(path)


def pose_callback(pose):
    global current_pose

    current_pose = pose

    if len(path.poses) > 0:
        path.poses.insert(0, pose)
        pub.publish(path)
        path.poses.remove(pose)


def callback(pose):
    path.header = pose.header
    path.poses.append(pose)
    current_pose_saved = current_pose
    if current_pose_saved is not None:
        path.poses.insert(0, current_pose_saved)
        pub.publish(path)
        path.poses.remove(current_pose_saved)
    else:
        pub.publish(path)


def collision_avoidance_client(path_local):
    global path

    # Waits until the action server has started up and started
    # listening for goals.
    print("Waiting for server")
    client.wait_for_server()
    print("Connected to server")

    # Creates a goal to send to the action server.
    print("Create goal")
    goal = collision_avoidance.msg.PathControlGoal(path=path_local)

    path = Path()
    path.header = path_local.header
    path.header.stamp = rospy.Time.now()
    pub.publish(path)

    # Sends the goal to the action server.
    print("Send goal to server")
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    print("Waiting for result")
    client.wait_for_result()

    # Prints out the result of executing the action
    print("Done")


if __name__ == '__main__':
    try:
            # Initializes a rospy node so that the SimpleActionClient can
            # publish and subscribe over ROS.
        rospy.init_node('collision_avoidance_test')

        pub = rospy.Publisher("path", Path, latch=True, queue_size=10)
        sub = rospy.Subscriber("/setpoint", PoseStamped, callback)
        start_sub = rospy.Subscriber(
            "/initialpose", PoseWithCovarianceStamped, start_callback)
        pose_sub = rospy.Subscriber(
            "/mavros/local_position/pose", PoseStamped, pose_callback)

        client = actionlib.SimpleActionClient(
            'move_to2', collision_avoidance.msg.PathControlAction)

        rospy.spin()
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
