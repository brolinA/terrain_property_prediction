#!/usr/bin/python3
#
# Copyright (c) 2022-2024, ETH Zurich, Matias Mattamala, Jonas Frey.
# All rights reserved. Licensed under the MIT license.
# See LICENSE file in the project root for details.
#
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Float32
import math
from nav_msgs.msg import Odometry
from wild_visual_navigation_msgs.msg import RobotState, CustomState
from aliengo_dynamics_computer.msg import FootForces, ReactionForce
import rospy

# Preallocate messages
robot_state_msg = RobotState()

# Extract joint states - state 0
joint_position = CustomState()
joint_position.name = "joint_position"
joint_position.dim = 12
joint_position.labels = [""] * joint_position.dim
joint_position.values = [0] * joint_position.dim
robot_state_msg.states.append(joint_position)

# Joint velocity - state 1
joint_velocity = CustomState()
joint_velocity.name = "joint_velocity"
joint_velocity.dim = 12
joint_velocity.labels = [""] * joint_velocity.dim
joint_velocity.values = [0] * joint_velocity.dim
robot_state_msg.states.append(joint_velocity)

# Acceleration - state 2
joint_acceleration = CustomState()
joint_acceleration.name = "joint_acceleration"
joint_acceleration.dim = 12
joint_acceleration.labels = [""] * joint_acceleration.dim
joint_acceleration.values = [0] * joint_acceleration.dim
robot_state_msg.states.append(joint_acceleration)

# Effort - state 3
joint_effort = CustomState()
joint_effort.name = "joint_effort"
joint_effort.dim = 12
joint_effort.labels = [""] * joint_effort.dim
joint_effort.values = [0] * joint_effort.dim
robot_state_msg.states.append(joint_effort)

# Vector state - state 4
vector_state = CustomState()
vector_state.name = "vector_state"
# vector_state.dim = 7 + 6  #(7 for pose, 6 for twist)
vector_state.dim = 12 #3 force comp for each leg
vector_state.labels = [""] * vector_state.dim
vector_state.values = [0] * vector_state.dim
robot_state_msg.states.append(vector_state)

i = 0
# for x in ["tx", "ty", "tz", "qx", "qy", "qz", "qw", "vx", "vy", "vz", "wx", "wy", "wz"]:
for x in ["FLx", "FLy", "FLz", "FRx", "FRy", "FRz","RLx", "RLy", "RLz","RRx", "RRy", "RRz",]:
    robot_state_msg.states[4].labels[i] = x #updating vector state
    i += 1

def aliengo_gazebo_msg_callback(aliengo_gazebo_state, return_msg=False):

    # For RobotState msg
    robot_state_msg.header = aliengo_gazebo_state.header

    # Keeping the old code here for reference
    # Extract pose
    # robot_state_msg.pose.header = aliengo_gazebo_state.header
    # robot_state_msg.pose.pose = aliengo_gazebo_state.pose.pose

    # Extract twist
    # robot_state_msg.twist.header = aliengo_gazebo_state.header
    # robot_state_msg.twist.header.frame_id = aliengo_gazebo_state.child_frame_id
    # robot_state_msg.twist.twist = aliengo_gazebo_state.twist.twist

    #FL
    robot_state_msg.states[4].values[0] = aliengo_gazebo_state.reaction_forces[0].wrench.force.x
    robot_state_msg.states[4].values[1] = aliengo_gazebo_state.reaction_forces[0].wrench.force.y
    robot_state_msg.states[4].values[2] = aliengo_gazebo_state.reaction_forces[0].wrench.force.z
    #FR
    robot_state_msg.states[4].values[3] = aliengo_gazebo_state.reaction_forces[1].wrench.force.x
    robot_state_msg.states[4].values[4] = aliengo_gazebo_state.reaction_forces[1].wrench.force.y
    robot_state_msg.states[4].values[5] = aliengo_gazebo_state.reaction_forces[1].wrench.force.z
    # RL
    robot_state_msg.states[4].values[6] = aliengo_gazebo_state.reaction_forces[2].wrench.force.x
    robot_state_msg.states[4].values[7] = aliengo_gazebo_state.reaction_forces[2].wrench.force.y
    robot_state_msg.states[4].values[8] = aliengo_gazebo_state.reaction_forces[2].wrench.force.z
    # RR
    robot_state_msg.states[4].values[9] = aliengo_gazebo_state.reaction_forces[3].wrench.force.x
    robot_state_msg.states[4].values[10] = aliengo_gazebo_state.reaction_forces[3].wrench.force.y
    robot_state_msg.states[4].values[11] = aliengo_gazebo_state.reaction_forces[3].wrench.force.z
    # robot_state_msg.states[4].values[12] = robot_state_msg.twist.twist.angular.z

    if return_msg:
        return robot_state_msg
    # Publish
    robot_state_pub.publish(robot_state_msg)


if __name__ == "__main__":
    rospy.init_node("aliengo_state_converter_node")

    robot_state_pub = rospy.Publisher("/wild_visual_navigation_node/robot_state", RobotState, queue_size=20)
    gazebo_force_sub = rospy.Subscriber("/normalized_gazebo_leg_forces_component", ReactionForce, aliengo_gazebo_msg_callback, queue_size=20)

    rospy.loginfo("[aliengo_state_converter_node] ready")
    rospy.spin()
