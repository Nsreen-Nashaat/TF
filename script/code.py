#!/usr/bin/env python3 

# import libraries 
import rospy
import moveit_commander
import sys
import moveit_msgs.msg
from math import pi 
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import _Constraints,JointConstraint
from std_msgs.msg import Int8

flag1 = 0 
flag2 = 0 
flag3 = 0
flag4 = 0



#init node
rospy.init_node("moveit_node",anonymous=True)

#init moveit 
moveit_commander.roscpp_initialize(sys.argv)

#describtion 

robot=moveit_commander.robot.RobotCommander()
print(robot.get_group_names())

#planning group 
moveit_group = moveit_commander.MoveGroupCommander("manipulator")
hand = moveit_commander.MoveGroupCommander("gripper")

moveit_group.set_pose_reference_frame("base_link")
moveit_group.set_end_effector_link("tool0")

end_effector_link = moveit_group.get_end_effector_link()

moveit_group.set_goal_position_tolerance(0.005)
moveit_group.allow_replanning(True)
# moveit_group.set_planning_id()

#publisher
pub=rospy.Publisher("/move_group/display_planned_path",moveit_msgs.msg.DisplayTrajectory,queue_size=20)


Home=[0,-2*pi/3,5*pi/9,pi/9,pi/2,-pi/2]
moveit_group.go(Home,wait=True)

start_pose = moveit_group.get_current_pose().pose

print(moveit_group.get_active_joints())
rospy.sleep(10)

target = PoseStamped()
target.header.frame_id = 'base_link'
      
goal_z = 0.4052581061654696 - (0.02 + 0.03)

target.pose.position.x = 0.4736232185480931 - 0.25 + 0.015
target.pose.position.y = -0.05969792123713007
target.pose.position.z = goal_z

target.pose.orientation.x = start_pose.orientation.x
target.pose.orientation.y = start_pose.orientation.y
target.pose.orientation.z = start_pose.orientation.z
target.pose.orientation.w = start_pose.orientation.w
print("Marker_2")
moveit_group.set_start_state(robot.get_current_state())

moveit_group.set_pose_target(target,end_effector_link)
success = moveit_group.go(wait = True)
rospy.sleep(5)

rospy.spin()
#joints_con = _Constraints()
#joints_con.name = 'goal_constrains'

#goal_joint = moveit_group.get_joints_value_target()
#elbow_con = JointConstraint()
#elbow_con.joint_name = 'elbow_joint'
#elbow_con.position = 1.745
#elbow_con.tolerance_above = 1
#elbow_con.tolerance_below = 1
