#!/usr/bin/env python3 

# import libraries 
import rospy
import moveit_commander
import sys
import moveit_msgs.msg
from math import pi 

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

moveit_group.set_goal_position_tolerance(0.005)
moveit_group.allow_replanning(True)
# moveit_group.set_planning_id()

#publisher
pub=rospy.Publisher("/move_group/display_planned_path",moveit_msgs.msg.DisplayTrajectory,queue_size=20)


Home=[0,-2*pi/3,5*pi/9,pi/9,pi/2,-pi/2]
moveit_group.go(Home,wait=True)



# x_list=moveit_group.get_current_joint_values()
# x_list[5]-= pi/2
# moveit_group.go(x_list,wait=True)

# print(moveit_group.get_current_pose(end_effector_link="tool0").pose.position)

wp = moveit_group.get_current_pose(end_effector_link="tool0").pose

# wp.position.x+=0.1




while(1):
    x=input()
    if x=='a':
        wp.position.x+=0.1
    elif x=='s':
        wp.position.x-=0.1    
    elif x=='d':
        wp.position.y+=0.1 
    elif x=='f':
        wp.position.y-=0.1    
    elif x=='g':
        wp.position.z+=0.1 
    elif x=='h':
        wp.position.z-=0.1  
    elif x=='q':
        break           
    (plan,_) = moveit_group.compute_cartesian_path([wp],0.01,0)
    moveit_group.execute(plan,wait=True)