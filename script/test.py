#!/usr/bin/env python3
import csv
import os
import rospy 
import moveit_commander
import sys
import moveit_msgs.msg
from math import pi 
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import _Constraints,JointConstraint
from std_msgs.msg import Int8
import subprocess
import time


rospy.init_node("moveit_node",anonymous=True)
moveit_commander.roscpp_initialize(sys.argv)
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

Home=[0,-2*pi/3,5*pi/9,pi/9,pi/2,-pi/2]
moveit_group.go(Home,wait=True)

os.chdir(os.path.dirname(os.path.abspath(__file__)))

# n = [1, 2, 3, 4]
j = 0
while j <4:
    n = rospy.get_param('~sw')
    rospy.loginfo(n)
    print(n)
    x = n[j]
    def get_pos(x):
        print("a")
        with open('filex.csv','r') as csv_pos_file: 
            csv_reader = csv.reader(csv_pos_file)
            y=[] 
            pos=[]
            print("b")
            for line in csv_reader : #read file
                for word in line :   
                    y.append (word) #move elements to list y
            i=0
            print("c")
            for key in y : 
                i += 1
                if key == f"ID = {x}":  #search in list for ID n
                    y[i]= y[i].split()
                    pos.append(y[i][1]) #add position to new list pos
                    pos.append(y[i][3])
                    pos.append(y[i][5])
                    break        
            return pos
    print("d") 
    print(get_pos(x))
    j +=1
    print(j)
    print("e")
    target = PoseStamped()
    target.header.frame_id = 'base_link'

    target.pose.position.x = float(get_pos(x)[0]) - 0.25 + 0.015
    target.pose.position.y = float(get_pos(x)[1]) 
    target.pose.position.z = float(get_pos(x)[2])  - (0.02 + 0.03)
    print("f")
    start_pose = moveit_group.get_current_pose().pose
    target.pose.orientation.x = start_pose.orientation.x
    target.pose.orientation.y = start_pose.orientation.y
    target.pose.orientation.z = start_pose.orientation.z
    target.pose.orientation.w = start_pose.orientation.w
    print("g")
    moveit_group.set_start_state(robot.get_current_state())

    moveit_group.set_pose_target(target,end_effector_link)
    success = moveit_group.go(wait = True)
    time.sleep(1)
    
    moveit_group.go(Home,wait=True)
    print("h")
    process = subprocess.Popen("rostopic pub /gripper_command std_msgs/String 'close'", shell=True,start_new_session=True)
    time.sleep(5)
    process.terminate()
    process.wait()
    
    rospy.sleep(1)
    print("i")
    
print("Done")