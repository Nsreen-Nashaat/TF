#!/usr/bin/env python3 

# import libraries 
import rospy
import moveit_commander
import sys
import moveit_msgs.msg
from math import pi 
from sensor_msgs.msg import Joy
from std_msgs.msg import Int8

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


wp = moveit_group.get_current_pose(end_effector_link="tool0").pose


def cb (msg):
    if msg.axes[1]== 1:       #forward
        wp.position.x+=0.1
    elif msg.axes[1]== -1:    #back
        wp.position.x-=0.1    
    elif msg.axes[0] == 1:    #right
        wp.position.y+=0.1 
    elif msg.axes[0] == -1:   #left
        wp.position.y-=0.1    
    elif msg.buttons[0] == 1:    #UP
        wp.position.z+=0.1 
    elif msg.buttons[2] == 1:   #Down
        wp.position.z-=0.1  
    

    (plan,_) = moveit_group.compute_cartesian_path([wp],0.01,0)
    moveit_group.execute(plan,wait=True) 
        
def joy_manual():
	rospy.Subscriber("joy",Joy,cb)
	rospy.spin()

if __name__ == '__main__':
	try:
		joy_manual()
	except rospy.ROSInterruptException:
		pass