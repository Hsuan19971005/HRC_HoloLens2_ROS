#!/usr/bin/env python
from __future__ import print_function
import imp
import re
import rospy, sys
from moveit_msgs.msg import RobotTrajectory
from std_msgs.msg import Bool
import moveit_commander
from hsuan_moveit_command.srv import TrajectoryExecutionService, TrajectoryExecutionServiceRequest, TrajectoryExecutionServiceResponse
joint_names=['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

def execute_trajectory(req):
    response=TrajectoryExecutionServiceResponse()
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    #Set speed
    speed_factor=1
    if req.speed_factor==None: 
        print('req.speed_factor = None')
        speed_factor=1
    elif speed_factor<0.01 or speed_factor >1:
        speed_factor=1
    else:
        speed_factor=req.speed_factor
    move_group.set_max_velocity_scaling_factor(speed_factor)
    #execute trajectories
    robot_traj=RobotTrajectory()
    robot_traj.joint_trajectory=req.trajectories
    move_group.execute(robot_traj)
    print("Exevute Traj success!")
    #return response
    complete=Bool(True)
    response.complete_execution=complete
    return response

def server():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('trajectory_execution_server')
    srv = rospy.Service('trajectory_execution', TrajectoryExecutionService, execute_trajectory)
    print("Ready to execute trajectories")
    rospy.spin()

if __name__=="__main__":
    server()