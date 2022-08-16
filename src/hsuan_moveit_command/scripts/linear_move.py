#!/usr/bin/env python
from __future__ import print_function
from shutil import move
import rospy, sys
import moveit_commander
from hsuan_moveit_command.msg import HsuanMoveitJoints
from hsuan_moveit_command.srv import HsuanMoveService, HsuanMoveServiceRequest, HsuanMoveServiceResponse
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState
from geometry_msgs.msg import PoseStamped, Pose
#joint_names=['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

if sys.version_info >= (3, 0):
    def planCompat(plan):
        return plan[1]
else:
    def planCompat(plan):
        return plan

def generate_linear_trajectory(req):
    response=HsuanMoveServiceResponse()
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    waypoints=[]
    #Set start angles.
    if len(req.hsuan_moveit_joints.start_joint_state.name)!=0:
        print("Set start_joint_state...")
        current_joint_state=JointState()
        current_joint_state=req.hsuan_moveit_joints.start_joint_state
        #current_joint_state.name=joint_names
        current_robot_state=RobotState()
        current_robot_state.joint_state=current_joint_state
        move_group.set_start_state(current_robot_state)
    else:
        print("Don't have start_joint_angle.")
        return response
    #Set target angles or pose.
    if req.hsuan_moveit_joints.end_pose!=Pose():
        print("Set waypoints...")
        waypoints.append(req.hsuan_moveit_joints.end_pose)
    else:
        print("Don have end_joint or end_pose.")
        return response
    #Set velocity factor
    speed_factor=1
    if req.hsuan_moveit_joints.speed_factor==None: 
        print('req.speed_factor = None')
        speed_factor=1
    elif speed_factor<0.01 or speed_factor >1:
        speed_factor=1
    else:
        speed_factor=req.hsuan_moveit_joints.speed_factor
    move_group.set_max_velocity_scaling_factor(speed_factor)
    #generate trajectories
    fraction=0.0
    maxtries=100
    attempts=0
    while fraction<1.0 and attempts < maxtries:
        (plan,fraction)=move_group.compute_cartesian_path(waypoints,0.01,0,False)
        attempts+=1
        if attempts%10 == 1.0:
            print("Still trying after "+str(attempts)+" attempts...")
    if fraction==1.0:
        print("Path computed successfully.")
        response.trajectories=planCompat(plan).joint_trajectory
        move_group.clear_pose_targets()
    else:
        print("Path planning failed with only {fraction} success after {maxtries} attempts.")
    
    #Shut down moveit and leave
    #moveit_commander.roscpp_shutdown()
    #moveit_commander.os._exit(0)
    return response

def server():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('linear_move_server')
    srv = rospy.Service('linear_move', HsuanMoveService, generate_linear_trajectory)
    print("Ready to plan")
    rospy.spin()

if __name__=="__main__":
    server()