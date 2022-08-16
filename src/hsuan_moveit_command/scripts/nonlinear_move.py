#!/usr/bin/env python
from __future__ import print_function
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

def generate_nonlinear_trajectory(req):
    response=HsuanMoveServiceResponse()
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    #Set start angles.
    if len(req.hsuan_moveit_joints.start_joint_state.name)!=0:
        print("Set start_joint_state...")
        current_joint_state=JointState()
        current_joint_state=req.hsuan_moveit_joints.start_joint_state
        #current_joint_state.name=joint_names
        current_robot_state=RobotState()
        current_robot_state.joint_state=current_joint_state
        move_group.set_start_state(current_robot_state)

        print('Start angles:')
        for i in current_robot_state.joint_state.position:
            print(i,", ",end="")
        print()
    else:
        print("Don't have start_joint_angle.")
        return response

    #Set target angles or pose.
    if len(req.hsuan_moveit_joints.end_joint_state.name)!=0:
        print("Set end_joint_state...")
        move_group.set_joint_value_target(req.hsuan_moveit_joints.end_joint_state)
        print('End angles:')
        for i in req.hsuan_moveit_joints.end_joint_state.position:
            print(i,", ",end="")
        print()
    elif req.hsuan_moveit_joints.end_pose!=Pose():
        print("Set pose_target...")
        print("End Pose=",req.hsuan_moveit_joints.end_pose)
        move_group.set_pose_target(req.hsuan_moveit_joints.end_pose)
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
    plan=move_group.plan()
    print("plan=",plan)
    print("Planning trajectories success!")
    response.trajectories=planCompat(plan).joint_trajectory
    move_group.clear_pose_targets()
    return response

def server():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('nonlinear_move_server')
    srv = rospy.Service('nonlinear_move', HsuanMoveService, generate_nonlinear_trajectory)
    print("Ready to plan")
    rospy.spin()

if __name__=="__main__":
    server()