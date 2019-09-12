#!/usr/bin/env python

## Runs initialization procedures for robot.
# @ingroup integration_tests
# @file initialize_robot.py
# @namespace scripts.initialize_robot Integration test

# Copyright (c) 2017, Robot Control and Pattern Recognition Group,
# Institute of Control and Computation Engineering
# Warsaw University of Technology
#
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Warsaw University of Technology nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Author: Dawid Seredynski
#

import roslib; roslib.load_manifest('velma_task_cs_ros_interface')
import rospy
import PyKDL
import math

from velma_common import *
from rcprg_planner import *
from rcprg_ros_utils import exitError

gcomp = 10.0
gcompd = 1.1

# gcomp = 0.0
# gcompd = 0.0


if __name__ == "__main__":

    tol = PyKDL.Twist()
    tol.vel = PyKDL.Vector(100, 100, 100)
    tol.rot = PyKDL.Vector(100, 100, 100)

    rospy.init_node('test_init', anonymous=False)

    rospy.sleep(0.5)

    print "This test/tutorial executes initialization"\
        " procedures required for robot operation.\n"

    print "Running python interface for Velma..."
    velma = VelmaInterface()
    print "Waiting for VelmaInterface initialization..."
    if not velma.waitForInit(timeout_s=10.0):
        print "Could not initialize VelmaInterface\n"
        exitError(1)
    print "Initialization ok!\n"

    def close_gripper():
        half_pi = 90.0/180.0*math.pi
        dest_q = [half_pi,half_pi,half_pi,0]
        print "move right:", dest_q
        velma.moveHandRight(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
        if velma.waitForHandRight() != 0:
            exitError(8)
        rospy.sleep(0.5)

    def open_gripper():
        dest_q = [0,0,0,0]
        print "move right:", dest_q
        velma.moveHandRight(dest_q, [1,1,1,1], [2000,2000,2000,2000], 1000, hold=True)
        if velma.waitForHandRight() != 0:
            exitError(8)
        rospy.sleep(0.5)


    print "Motors must be enabled every time after the robot enters safe state."
    print "If the motors are already enabled, enabling them has no effect."
    print "Enabling motors..."
    if velma.enableMotors() != 0:
        exitError(14)

    print "Also, head motors must be homed after start-up of the robot."
    print "Sending head pan motor START_HOMING command..."
    velma.startHomingHP()
    if velma.waitForHP() != 0:
        exitError(14)
    print "Head pan motor homing successful."

    print "Sending head tilt motor START_HOMING command..."
    velma.startHomingHT()
    if velma.waitForHT() != 0:
        exitError(15)
    print "Head tilt motor homing successful."

# define some configurations
    q_map_starting = {'torso_0_joint':0,
        'right_arm_0_joint':-0.3,   'left_arm_0_joint':0.3,
        'right_arm_1_joint':-1.8,   'left_arm_1_joint':1.8,
        'right_arm_2_joint':1.25,   'left_arm_2_joint':-1.25,
        'right_arm_3_joint':0.85,   'left_arm_3_joint':-0.85,
        'right_arm_4_joint':0,      'left_arm_4_joint':0,
        'right_arm_5_joint':-0.5,   'left_arm_5_joint':0.5,
        'right_arm_6_joint':0,      'left_arm_6_joint':0 }

    q_map_1 = {'torso_0_joint':0.0,
        'right_arm_0_joint':-0.3,   'left_arm_0_joint':0.3,
        'right_arm_1_joint':-1.57,  'left_arm_1_joint':1.57,
        'right_arm_2_joint':1.57,   'left_arm_2_joint':-1.57,
        'right_arm_3_joint':1.57,   'left_arm_3_joint':-1.7,
        'right_arm_4_joint':0.0,    'left_arm_4_joint':0.0,
        'right_arm_5_joint':-1.57,  'left_arm_5_joint':1.57,
        'right_arm_6_joint':0.0,    'left_arm_6_joint':0.0 }


    print "waiting for Planner init..."
    p = Planner(velma.maxJointTrajLen())
    if not p.waitForInit():
        print "could not initialize PLanner"
        exitError(4)
    print "Planner init ok"

    # define a function for frequently used routine in this test
    def planAndExecute(q_dest):
        print "Planning motion to the goal position using set of all joints..."
        print "Moving to valid position, using planned trajectory."
        goal_constraint = qMapToConstraints(q_dest, 0.01, group=velma.getJointGroup("impedance_joints"))
        for i in range(5):
            rospy.sleep(0.5)
            js = velma.getLastJointState()
            print "Planning (try", i, ")..."
            traj = p.plan(js[1], [goal_constraint], "impedance_joints", max_velocity_scaling_factor=0.15, planner_id="RRTConnect")
            if traj == None:
                continue
            print "Executing trajectory..."
            if not velma.moveJointTraj(traj, start_time=0.5, position_tol=20000, velocity_tol=20000):
                exitError(5)
            if velma.waitForJoint() == 0:
                break
            else:
                print "The trajectory could not be completed, retrying..."
                continue
        rospy.sleep(0.5)
        # js = velma.getLastJointState()
        # if not isConfigurationClose(q_dest, js[1]):
        #     exitError(6)

    def makeWrench(lx,ly,lz,rx,ry,rz):
        return PyKDL.Wrench(PyKDL.Vector(lx,ly,lz), PyKDL.Vector(rx,ry,rz))


    open_gripper()

    print "Switch to jnt_imp mode (no trajectory)..."
    velma.moveJointImpToCurrentPos(start_time=0.2)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(7)

    rospy.sleep(0.5)
    diag = velma.getCoreCsDiag()
    if not diag.inStateJntImp():
        print "The core_cs should be in jnt_imp state, but it is not"
        exitError(8)

    print "Checking if the starting configuration is as expected..."
    rospy.sleep(0.5)
    js = velma.getLastJointState()
    # if not isConfigurationClose(q_map_starting, js[1], tolerance=0.2):
    #     print "This test requires starting pose:"
    #     print q_map_starting
    #     exitError(9)

    # get initial configuration
    js_init = velma.getLastJointState()

    planAndExecute(q_map_1)


    print "Switch to cart_imp mode (no trajectory)..."
    if not velma.moveCartImpRightCurrentPos(start_time=0.2):
        exitError(10)
    if velma.waitForEffectorRight() != 0:
        exitError(11)

    rospy.sleep(0.5)

    diag = velma.getCoreCsDiag()
    if not diag.inStateCartImp():
        print "The core_cs should be in cart_imp state, but it is not"
        exitError(12)

    print "reset left"
    velma.resetHandLeft()
    if velma.waitForHandLeft() != 0:
        exitError(2)
    rospy.sleep(0.5)
    if not isHandConfigurationClose( velma.getHandLeftCurrentConfiguration(), [0,0,0,0]):
        exitError(3)

    print "reset right"
    velma.resetHandRight()
    if velma.waitForHandRight() != 0:
        exitError(4)
    rospy.sleep(0.5)

    open_gripper()



    # imp_list = [makeWrench(15,15,15,15,15,15)]
    imp_list = [makeWrench(150,150,150,150,150,150)]
    T_B_Trd = PyKDL.Frame(PyKDL.Rotation.Quaternion( 0.0 , 0.0 , 0.0 , 1.0 ), PyKDL.Vector( 0.45 , -0.4 , 1.10  ))
    print "Set impedance to (1000,1000,125,150,150,150) in tool frame."
    if not velma.moveCartImpRight([T_B_Trd], [5.0], None, None, imp_list, [0.0], PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5, path_tol=tol, gravity_compensation_i=gcomp, gravity_compensation_d=gcompd):
        exitError(16)
    if velma.waitForEffectorRight() != 0:
        exitError(17)

    rospy.sleep(2.0)

    exit(1)

    T_B_Trd = PyKDL.Frame(PyKDL.Rotation.Quaternion( 0.0 , 0.0 , 0.0 , 1.0 ), PyKDL.Vector( 0.55 , -0.4 , 1.10  ))
    print "Set impedance to (1000,1000,125,150,150,150) in tool frame."
    if not velma.moveCartImpRight([T_B_Trd], [5.0], None, None, imp_list, [0.0], PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5, path_tol=tol, gravity_compensation_i=gcomp, gravity_compensation_d=gcompd):
        exitError(16)
    if velma.waitForEffectorRight() != 0:
        exitError(17)

    rospy.sleep(5.0)




    close_gripper()

    rospy.sleep(5)


    T_B_Trd = PyKDL.Frame(PyKDL.Rotation.Quaternion( 0.0 , 0.0 , 0.0 , 1.0 ), PyKDL.Vector( 0.55 , -0.4 , 1.35  ))
    print "Set impedance to (1000,1000,125,150,150,150) in tool frame."
    if not velma.moveCartImpRight([T_B_Trd], [5.0], None, None, imp_list, [0.0], PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5, path_tol=tol, gravity_compensation_i=gcomp, gravity_compensation_d=gcompd):
        exitError(16)
    if velma.waitForEffectorRight() != 0:
        exitError(17)

    rospy.sleep(25.0)



    # if not isHandConfigurationClose( velma.getHandRightCurrentConfiguration(), dest_q):
    #     exitError(9)


    t = 0
    a = 0.3
    y = (a*math.sqrt(2)*math.cos(t))/(math.sin(t)**2+1)
    z = y*math.sin(t)
    T_B_Trd = PyKDL.Frame(PyKDL.Rotation.Quaternion( 0.0, 0, 0.0 , 1.0 ), PyKDL.Vector( 0.55, -0.4 +y*0.7, 1.5 +z -0.1))
    if not velma.moveCartImpRight([T_B_Trd], [5.0], None, None, imp_list, [0.0], PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5, path_tol=tol, gravity_compensation_i=gcomp, gravity_compensation_d=gcompd):
        exitError(16)
    if velma.waitForEffectorRight() != 0:
        exitError(17)

    rospy.sleep(25.0)

    # generate leminiscate of bernoulli
    t = 0
    frame_list = []
    time_list = []
    time = 1.0
    while t <= math.pi*2:
        y = (a*math.sqrt(2)*math.cos(t))/(math.sin(t)**2+1)
        z = y*math.sin(t)
        t += 0.2
        time += 1.2

        T_B_Trd = PyKDL.Frame(PyKDL.Rotation.Quaternion( 0.0 , 0.0, 0.0 , 1.0 ), PyKDL.Vector( 0.55, -0.4+y*0.7, 1.5+z-0.1))
        frame_list.append(T_B_Trd)
        time_list.append(time)


    print "osemka"
    print frame_list
    print time_list


    if not velma.moveCartImpRight(frame_list, time_list, None, None, imp_list, [0.0], PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.0, path_tol=tol, gravity_compensation_i=gcomp, gravity_compensation_d=gcompd):
        exitError(16)
    if velma.waitForEffectorRight() != 0:
        exitError(17)


    rospy.sleep(20)


    #ruch do pozycji przed osemka, lekko obnizona

    t = 0
    a = 0.3
    y = (a*math.sqrt(2)*math.cos(t))/(math.sin(t)**2+1)
    z = y*math.sin(t)
    T_B_Trd = PyKDL.Frame(PyKDL.Rotation.Quaternion( 0.0, 0, 0.0 , 1.0 ), PyKDL.Vector( 0.55, -0.6 +y*0.7 -0.1, 1.6+z-0.1-0.1 ))
    if not velma.moveCartImpRight([T_B_Trd], [4.0], None, None, imp_list, [0.0], PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5, path_tol=tol, gravity_compensation_i=gcomp, gravity_compensation_d=gcompd):
        exitError(16)
    if velma.waitForEffectorRight() != 0:
        exitError(17)

    rospy.sleep(20.0)


    # ruch do gory
    t = 0
    a = 0.3
    y = (a*math.sqrt(2)*math.cos(t))/(math.sin(t)**2+1)
    z = y*math.sin(t)
    T_B_Trd = PyKDL.Frame(PyKDL.Rotation.Quaternion( 0.0, 0, 0.0 , 1.0 ), PyKDL.Vector( 0.55, -0.6 +y*0.7-0.1, 1.6 +z + 0.1-0.1 ))
    print "Set impedance to (1000,1000,125,150,150,150) in tool frame."
    if not velma.moveCartImpRight([T_B_Trd], [3.0], None, None, imp_list, [0.0], PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5, path_tol=tol, gravity_compensation_i=gcomp, gravity_compensation_d=gcompd):
        exitError(16)
    if velma.waitForEffectorRight() != 0:
        exitError(17)

    rospy.sleep(25.0)


    # ruch do pozycji przed osemka, lekko obnizona
    t = 0
    a = 0.3
    y = (a*math.sqrt(2)*math.cos(t))/(math.sin(t)**2+1)
    z = y*math.sin(t)
    T_B_Trd = PyKDL.Frame(PyKDL.Rotation.Quaternion( 0.0, 0, 0.0 , 1.0 ), PyKDL.Vector( 0.55, -0.6 +y*0.7-0.1, 1.6 +z-0.1-0.1))
    print "Set impedance to (1000,1000,125,150,150,150) in tool frame."
    if not velma.moveCartImpRight([T_B_Trd], [3.0], None, None, imp_list, [0.0], PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5, path_tol=tol, gravity_compensation_i=gcomp, gravity_compensation_d=gcompd):
        exitError(16)
    if velma.waitForEffectorRight() != 0:
        exitError(17)

    rospy.sleep(20.0)



    # ruch do przodu

    t = 0
    a = 0.3
    y = (a*math.sqrt(2)*math.cos(t))/(math.sin(t)**2+1)
    z = y*math.sin(t)
    T_B_Trd = PyKDL.Frame(PyKDL.Rotation.Quaternion( 0.0, 0, 0.0 , 1.0 ), PyKDL.Vector( 0.55 + 0.2, -0.6 +y*0.7-0.1, 1.2 +z + 0.3-0.1))
    print "Set impedance to (1000,1000,125,150,150,150) in tool frame."
    if not velma.moveCartImpRight([T_B_Trd], [3.0], None, None, imp_list, [0.0], PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5, path_tol=tol, gravity_compensation_i=gcomp, gravity_compensation_d=gcompd):
        exitError(16)
    if velma.waitForEffectorRight() != 0:
        exitError(17)

    rospy.sleep(20.0)

    #ruch od pozycji przed osemka
    t = 0
    a = 0.3
    y = (a*math.sqrt(2)*math.cos(t))/(math.sin(t)**2+1)
    z = y*math.sin(t)
    T_B_Trd = PyKDL.Frame(PyKDL.Rotation.Quaternion( 0.0, 0, 0.0 , 1.0 ), PyKDL.Vector( 0.55, -0.6 +y*0.7-0.1, 1.2 +z + 0.3-0.1))
    print "Set impedance to (1000,1000,125,150,150,150) in tool frame."
    if not velma.moveCartImpRight([T_B_Trd], [3.0], None, None, imp_list, [0.0], PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5, path_tol=tol, gravity_compensation_i=gcomp, gravity_compensation_d=gcompd):
        exitError(16)
    if velma.waitForEffectorRight() != 0:
        exitError(17)

    rospy.sleep(20.0)



    #ruch  w bok
    t = 0
    a = 0.3
    y = (a*math.sqrt(2)*math.cos(t))/(math.sin(t)**2+1)
    z = y*math.sin(t)
    T_B_Trd = PyKDL.Frame(PyKDL.Rotation.Quaternion( 0.0, 0, 0.0 , 1.0 ), PyKDL.Vector( 0.55, -0.6 +y*0.7 -0.2-0.1, 1.2 +z + 0.3-0.1))
    print "Set impedance to (1000,1000,125,150,150,150) in tool frame."
    if not velma.moveCartImpRight([T_B_Trd], [3.0], None, None, imp_list, [0.0], PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5, path_tol=tol, gravity_compensation_i=gcomp, gravity_compensation_d=gcompd):
        exitError(16)
    if velma.waitForEffectorRight() != 0:
        exitError(17)

    rospy.sleep(20.0)


    #ruch do pozycji przed osemka
    t = 0
    a = 0.3
    y = (a*math.sqrt(2)*math.cos(t))/(math.sin(t)**2+1)
    z = y*math.sin(t)
    T_B_Trd = PyKDL.Frame(PyKDL.Rotation.Quaternion( 0.0, 0, 0.0 , 1.0 ), PyKDL.Vector( 0.55, -0.6 +y*0.7-0.1, 1.2 +z + 0.3-0.1))
    print "Set impedance to (1000,1000,125,150,150,150) in tool frame."
    if not velma.moveCartImpRight([T_B_Trd], [3.0], None, None, imp_list, [0.0], PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5, path_tol=tol, gravity_compensation_i=gcomp, gravity_compensation_d=gcompd):
        exitError(16)
    if velma.waitForEffectorRight() != 0:
        exitError(17)

    rospy.sleep(20.0)

    #ruch do pozycji obroconej

    t = 0
    a = 0.3
    y = (a*math.sqrt(2)*math.cos(t))/(math.sin(t)**2+1)
    z = y*math.sin(t)
    T_B_Trd = PyKDL.Frame(PyKDL.Rotation.Quaternion( -0.2397128, -0.0612087, -0.2397128, 0.9387913 ), PyKDL.Vector( 0.55, -0.6 +y*0.7-0.1, 1.2 +z + 0.3-0.1))
    if not velma.moveCartImpRight([T_B_Trd], [3.0], None, None, imp_list, [0.0], PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5, path_tol=tol, gravity_compensation_i=gcomp, gravity_compensation_d=gcompd):
        exitError(16)
    if velma.waitForEffectorRight() != 0:
        exitError(17)

    rospy.sleep(20.0)

    #ruch do pozycji przed osemka

    t = 0
    a = 0.3
    y = (a*math.sqrt(2)*math.cos(t))/(math.sin(t)**2+1)
    z = y*math.sin(t)
    T_B_Trd = PyKDL.Frame(PyKDL.Rotation.Quaternion( 0, 0, 0.0 , 1.0 ), PyKDL.Vector( 0.55, -0.6 +y*0.7-0.1, 1.2 +z + 0.3-0.1))
    print "Set impedance to (1000,1000,125,150,150,150) in tool frame."
    if not velma.moveCartImpRight([T_B_Trd], [3.0], None, None, imp_list, [0.0], PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5, path_tol=tol, gravity_compensation_i=gcomp, gravity_compensation_d=gcompd):
        exitError(16)
    if velma.waitForEffectorRight() != 0:
        exitError(17)

    rospy.sleep(20.0)



    #ruch do pozycji obroconej w gorze

    t = 0
    a = 0.3
    y = (a*math.sqrt(2)*math.cos(t))/(math.sin(t)**2+1)
    z = y*math.sin(t)
    T_B_Trd = PyKDL.Frame(PyKDL.Rotation.Quaternion( -0.2397128, -0.0612087, -0.2397128, 0.9387913 ), PyKDL.Vector( 0.40, -0.8+y*0.7-0.1, 1.2 +z + 0.6-0.1))
    if not velma.moveCartImpRight([T_B_Trd], [4.0], None, None, imp_list, [0.0], PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5, path_tol=tol, gravity_compensation_i=gcomp, gravity_compensation_d=gcompd):
        exitError(16)
    if velma.waitForEffectorRight() != 0:
        exitError(17)

    rospy.sleep(20.0)


    t = 0
    a = 0.3
    y = (a*math.sqrt(2)*math.cos(t))/(math.sin(t)**2+1)
    z = y*math.sin(t)
    T_B_Trd = PyKDL.Frame(PyKDL.Rotation.Quaternion( 0, 0, 0.0 , 1.0 ), PyKDL.Vector( 0.55, -0.6 +y*0.7-0.1, 1.2 +z + 0.3-0.1 ))
    if not velma.moveCartImpRight([T_B_Trd], [4.0], None, None, imp_list, [0.0], PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5, path_tol=tol, gravity_compensation_i=gcomp, gravity_compensation_d=gcompd):
        exitError(16)
    if velma.waitForEffectorRight() != 0:
        exitError(17)

    rospy.sleep(20.0)


    #kolizja


    t = 0
    a = 0.3
    y = (a*math.sqrt(2)*math.cos(t))/(math.sin(t)**2+1)
    z = y*math.sin(t)
    T_B_Trd = PyKDL.Frame(PyKDL.Rotation.Quaternion( 0, 0, 0.0 , 1.0 ), PyKDL.Vector( 0.6, -0.6 +y*0.7-0.1,  1.2 +z -0.5))
    print "Set impedance to (1000,1000,125,150,150,150) in tool frame."
    if not velma.moveCartImpRight([T_B_Trd], [5.0], None, None, imp_list, [0.0], PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5, path_tol=tol, gravity_compensation_i=gcomp, gravity_compensation_d=gcompd):
        exitError(16)
    if velma.waitForEffectorRight() != 0:
        exitError(17)

    rospy.sleep(10.0)

    exitError(0)
