#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ##
# @brief    Project developed for Robot Control Systems, robot controlled by keyboard
# @author   Nagy Timea (Nagy.Io.Timea@student.utcluj.ro)  

import rospy
import os
import curses
import threading, time
import sys
import math
import numpy as np
#import keyboard
sys.dont_write_bytecode = True
sys.path.append( os.path.abspath(os.path.join(os.path.dirname(__file__),"../../../../common/imp")) ) # get import path : DSR_ROBOT.py 

# for single robot 
ROBOT_ID     = "dsr01"
ROBOT_MODEL  = "m1013"
import DR_init
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
from DSR_ROBOT import *

def shutdown():
    print("shutdown time!")
    pub_stop.publish(stop_mode=STOP_TYPE_QUICK)
    return 0

def keyboard_control(stdscr):
    stdscr.clear()
    stdscr.addstr(0, 0, "Control X(red)   coordinate: Q, W")
    stdscr.addstr(1, 0, "Control Y(green) coordinate: A, S")
    stdscr.addstr(2, 0, "Control Z(blue)  coordinate: Z, X")
    stdscr.addstr(3, 0, "Close: C")

    current_position = [0.0, -12.6, 101.1, 0.0, 91.5, -0.0]
    initial_position = [0.0, -12.6, 101.1, 0.0, 91.5, -0.0]
    #joint_positions = list(ikin(initial_position_cartesian, sol_space=2))

    # Convert the list to a string for display
    # joint_positions_str = ", ".join(map(str, joint_positions))
    # stdscr.addstr(3, 0, "Joint positions: {}".format(joint_positions_str))

    velx = 1
    accx = 1
    t = 2
    movej(initial_position, velx, accx, t=t)
    current_position = fkin(initial_position,  DR_WORLD)

    while True:
        key = stdscr.getch()
        if key == ord('z'):
            print("Z")
            current_position = [current_position[0], current_position[1], current_position[2] - 100, current_position[3], current_position[4], current_position[5]]
            joint_positions = list(ikin(current_position, sol_space=2))
            movej(joint_positions, velx, accx, t=t)
        elif key == ord('x'):
            print("X")
            current_position = [current_position[0], current_position[1], current_position[2] + 100, current_position[3], current_position[4], current_position[5]]
            joint_positions = list(ikin(current_position, sol_space=2))
            movej(joint_positions, velx, accx, t=t)
        elif key == ord('a'):
            print("A")
            current_position = [current_position[0], current_position[1] - 100, current_position[2], current_position[3], current_position[4], current_position[5]]
            joint_positions = list(ikin(current_position, sol_space=2))
            movej(joint_positions, velx, accx, t=t)
        elif key == ord('s'):
            print("S")
            current_position = [current_position[0], current_position[1] + 100, current_position[2], current_position[3], current_position[4], current_position[5]]
            joint_positions = list(ikin(current_position, sol_space=2))
            movej(joint_positions, velx, accx, t=t)
        elif key == ord('q'):
            print("Q")
            current_position = [current_position[0] - 100, current_position[1], current_position[2], current_position[3], current_position[4], current_position[5]]
            joint_positions = list(ikin(current_position, sol_space=2))
            movej(joint_positions, velx, accx, t=t)
        elif key == ord('w'):
            print("W")
            current_position = [current_position[0] + 100, current_position[1], current_position[2], current_position[3], current_position[4], current_position[5]]
            joint_positions = list(ikin(current_position, sol_space=2))
            movej(joint_positions, velx, accx, t=t)
        elif key == ord('r'):
            print("Reset position")
            movel(initial_position, velx, accx, t=t)
        elif key == ord('c'):
            curses.endwin()
            break

    
def msgRobotState_cb(msg):
    msgRobotState_cb.count += 1

    if (0==(msgRobotState_cb.count % 100)): 
        rospy.loginfo("________ ROBOT STATUS ________")
        print("  robot_state       : %d" % (msg.robot_state))
        print("  robot_state_str   : %s" % (msg.robot_state_str))
        print("  current_posj      : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.current_posj[0],msg.current_posj[1],msg.current_posj[2],msg.current_posj[3],msg.current_posj[4],msg.current_posj[5]))
        #print("  current_posx      : %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f" % (msg.current_posx[0],msg.current_posx[1],msg.current_posx[2],msg.current_posx[3],msg.current_posx[4],msg.current_posx[5]))
        #print("  io_control_box    : %d" % (msg.io_control_box))
        #print("  io_modbus         : %d" % (msg.io_modbus))
        #print("  error             : %d" % (msg.error))
        #print("  access_control    : %d" % (msg.access_control))
        #print("  homming_completed : %d" % (msg.homming_completed))
        #print("  tp_initialized    : %d" % (msg.tp_initialized))
        #print("  speed             : %d" % (msg.speed))
        #print("  mastering_need    : %d" % (msg.mastering_need))
        #print("  drl_stopped       : %d" % (msg.drl_stopped))
        #print("  disconnected      : %d" % (msg.disconnected))
msgRobotState_cb.count = 0

def thread_subscriber():
    rospy.Subscriber('/'+ROBOT_ID +ROBOT_MODEL+'/state', RobotState, msgRobotState_cb)
    rospy.spin()
    #rospy.spinner(2)  

def move_circle_trajectory(center, radius, num_points, velx, accx, t):
    theta_values = np.linspace(0, 2 * math.pi, num_points)
    
    # for theta in theta_values:
        
    #     # Modify the coordinates accordingly
    #     cartesian_point = posx(
    #         center[0] + radius * math.cos(theta),
    #         center[1] + radius * math.sin(theta),
    #         float(center[2]),
    #         0.0,
    #         0.0,
    #         0.0
    #     )

    #     print("Cartesian Point:", cartesian_point)
        
    #     # Get the joint positions
    #     joint_positions = list(ikin(cartesian_point, sol_space=6))
        
    #     print("Joint positions:", joint_positions)

    #     # Move the robot
    #     movej(joint_positions, v=velx, a=accx, t=t, radius=radius, mod=DR_MV_MOD_ABS)


if __name__ == "__main__":
    _srv_name_prefix = "/dsr01m1013"
    _ros_get_current_tool_flange_posx = rospy.ServiceProxy(_srv_name_prefix + "/aux_control/get_current_tool_flange_posx", GetCurrentToolFlangePosx)
    threading.Thread(target=curses.wrapper, args=(keyboard_control,)).start()
    #move_circle_trajectory(center=[0, 0, 0], radius=100, num_points=100, velx=10, accx=10, t=2)

    rospy.init_node('dsr_simple_test_py')
    rospy.on_shutdown(shutdown)

    pub_stop = rospy.Publisher('/'+ROBOT_ID +ROBOT_MODEL+'/stop', RobotStop, queue_size=10)           

    set_velx(30,20)  # set global task speed: 30(mm/sec), 20(deg/sec)
    set_accx(60,40)  # set global task accel: 60(mm/sec2), 40(deg/sec2)