#!/usr/bin/env python

##  
#   @package rg_data
#   
#   \file rg_data.py
#   \brief A simple collection of global constants used by the python components of the program.
#   
#   \author Francesco Ganci (S4143910)
#   \version 1.0
#   
#   \details
#       This file, imported by the python scripts within this project, simply contains
#       some global constants used for avoiding hard-coding of the strings as well as 
#       making easier the debugging. <br>
#       This is not a ROS node, but just a module on duty.

## Tolerance, i.e. minimum distance the robot must have to assume it reached. 
tolerance = 0.1

## Used for computing the linear output twist
twist_gain = 1.0

## Name of the topic "base_pose_ground_truth" as input
input_topic = "base_pose_ground_truth"

## name of the topic 'cmd_vel' as output 
output_topic = "cmd_vel"

## name of the service 'rg_check_target'
srv_name_check_target = "rg_check_target"

## name of the service 'rg_get_target'
srv_name_get_target = "rg_get_target"

## name of the service 'rg_get_vel'
srv_name_get_velocity = "rg_get_vel"