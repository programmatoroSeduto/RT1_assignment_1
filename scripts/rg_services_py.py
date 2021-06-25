#!/usr/bin/env python

##
#	@file rg_services_py.py
#	@author Francesco Ganci (S4143910)
#	@brief A collection of services the controller needs. 
#	@version 1.0
#	@date 2021-06-25
#	
#	\details
# This file contains the three services the controller needs in order to work: <br>
# <ul>
# <li> <b>get target</b> randomly generate a new target </li>
# <li> <b>check target</b> check if the target is reached or not </li>
# <li> <b>get velocity</b> generate the gelocity for approaching the goal </li>
# </ul>
#	
#	\see rg_services.cpp   c++ analog implementation of this collection of services
#   \see rg_data.py   for the names of the services
#	
#	@copyright Copyright (c) 2021
#



import rospy

import math
import random
import datetime
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from robot_game.srv import rg_get_target_srv, rg_get_target_srvResponse
from robot_game.srv import rg_check_srv, rg_check_srvRequest, rg_check_srvResponse
from robot_game.srv import rg_get_vel_srv, rg_get_vel_srvRequest, rg_get_vel_srvResponse
import rg_data

srv_get_target = None
srv_check_target = None
srv_get_vel = None


## 
#	@brief Generate a new random target. 
#	
#	@param limits (robot_game/rg_get_target_srvRequest) the request
#	@return robot_game/rg_get_target_srvRsponse the new target
#
def rg_get_target_callback( limits ):
    ret = rg_get_target_srvResponse()

    ret.xt.x = limits.xmin + random.random() * (limits.xmax - limits.xmin)
    ret.xt.y = limits.ymin + random.random() * (limits.ymax - limits.ymin)
    
    return ret

## 
#	@brief Check if the distance between two points is less than a given tolerance 
#	
#	@param data (robot_game/rg_check_target_srvRequest) it contains X and Xt
#	@return robot_game/rg_check_target_srvRsponse if the orobt is near enough or not
#   
#   \see rg_data tolerance value
#   
def rg_check_target_callback( data ):
    x = data.x.x - data.xt.x
    y = data.x.y - data.xt.y

    return rg_check_srvResponse( reached=( math.sqrt(x*x + y*y) < rg_data.tolerance ) )

## 
#	@brief Check if the distance between two points is less than a given tolerance 
#	
#	@param data (robot_game/rg_get_vel_srvRequest) X and Xt
#	@return robot_game/rg_get_vel_srvResponse the Twist
#   
#   \see rg_data twist_gain value
#   
def rg_get_vel_callback( data ):
    ret = rg_get_vel_srvResponse()
    k = rg_data.twist_gain

    ret.vel.linear.x = k * ( data.xt.x - data.x.x )
    ret.vel.linear.y = k * ( data.xt.y - data.x.y )
    ret.vel.linear.z = 0.0

    ret.vel.angular.x = 0.0
    ret.vel.angular.y = 0.0
    ret.vel.angular.z = 0.0

    return ret

## 
#	@brief Called when shutdown signal is raised. 
#   
def services_on_shutdown( ):
    rospy.loginfo( "(Robot Game) Services node closed." )

if __name__ == "__main__":
    random.seed( datetime.datetime.now() )

    # Initialization of the node
    rospy.init_node( "rg_services_py" )

    # Get services
    srv_get_target = rospy.Service( rg_data.srv_name_get_target, rg_get_target_srv, rg_get_target_callback )
    srv_check_target = rospy.Service( rg_data.srv_name_check_target, rg_check_srv, rg_check_target_callback )
    srv_get_vel = rospy.Service( rg_data.srv_name_get_velocity, rg_get_vel_srv, rg_get_vel_callback )

    # spin forever
    rospy.on_shutdown( services_on_shutdown )
    rospy.loginfo( "(Robot Game) Services started! Spinning..." )
    rospy.spin( )