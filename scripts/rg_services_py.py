#!/usr/bin/env python

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



def rg_get_target_callback( limits ):
    ret = rg_get_target_srvResponse()

    ret.xt.x = limits.xmin + random.random() * (limits.xmax - limits.xmin)
    ret.xt.y = limits.ymin + random.random() * (limits.ymax - limits.ymin)
    
    return ret


def rg_check_target_callback( data ):
    x = data.x.x - data.xt.x
    y = data.x.y - data.xt.y

    return rg_check_srvResponse( reached=( math.sqrt(x*x + y*y) < rg_data.tolerance ) )


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