#!/usr/bin/env python

##
#	@file rg_controller_py.py
#	@author Francesco Ganci (S4143910)
#	@brief A basic controller for an holonomic mobile robot. 
#	@version 1.0
#	@date 2021-06-25
#	
#	\details
#	    This ROS node implements a simple go-to-point behaviour, using linear
#	    planar twists. <br>
#       It simply reaches the actual position, <br> evaluates the 
#	    distance from it, <br> generates a linear planar twist <br> and finally sends it to the
#	    simulated environment via 'cmd_vel'.
#   
#   \see rg_controller.cpp
#	
#	@copyright Copyright (c) 2021
#

import rospy
from robot_game import rg_data
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from robot_game import srv

## Topic handler from the simulation environment (Subscriber) 'base_pose_ground_truth'
sim_input = None

## Topic handler to the simulation environment (Publisher) 'cmd_vel'
sim_output = None

## Service entry point for checking if the target is reached
srv_check_target = None

## Service entry point for requiring a nwe random target
srv_get_target = None

## Service entry point for getting a linear planar twist
srv_get_vel = None

## The actual target (geometry_msgs/Point)
xt = None

## 
#	@brief Ask the service for a new target to reach.
#
#	@return rg_get_target_srvResponse the new target
#
def get_new_target( ):
    request = srv.rg_get_target_srvRequest()
    request.xmin = -6
    request.xmax = 6
    request.ymin = -6
    request.ymax = 6
    recv = srv_get_target( request )

    rospy.loginfo( "Next target: [%f, %f]", recv.xt.x, recv.xt.y )

    return recv.xt

## 
#	@brief The main callback of the node. 
#	
#	@param pose (nav_msgs/Odometry) the actual posture of the robot
#	
#	\details
# This callback is called when a new position comes from the simulated environment via 'base_pose_ground_truth'. <br>
# It works in this way: <br>
# <ol>
# <li> get the actual pose from the received message </li>
# <li> check if the target was reached; if yes, ask for a new target </li>
# <li> generate the velocity to give the robot for approaching the goal </li>
# <li> publish the new velocity </li>
# </ol>

def rg_controller_callback( pose ):
    global xt, srv_check_target, srv_get_vel, sim_output
    
    # actual position
    x = pose.pose.pose.position

    # target reached? 
    req = srv.rg_check_srvRequest( x=x, xt=xt )
    if( ( srv_check_target( req ) ).reached ):
        rospy.loginfo( "Target Reached!" )
        xt = get_new_target()
    
    # get the velocity
    vel_data = srv_get_vel( x, xt )

    # send the new twist
    sim_output.publish( vel_data.vel )

## 
#	@brief Called when shutdown signal is raised. 
#   
def controller_on_shutdown():
    rospy.loginfo( "(Robot Game) Controller node closed." )


if __name__ == "__main__":
    rospy.init_node( "rg_controller_py" )
    rospy.on_shutdown( controller_on_shutdown )

    # input and output
    rospy.loginfo( "(Controller) subscribing to: %s", rg_data.input_topic )
    sim_input = rospy.Subscriber( rg_data.input_topic, Odometry, rg_controller_callback )
    rospy.loginfo( "(Controller) creating topic : %s", rg_data.output_topic )
    sim_output = rospy.Publisher( rg_data.output_topic, Twist, queue_size=1 )

    # services
    rospy.wait_for_service( rg_data.srv_name_check_target )
    rospy.loginfo( "(Controller) server : %s", rg_data.srv_name_check_target )
    srv_check_target = rospy.ServiceProxy( rg_data.srv_name_check_target, srv.rg_check_srv )
    
    rospy.wait_for_service( rg_data.srv_name_get_target )
    rospy.loginfo( "(Controller) server : %s", rg_data.srv_name_get_target )
    srv_get_target = rospy.ServiceProxy( rg_data.srv_name_get_target, srv.rg_get_target_srv )
    
    rospy.wait_for_service( rg_data.srv_name_get_velocity )
    rospy.loginfo( "(Controller) server : %s", rg_data.srv_name_get_velocity )
    srv_get_vel = rospy.ServiceProxy( rg_data.srv_name_get_velocity, srv.rg_get_vel_srv )

    # target init
    rospy.wait_for_service( rg_data.srv_name_get_target )
    rospy.loginfo( "(Controller) getting initial target" )
    xt = get_new_target()

    rospy.loginfo( "(Robot Game) Controller started! Spinning..." )
    rospy.spin()
