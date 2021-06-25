#!/usr/bin/env python

import rospy
import rg_data
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from robot_game import srv

sim_input = None
sim_output = None

srv_check_target = None
srv_get_target = None
srv_get_vel = None

xt = None


def get_new_target( ):
    request = srv.rg_get_target_srvRequest()
    request.xmin = -6
    request.xmax = 6
    request.ymin = -6
    request.ymax = 6
    recv = srv_get_target( request )

    rospy.loginfo( "Next target: [%f, %f]", recv.xt.x, recv.xt.y )

    return recv.xt


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