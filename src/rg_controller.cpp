
/**
 * @file rg_controller.cpp
 * @author Francesco Ganci (S4143910)
 * @brief C++ Controller for a smallholonomic mobile robot. 
 * @version 1.0
 * @date 2021-06-25
 * 
 * @details 
 * This node implements a go-to-point behaviour using a very simple approach. <br>
 * The controller listen to the topic 'base_pose_ground_truth'. When a new update about
 * the posture comes, the controller generates a linear planar twist straight to the point,
 * trying and approching to it. <br>
 * When the target is "reached" (i.e. the robot is close enough to the target location), a 
 * new randomly generated target is required to a dedicated service. 
 * 
 * @copyright Copyright (c) 2021
 * 
 * @see rg_controller_py.py the same controller, implemented in Python. 
 * 
 */

#include "ros/ros.h"
#include "ros/console.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

#include "robot_game/rg_get_target_srv.h"
#include "robot_game/rg_check_srv.h"
#include "robot_game/rg_get_vel_srv.h"

/** x min bound */
#define XMIN (-6.f)

/** x max bound */
#define XMAX (6.f)

/** y min bound */
#define YMIN (-6.f)

/** y max bound */
#define YMAX (6.f)

/** subscription to base_pose_ground_truth (nav_msgs/Odometry) */
ros::Subscriber in;	

/** publisher to cmd_vel (geometry_msgs/Twist) */
ros::Publisher out;	

ros::ServiceClient check_tg_srv;
ros::ServiceClient newtg_srv;
ros::ServiceClient get_vel_srv;

/** the actual target */
geometry_msgs::Point target;




/**
 * @brief require a new target to reach to the server. 
 * 
 * @param srv the service
 * @param xmin x min bound
 * @param xmax x max bound
 * @param ymin y min bound
 * @param ymax y max bound
 * @return geometry_msgs::Point The new target to reach
 */
geometry_msgs::Point rg_get_new_target( ros::ServiceClient srv, double xmin, double xmax, double ymin, double ymax )
{
	// ask for a new target (rg_get_target)
	robot_game::rg_get_target_srv newtrg_msg;
	newtrg_msg.request.xmin = xmin;
	newtrg_msg.request.xmax = xmax;
	newtrg_msg.request.ymin = ymin;
	newtrg_msg.request.ymax = ymax;
	
	//newtg_srv.call( newtrg_msg );
	srv.call( newtrg_msg );
	
	return newtrg_msg.response.xt;
}



/**
 * @brief The main function of the controller
 * 
 * @param posemsg the actual posture received by the simulator
 * 
 * @details 
 * This callback is called when a new position comes from the simulated environment via 'base_pose_ground_truth'. <br>
 * It works in this way: <br>
 * <ol>
 * <li> get the actual pose from the received message </li>
 * <li> check if the target was reached; if yes, ask for a new target </li>
 * <li> generate the velocity to give the robot for approaching the goal </li>
 * <li> publish the new velocity </li>
 * </ol>
 */
void rg_controller_task( const nav_msgs::Odometry::ConstPtr& posemsg )
{
	// check if the target was reached (rg_check_target)
	robot_game::rg_check_srv checkmsg;
	checkmsg.request.x.x = posemsg->pose.pose.position.x;
	checkmsg.request.x.y = posemsg->pose.pose.position.y;
	checkmsg.request.x.z = posemsg->pose.pose.position.z;
	checkmsg.request.xt = target;
	bool targetReached = check_tg_srv.call( checkmsg );
	
	if( targetReached )
		target = rg_get_new_target( newtg_srv, XMIN, XMAX, YMIN, YMAX );
	
	// ask a new Twist, then move the robot (rg_get_velocity)
	robot_game::rg_get_vel_srv vel_msg;
	vel_msg.request.x = checkmsg.request.x;
	vel_msg.request.xt = target;
	
	get_vel_srv.call( vel_msg );
	
	out.publish( vel_msg.response.vel );
	ros::spinOnce();
}


/**
 * @brief Ask for services, topics, and initialise the node. 
 */
int main( int argc, char** argv )
{
	ros::init( argc, argv, "rg_controller" );
	ros::NodeHandle h;
	
	check_tg_srv = h.serviceClient<robot_game::rg_check_srv>("rg_check_target");
	newtg_srv = h.serviceClient<robot_game::rg_get_target_srv>("rg_get_target");
	get_vel_srv = h.serviceClient<robot_game::rg_get_vel_srv>("rg_get_velocity");
	
	target = rg_get_new_target( newtg_srv, XMIN, XMAX, YMIN, YMAX );
	
	in = h.subscribe( "base_pose_ground_truth", 1000, rg_controller_task );
	out = h.advertise<geometry_msgs::Twist>( "cmd_vel", 1000 );
	
	ros::spin();
	
	return 0;
}
