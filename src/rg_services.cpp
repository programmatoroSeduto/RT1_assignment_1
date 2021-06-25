
/**
 * @file rg_services.cpp
 * @author Francesco Ganci (S4143910)
 * @brief A collection of services for the controller. 
 * @version 1.0
 * @date 2021-06-25
 * 
 * @details 
 * This file contains the three services the controller needs in order to work: <br>
 * <ul>
 * <li> <b>get target</b> randomly generate a new target </li>
 * <li> <b>check target</b> check if the target is reached or not </li>
 * <li> <b>get velocity</b> generate the gelocity for approaching the goal </li>
 * </ul>
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "ros/ros.h"
#include "ros/console.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include "robot_game/rg_check_srv.h"
#include "robot_game/rg_get_target_srv.h"
#include "robot_game/rg_get_vel_srv.h"

#include <cmath>
#include <cstdlib>
#include <ctime>

/** minimum distance from the target for considering it reached */
#define TOLERANCE 0.1f

/** the speed at which the robot reaches its position. */
#define K (1.f)





/**
 * @brief Eucledian distance between two points
 * 
 * @param P1 the start point of the vector
 * @param P2 the final point of the vector
 * @return double the eucledian distance of (P2-P1)
 */
double compute_eucledian_distance( geometry_msgs::Point P1, geometry_msgs::Point P2 )
{
	return sqrt( pow(P2.x-P1.x, 2.0) + pow(P2.y-P1.y, 2.0) + pow(P2.z-P1.z, 2.0) );
}

/**
 * @brief Generate a random value within a given range.
 * 
 * @param min the minimum number (included)
 * @param max the maximum number (include)
 * @return a random number in [min, max]
 * 
 */
double rand_range( double min, double max )
{
	return min + (max - min)*( ((double)rand()) / RAND_MAX );
}







/**
 * @brief Service - check target
 * 
 * @param initpos X and Xt
 * @param r is the target reached?
 * @return true if the lenght of (X - Xt) is less than the TOLERANCE
 * @return false if the distance is greater than the TOLERANCE
 * 
 * @details 
 * The function simply takes the distance between actual position and target,
 * and compares this with the TOLERACE. If the distance is less than the TOLERANCE, 
 * the target is considered to be reached. 
 */
bool rg_check_target_callback( robot_game::rg_check_srv::Request &initpos, robot_game::rg_check_srv::Response &r )
{
	//ROS_INFO( "rg_check_target_callback( X(%f, %f, %f) Xt(%f, %f, %f) )", initpos.x.x, initpos.x.y, initpos.x.z, initpos.xt.x, initpos.xt.y, initpos.xt.z );
	
	double dist = compute_eucledian_distance( initpos.x, initpos.xt );
	//ROS_INFO( "computed distance: %f", dist );
	
	return ( dist <= 0.1f );
}

/**
 * @brief Service - get a new random target
 * 
 * @param limits bounds within to take the target
 * @param target the respose of the server, the target 
 */
bool rg_get_target_callback( robot_game::rg_get_target_srv::Request &limits, robot_game::rg_get_target_srv::Response &target )
{
	/*
	 * Content of the request:
	 * xmin, xmax
	 * ymin, ymsx
	 * 
	 * Point: x, y, z
	 */
	 
	 //ROS_INFO( "rg_get_target_callback( %f, %f, %f, %f )", limits.xmin, limits.xmax, limits.ymin, limits.ymax );
	 
	 target.xt.z = 0.f;
	 target.xt.x = rand_range( limits.xmin, limits.xmax );
	 target.xt.y = rand_range( limits.ymin, limits.ymax );
	 
	 ROS_INFO( "new target is ( %f, %f, %f )", target.xt.x, target.xt.y, target.xt.z );
	 
	 return true;
}

/**
 * @brief Service - generate a twist command
 * 
 * @param initpos X and Xt
 * @param vel the return of the server, the Twist
 */
bool rg_get_velocity_callback( robot_game::rg_get_vel_srv::Request &initpos, robot_game::rg_get_vel_srv::Response &vel )
{
	//ROS_INFO( "rg_get_velocity_callback( X(%f, %f, %f) Xt(%f, %f, %f) )", initpos.x.x, initpos.x.y, initpos.x.z, initpos.xt.x, initpos.xt.y, initpos.xt.z );
	
	vel.vel.linear.x = K * ( initpos.xt.x - initpos.x.x );
	vel.vel.linear.y = K * ( initpos.xt.y - initpos.x.y );
	vel.vel.linear.z = 0.f;
	
	vel.vel.angular.x = 0.f;
	vel.vel.angular.y = 0.f;
	vel.vel.angular.z = 0.f;
	
	//ROS_INFO( "output Twist( v(%f, %f, %f), w(%f, %f, %f) )", vel.vel.linear.x, vel.vel.linear.y, vel.vel.linear.z, vel.vel.angular.x, vel.vel.angular.y, vel.vel.angular.z);
	
	return true;
}









/**
 * @brief Ask for services, topics, and initialise the node. 
 */
int main( int argc, char** argv )
{
	ros::init( argc, argv, "rg_server" );
	ros::NodeHandle h;
	
	ros::ServiceServer srv_check_target = h.advertiseService( "rg_check_target", rg_check_target_callback );
	ros::ServiceServer srv_get_target = h.advertiseService( "rg_get_target", rg_get_target_callback );
	ros::ServiceServer srv_get_velocity = h.advertiseService( "rg_get_velocity", rg_get_velocity_callback );
	
	ros::spin();
	
	return 0;
}
