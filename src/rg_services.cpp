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

#define TOLERANCE 0.1f
#define K (1.f)






double compute_eucledian_distance( geometry_msgs::Point P1, geometry_msgs::Point P2 )
{
	return sqrt( pow(P2.x-P1.x, 2.0) + pow(P2.y-P1.y, 2.0) + pow(P2.z-P1.z, 2.0) );
}

double rand_range( double min, double max )
{
	return min + (max - min)*( ((double)rand()) / RAND_MAX );
}







// check target
bool rg_check_target_callback( robot_game::rg_check_srv::Request &initpos, robot_game::rg_check_srv::Response &r )
{
	//ROS_INFO( "rg_check_target_callback( X(%f, %f, %f) Xt(%f, %f, %f) )", initpos.x.x, initpos.x.y, initpos.x.z, initpos.xt.x, initpos.xt.y, initpos.xt.z );
	
	double dist = compute_eucledian_distance( initpos.x, initpos.xt );
	//ROS_INFO( "computed distance: %f", dist );
	
	return ( dist <= 0.1f );
}

// get target
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

// get velocity
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
