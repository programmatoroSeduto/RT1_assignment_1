
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

//include "robot_game/misc.h"

#define XMIN (-6.f)
#define XMAX (6.f)
#define YMIN (-6.f)
#define YMAX (6.f)

ros::Subscriber in;	// subscription to base_pose_ground_truth (nav_msgs/Odometry)
ros::Publisher out;	// publisher to cmd_vel (geometry_msgs/Twist)
ros::ServiceClient check_tg_srv;
ros::ServiceClient newtg_srv;
ros::ServiceClient get_vel_srv;
geometry_msgs::Point target;





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
