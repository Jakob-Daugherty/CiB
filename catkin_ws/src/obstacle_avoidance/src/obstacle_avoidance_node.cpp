//obstacle_avoidance_node
# include "ros/ros.h"
# include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "ca_msgs/Bumper.h"


bool left0 = 0;
//bool left1 = 0;
//bool left2 = 0;
//bool left3 = 0;
bool right0 = 0;
//bool right1 = 0;
//bool right2 = 0;
//bool right3 = 0; 

geometry_msgs::Twist move_avoid(){
	geometry_msgs::Twist return_msg;
	
	//init return message 
	return_msg.linear.x = 0.0;
	return_msg.linear.y = 0.0;
	return_msg.linear.z = 0.0;
	return_msg.angular.x = 0.0;
	return_msg.angular.y = 0.0;
	return_msg.angular.z = 0.0;
	
	if((left0 && right0) == 1){
		return_msg.angular.z = -1;
	} else if(left0 == 1){
		//return_msg.velocityLeft = 1;
		return_msg.angular.z = -1;
		//return_msg.velocityRight = -1;
	} else if(right0 == 1){
		//return_msg.velocityLeft = -1;
		//return_msg.velocityRight = 1;
		return_msg.angular.z = 1;
	} else {
		//return_msg.velocityLeft = 0;
		//return_msg.velocityRight = 0;
		return_msg.angular.z = 0;
	}
	return return_msg;
}

void chatterCallback(const ca_msgs::Bumper::ConstPtr& vel)
{	
	ROS_INFO("\n");
	ROS_INFO("Contact Sensors");
	ROS_INFO("is left pressed -> [%d]", vel->is_left_pressed);
	ROS_INFO("is right pressed -> [%d]", vel->is_right_pressed);
	ROS_INFO("\n");
//	ROS_INFO("Bumper light Sensors");
//	ROS_INFO("is light left -> [%d]", vel->is_light_left);
//	ROS_INFO("is light front left -> [%d]", vel->is_light_front_left);
//	ROS_INFO("is light center left -> [%d]", vel->is_light_center_left);
//	ROS_INFO("is light center right -> [%d]", vel->is_light_center_right);
//	ROS_INFO("is light front right -> [%d]", vel->is_light_front_right);
//	ROS_INFO("is light right -> [%d]", vel->is_light_right);
//	ROS_INFO("\n");
//	ROS_INFO("Raw light sensors");
//	ROS_INFO("light signal left -> [%d]", vel->light_signal_left);
//	ROS_INFO("light signal front left -> [%d]", vel->light_signal_front_left);
//	ROS_INFO("light signal center left -> [%d]", vel->light_signal_center_left);
//	ROS_INFO("light signal center right -> [%d]", vel->light_signal_center_right);
//	ROS_INFO("light signal front right -> [%d]", vel->light_signal_front_right);
//	ROS_INFO("light signal right -> [%d]", vel->light_signal_right);
//	//ROS_INFO("I heard angular: [%f,%f]", vel->linear_velocity, vel->angular_velocity);
	left0 = vel->is_left_pressed;
//	left1 = vel->is_light_center_left;
//	left2 = vel->is_light_front_left;
//	left3 = vel->is_light_left;
	right0 = vel->is_right_pressed;
//	right1 = vel->is_light_center_right;
//	right2 = vel->is_light_front_right;
//	right3 = vel->is_light_right;


}

int main (int argc, char **argv)
{
ros::init(argc, argv, "obstacle_avoidance_node");
ros::start();
ROS_INFO_STREAM("Obstacle Avoidance Node Start");
ros::NodeHandle chatListen;
ros::Subscriber sub = chatListen.subscribe("/bumper", 1000, chatterCallback);
ros::NodeHandle chatTalk;
ros::Publisher chatter_pub = chatTalk.advertise<geometry_msgs::Twist>("/obstacle_avoidance/cmd_vel", 1000);
ros::Rate loop_rate(1000);
loop_rate.sleep();
geometry_msgs::Twist msg;
while(ros::ok()){
	ros::spinOnce();
	msg = move_avoid();
	if(0 != msg.angular.z){
		chatter_pub.publish(msg);
	}
	//chatter_pub.publish(msg);
	ros::spinOnce();
	loop_rate.sleep();
} 
//ros::spin();
ros::shutdown();
return 0;
}