/*
* Class: CSCI573
* Name: Jiseong Yoo
* Date: 2018-02-15
*
*/

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <sstream>

#define TRUE 1
#define FALSE 0

using namespace std;
using namespace ros;
using namespace geometry_msgs;
using namespace turtlesim;

/** declare publishers and subscribers **/
Publisher velocity_publisher;
Subscriber pose_subscriber;

Pose turtlesim_pose;

/** functions declarations **/
void getDesired(turtlesim::Pose desired_pose, double tolerance);
void setOrientaion(turtlesim::Pose desired_pose, double tolerance);
double getDistance(double x1, double y1, double x2, double y2);
void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);
void logoSetting(turtlesim::Pose *desired_pose);
void drawMines();
void title();


/** main function **/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "turtlesim_logo");
	ros::NodeHandle n;
	
	velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
	pose_subscriber = n.subscribe("/turtle1/pose",5,poseCallback);
	ros::spinOnce();

	while(ros::ok()){
		title();
		ros::spinOnce();
		drawMines();
		break;		
	}

	// Forced End? or Successful End?	
	if(ros::ok() == FALSE){
		cout<<"User stopped the program"<<endl;
	}
	else{
		cout<<"Successful End"<<endl;
	}

	return 0;
}


/** update the global variable turtlesim_pose with the pose of the robot **/
void poseCallback(const turtlesim::Pose::ConstPtr & pose_message){
	turtlesim_pose.x=pose_message->x;
	turtlesim_pose.y=pose_message->y;
	turtlesim_pose.theta=pose_message->theta;
}


/** draw the Mines logo. Use move and rotate functions defined below **/
void drawMines(){
	Pose desired_pose[24];
	logoSetting(desired_pose);

	for(int i=0;i<24;i++){
		setOrientaion(desired_pose[i], 0.01);
		getDesired(desired_pose[i], 0.05);	
	}
	
	cout<<"Drawing End"<<endl;
}

/** display the start of the application **/
void title(){
	cout<<"***************************"<<endl;
	cout<<"TurtleSim! Draw Mines Logo!"<<endl;
	cout<<"***************************"<<endl;
}

/** Set orientation of the turtlesim toward a desired point **/
void setOrientaion(turtlesim::Pose desired_pose, double tolerance){
	geometry_msgs::Twist vel_msg;
	double p_angular = 1.5;	// Proportional for angular velocity
	double desired_angle = atan2(desired_pose.y-turtlesim_pose.y, desired_pose.x-turtlesim_pose.x);
	double angle_error = desired_angle - turtlesim_pose.theta;

	ros::Rate loop_rate(10);	// loop rate 10Hz
	do{
		if(!ros::ok())
			break;

		vel_msg.linear.x = 0;
		vel_msg.linear.y = 0;
		vel_msg.linear.z = 0;

		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		vel_msg.angular.z = p_angular*angle_error;

		cout<<"Desired_angle = "<<desired_angle<<", Turtlesim_theta = "<<turtlesim_pose.theta<<", Angular_error = "<<angle_error<<endl;
		velocity_publisher.publish(vel_msg);
		
		ros::spinOnce();
		loop_rate.sleep();
		angle_error = desired_angle - turtlesim_pose.theta;

	}while(abs(angle_error) > tolerance);

	cout<<"Angle set!  Error = "<<angle_error<<endl;
	vel_msg.angular.z = 0;
	velocity_publisher.publish(vel_msg);
}

/** Go to a desired point **/
void getDesired(turtlesim::Pose desired_pose, double tolerance){
	geometry_msgs::Twist vel_msg;

	double p_linear = 2;	// Proportional for linear velocity
	double distance_error = getDistance(turtlesim_pose.x, turtlesim_pose.y, desired_pose.x, desired_pose.y);

	// x_error, y_error with respect to the world frame
	double x_error = desired_pose.x - turtlesim_pose.x;
	double y_error = desired_pose.y - turtlesim_pose.y;

	// x_error, y_error with respect to the turtlesim frame
	double turtlesim_x_error = x_error*cos(turtlesim_pose.theta) + y_error*sin(turtlesim_pose.theta);
	double turtlesim_y_error = -x_error*sin(turtlesim_pose.theta) + y_error*cos(turtlesim_pose.theta);

	ros::Rate loop_rate(10);	// loop rate 10Hz

	do{
		if(!ros::ok())
			break;
		vel_msg.linear.x = p_linear*turtlesim_x_error;
		vel_msg.linear.y = p_linear*turtlesim_y_error;
		vel_msg.linear.z = 0;

		vel_msg.angular.x = 0;
		vel_msg.angular.y = 0;
		vel_msg.angular.z = 0;
		cout<<"Desired_pose = ("<<desired_pose.x<<","<<desired_pose.y<<"), Turtlesim_pose = ("<<turtlesim_pose.x<<","<<turtlesim_pose.y<<"), Distance_error = "<<distance_error<<endl;
		velocity_publisher.publish(vel_msg);
		
		ros::spinOnce();
		loop_rate.sleep();
		
		x_error = desired_pose.x - turtlesim_pose.x;
		y_error = desired_pose.y - turtlesim_pose.y;		
		turtlesim_x_error = x_error*cos(turtlesim_pose.theta) + y_error*sin(turtlesim_pose.theta);		
		turtlesim_y_error = -x_error*sin(turtlesim_pose.theta) + y_error*cos(turtlesim_pose.theta);		
		distance_error = getDistance(turtlesim_pose.x, turtlesim_pose.y, desired_pose.x, desired_pose.y);
	}while(distance_error > tolerance);
	
	cout<<"Desired get!  Distance Error = "<<distance_error<<endl;
	vel_msg.linear.x = 0;
	velocity_publisher.publish(vel_msg);
}

/** returns the distance between point (x1, y1) and point (x2, y2) **/
double getDistance(double x1, double y1, double x2, double y2){
	return sqrt(pow((x1-x2),2)+pow((y1-y2),2));
}

/** Colorado School of Mines Logo map setting **/
/** Reference image file = "Colorado_school_of_Mines_Logo_Map.png" **/
void logoSetting(turtlesim::Pose *desired_pose){

	double a = 1.0;
	double b = 0.5;
	double c = 0.2;
	double d = 2.5;
	
	desired_pose[0].x = 7;
	desired_pose[0].y = 7;

	desired_pose[1].x = desired_pose[0].x + a;
	desired_pose[1].y = desired_pose[0].y;

	desired_pose[2].x = desired_pose[1].x;
	desired_pose[2].y = desired_pose[1].y - b;

	desired_pose[3].x = desired_pose[2].x - c;
	desired_pose[3].y = desired_pose[2].y;

	desired_pose[4].x = desired_pose[3].x;
	desired_pose[4].y = desired_pose[3].y - d;

	desired_pose[5].x = desired_pose[4].x + c;
	desired_pose[5].y = desired_pose[4].y;

	desired_pose[6].x = desired_pose[5].x;
	desired_pose[6].y = desired_pose[5].y - b;

	desired_pose[7].x = desired_pose[6].x - a - c;
	desired_pose[7].y = desired_pose[6].y;

	desired_pose[8].x = desired_pose[7].x;
	desired_pose[8].y = desired_pose[7].y + b;

	desired_pose[9].x = desired_pose[8].x + c;
	desired_pose[9].y = desired_pose[8].y;

	desired_pose[10].x = desired_pose[9].x;
	desired_pose[10].y = 5.544445;

	desired_pose[11].x = 5.544445;
	desired_pose[11].y = 4;

	desired_pose[12].x = 4.08889;
	desired_pose[12].y = 5.544445;

	desired_pose[13].x = desired_pose[12].x;
	desired_pose[13].y = desired_pose[9].y;

	desired_pose[14].x = desired_pose[13].x + c;
	desired_pose[14].y = desired_pose[13].y;

	desired_pose[15].x = desired_pose[14].x;
	desired_pose[15].y = desired_pose[14].y - b;

	desired_pose[16].x = desired_pose[15].x - a - c;
	desired_pose[16].y = desired_pose[15].y;

	desired_pose[17].x = desired_pose[16].x;
	desired_pose[17].y = desired_pose[16].y + b;

	desired_pose[18].x = desired_pose[17].x + c;
	desired_pose[18].y = desired_pose[17].y;

	desired_pose[19].x = desired_pose[18].x;
	desired_pose[19].y = desired_pose[18].y + d;

	desired_pose[20].x = desired_pose[19].x - c;
	desired_pose[20].y = desired_pose[19].y;

	desired_pose[21].x = desired_pose[20].x;
	desired_pose[21].y = desired_pose[20].y + b;

	desired_pose[22].x = desired_pose[21].x + a;
	desired_pose[22].y = desired_pose[21].y;

	desired_pose[23].x = 5.544445;
	desired_pose[23].y = 5.544445;

}

