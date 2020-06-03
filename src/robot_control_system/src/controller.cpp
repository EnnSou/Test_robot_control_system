#include "ros/ros.h"
#include "astar.h"
#include "dwa.h"
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <vector>

State pose(5.0,0.0,0.0);
nav_msgs::Odometry cmd;
ros::Subscriber odom_sub;
ros::Publisher  cmd_vel_pub;
ros::Publisher	final_traj_pub;


double point_area=0.5;

void odom_subCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	pose.x = msg->pose.pose.position.x;
	pose.y = msg->pose.pose.position.y;
	pose.yaw = msg->pose.pose.orientation.w;
	pose.linear = msg->twist.twist.linear.x;
	pose.angular = msg->twist.twist.angular.z;
	
}

bool isreached(State goal)
{
	double dis;
	dis = sqrt((pose.x-goal.x) * (pose.x-goal.x) + (pose.y-goal.y) * (pose.y-goal.y));
	if(dis <= point_area)return true;
	return false;
}

State leastpoin(State pose,list<Point *>path)
{
	double dis;
	State target(0.0,0.0,0.0);
	while(!path.empty()){
		Point *temp = path.front();
		dis = sqrt((pose.x - temp->x) * (pose.x - temp->x) + (pose.y - temp->y) * (pose.y - temp->y));
		if(dis > point_area)
		{
			target.x = temp->x;
			target.y = temp->y;
			break;
		}
		else{
			path.pop_front();
		}
	}
	return target;	
}

int main(int argc,char **argv)
{
	ros::init(argc,argv,"controller");
	ros::NodeHandle n;
	ros::Subscriber odom_sub = n.subscribe("odom",100,odom_subCallback);
	ros::Publisher  cmd_vel_pub = n.advertise<nav_msgs::Odometry>("cmd_vel",100);
	ros::Publisher	final_traj_pub = n.advertise<nav_msgs::Odometry>("trajectory",100);
	ros::Rate loop_rate(10);
	
	State start(0.0,0.0,0.0),goal(10.0,10.0,0.0);
	Point startpoint(0,0),goalpoint(0,0);
	vector<double>limits(8,0);
	Astar astar;
	auto dwa = new Dwa;
	ros::param::get("start_x",start.x);
	ros::param::get("start_y",start.y);
	ros::param::get("start_yam",start.yaw);
	ros::param::get("goal_x",goal.x);
	ros::param::get("goal_y",goal.y);
	ros::param::get("goal_yaw",goal.yaw);
	ros::param::get("v_min",limits[0]);
	ros::param::get("v_max",limits[1]);
	ros::param::get("v_reso",limits[2]);
	ros::param::get("w_min",limits[3]);
	ros::param::get("w_max",limits[4]);
	ros::param::get("w_reso",limits[5]);
	ros::param::get("v_acc",limits[6]);
	ros::param::get("w_acc",limits[7]);

	startpoint.x = start.x;
	startpoint.y = start.y;
	goalpoint.x = goal.x;
	goalpoint.y = goal.y;

	

	while(ros::ok())
	{
		if(isreached(goal)){
			if(goal.yaw != pose.yaw)
			{
				cmd.twist.twist.linear.x = 0.0;
				cmd.twist.twist.angular.z = goal.yaw-pose.yaw;
			}
			else
			{
			ROS_INFO("REACHED GOAL !!!");
			}
		}
		else 
		{	
			vector<vector<int>> maze =
			{
				{1,1,1,1,1,1,1,1,1,1,1,1},
				{1,0,0,1,1,0,1,0,0,0,0,1},
				{1,0,0,1,1,0,0,0,0,0,0,1},
				{1,0,0,0,0,0,1,0,0,1,1,1},
				{1,1,1,0,0,0,0,0,1,1,0,1},
				{1,1,0,1,0,0,0,0,0,0,0,1},
				{1,0,1,0,0,0,0,1,0,0,0,1},
				{1,1,1,1,1,1,1,1,1,1,1,1}
			};

			astar.InitAstar(maze);
			list<Point *> global_path = astar.GetPath(startpoint,goalpoint,false);
			State target = leastpoin(pose,global_path);
 
			State final_cmd = dwa->FindBestTrajectory(pose,target,limits);

			cmd.twist.twist.linear.x = final_cmd.linear;
			cmd.twist.twist.angular.z = final_cmd.angular;
			cmd.pose.pose.position.x = final_cmd.x;
			cmd.pose.pose.position.y = final_cmd.y;


			std::cout<<"velocity:"<<cmd.twist.twist.linear.x<<"    angular:"<<cmd.twist.twist.angular.z<<endl;
			cmd_vel_pub.publish(cmd);
			final_traj_pub.publish(cmd);
		

		}
		ros::spinOnce();		
		loop_rate.sleep();
		
	}
	return 0;
}

