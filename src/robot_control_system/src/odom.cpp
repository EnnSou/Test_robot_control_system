#include <string>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

double vx=0.1,vth=0.1;
ros::Subscriber cmd_sub;
ros::Publisher odom_pub;

void Callback(const nav_msgs::Odometry::ConstPtr& msg)
{	
	vx=msg->twist.twist.linear.x;
	vth=msg->twist.twist.angular.z;
}


int main(int argc,char **argv)
{
    ros::init(argc, argv, "minitor");
    ros::NodeHandle n;
	ros::Subscriber cmd_sub=n.subscribe("cmd_vel",10,Callback);
    ros::Publisher odom_pub=n.advertise<nav_msgs::Odometry>("odom", 10);
	ros::Rate loop_rate(10);

    //initial position
    double x,y,th;	
	ros::param::get("init_x",x);
	ros::param::get("init_y",y);
	ros::param::get("init_yaw",th);
	

    //velocity
    ros::Time current_time;
    ros::Time last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();


    while(ros::ok())
    {
        current_time=ros::Time::now();

        double  dt=(current_time-last_time).toSec();	
		double	new_th=th+vth*dt;
		double	new_x=x+vx*cos(new_th)*dt;
		double	new_y=y+vx*sin(new_th)*dt;

        nav_msgs::Odometry odom;
        odom.header.stamp=current_time;
        odom.header.frame_id="odom";

        odom.pose.pose.position.x=new_x;
        odom.pose.pose.position.y=new_y;
        odom.pose.pose.position.z=0.0;

        odom.twist.twist.linear.x=vx;
        odom.twist.twist.linear.y=0.0;
        odom.twist.twist.linear.z=0.0;
        odom.twist.twist.angular.x=0.0;
        odom.twist.twist.angular.y=0.0;
        odom.twist.twist.angular.z=vth;

        last_time=current_time;
	
	    std::cout<<"_____________________________"<<"    X:"<<odom.pose.pose.position.x<<"    Y:"<<odom.pose.pose.position.y<<std::endl;
        odom_pub.publish(odom);

	    ros::spinOnce();        
        loop_rate.sleep();
    }
    return 0;
}
