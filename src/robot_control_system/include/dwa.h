#include <vector>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

using namespace std;


const double a=1.0;
const double b=0.2;
const double c=0.3;
const double dt=0.5;
const double pre_time=1;

struct State
{
	double x,y;
	double yaw;
	double linear,angular;
	State(double _x,double _y,double _yaw):x(_x),y(_y),yaw(_yaw),linear(0.0),angular(0.0)
	{
	}
	
};



class Dwa{
public:
	Dwa();
	void initdwa(vector<double> _limits,State _target);
	State FindBestTrajectory(State curr_state,State goal,vector<double>limits);
private:
	void calc_dynamic_window(State curr_sate,vector<double>limits,vector<double> &dw);
	State calc_trajectory(State curr_state,double v,double w,vector<double>limits);
	double calc_goal_cost(State traj,State goal,vector<double>limits);
	double calc_obj_cost();
private:
	vector<double>limits;
	State target;
	
};

