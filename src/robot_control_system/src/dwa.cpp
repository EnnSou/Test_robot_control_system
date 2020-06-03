#include <math.h>
#include "dwa.h"
#include <vector>

Dwa::Dwa() :target(0.0,0.0,0.0) {}

void Dwa::initdwa(vector<double> _limits,State _target)
{
	Dwa::limits = _limits;
	Dwa::target = _target;
}

void Dwa::calc_dynamic_window(State curr_state,vector<double>limits,vector<double> &dw)
{
	double v_min,v_max,w_min,w_max;
	
	v_min = curr_state.linear-limits[6]*dt;
	v_max = curr_state.linear+limits[6]*dt;
	w_min = curr_state.angular-limits[7]*dt;
	w_max = curr_state.angular+limits[7]*dt;

	dw[0] = max(v_min,limits[0]);
	dw[1] = min(v_max,limits[1]);
	dw[2] = max(w_min,limits[3]);
	dw[3] = min(w_max,limits[4]);
}

State Dwa::calc_trajectory(State curr_state,double v,double w,vector<double>limits)
{
	double time=0;
	State new_state(0.0,0.0,0.0);
	while(time < pre_time){
	new_state.yaw = curr_state.yaw+w*dt;
	new_state.x = curr_state.x+v*cos(new_state.yaw)*dt;
	new_state.y = curr_state.y+v*sin(new_state.yaw)*dt;
	new_state.linear = v;
	new_state.angular = w;
	time += dt;
	}
	return new_state;
}

double Dwa::calc_goal_cost(State traj,State goal,vector<double>limits)
{
	double goal_mag = sqrt(goal.x * goal.x + goal.y * goal.y);
	double traj_mag = sqrt(traj.x * traj.x + traj.y * traj.y);
	double dot_product = goal.x * traj.x + goal.y * traj.y;
	double dis = dot_product / (goal_mag * traj_mag);
	double angel = acos(dis);
	return angel;
}

double Dwa::calc_obj_cost(){
	return 1.0;
}


State Dwa::FindBestTrajectory(State curr_state,State goal,vector<double>limits)
{

	double v = curr_state.linear;
	double w = curr_state.angular;
	double v_reso = limits[2];
	double w_reso = limits[5];

	double min_cost = 10000.0;
	State  best_traj = curr_state;

//generate sample space

	vector<double>dw(4,0);
	calc_dynamic_window(curr_state,limits,dw);
//evalucate all trajectory
	for(v += v_reso;v < dw[1];)
	{
		for(w += w_reso;w < dw[3];)
		{
	
			State traj = calc_trajectory(curr_state,v,w,limits);

			//calculate cost
			double goal_cost = calc_goal_cost(traj,goal,limits);
			double speed_cost = 1/(limits[1]-traj.linear);
			double obj_cost = calc_obj_cost();
			double final_cost = a*goal_cost+b*obj_cost+c*speed_cost;

			//search best trajectory
			if(final_cost <= min_cost)
			{
				min_cost = final_cost;
				best_traj = traj;
			}
			w += w_reso;
		}
		v += v_reso;
	
	}
	return best_traj;
}
