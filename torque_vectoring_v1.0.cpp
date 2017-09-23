/******************************************************
This code is sole property of Sheikh Zeeshan Basar.
DO NOT CHANGE ANYTHING.
COPYRIGHT 2017
******************************************************/
//down, left, clockwise and braking/deccelerating is -ve
//up, right, anticlockwise and accelerating is +ve
#include <iostream>
#include <strings.h>
#include <math.h>
#include <stdlib.h>

using namespace std;

//all units in SI
float cg_height=0.432, total_mass=650, PI=3.142, g=9.80665, spring_roll_stiffness_front=246.518,
	spring_roll_stiffness_rear=425.510, sprung_mass_faxle=220, sprung_mass_raxle=304, rear_roll_center_height=0.062,
	roll_axis_cg_pdistance=.409011, roll_center_height_fsuspension=0.059, roll_center_height_rsuspension=0.062,
	unsprung_mass_faxle=40, unsprung_mass_raxle=86, track_width=1.500, wheel_base=2.750, rear_cg_dis=1.100,
	wheel_radius=0.275, peak_torque=272.700, peak_speed=97.91, I_perp_raxle=1222.225, load_front=260, load_rear=390;
//sensor data
float long_accel=0, lateral_accel=0, motor_speed_rl=0, motor_speed_rr=0, throttle_pos=0, steering_wheel_angle=0, actual_yaw_rate=0;
//sensor arrays
float speed_arr[5]={0}, steering_wheel_angle_arr[5]={0}, actual_yaw_rate_arr[5]={0};

//Queue-Dequeue functions(1)
int front1, rear1;

bool IsEmpty(){
	return(front1==-1&&rear1==-1);
}

bool IsFull(){
	return (rear1+1)%5==front1?true:false;
}

void Enqueue1(float x){
	if (IsFull()){
		return;
	}else if (IsEmpty()){
		front1=rear1=0;
	}else{
		rear1=(rear1+1)%5;
	}
	speed_arr[rear1]=x;
}

void Dequeue1(){
	if (IsEmpty()){
		return;
	}else if(front1==rear1){
		rear1=front1=-1;
	}else{
		front1=(front1+1)%5;
	}
}

//Queue-Dequeue fucntions(2)
int front2, rear2;

bool IsEmpty2(){
	return(front2==-1&&rear2==-1);
}

bool IsFull2(){
	return (rear2+1)%5==front2?true:false;
}

void Enqueue2(float x){
	if (IsFull2()){
		return;
	}else if (IsEmpty2()){
		front2=rear2=0;
	}else{
		rear2=(rear2+1)%5;
	}
	steering_wheel_angle_arr[rear2]=x;
}

void Dequeue2(){
	if (IsEmpty()){
		return;
	}else if(front2==rear2){
		rear2=front2=-1;
	}else{
		front2=(front2+1)%5;
	}
}

//Queue-Dequeue functions(3)
int front3, rear3;

bool IsEmpty3(){
	return(front3==-1&&rear3==-1);
}

bool IsFull3(){
	return (rear3+1)%5==front3?true:false;
}

void Enqueue3(float x){
	if (IsFull3()){
		return;
	}else if (IsEmpty3()){
		front3=rear3=0;
	}else{
		rear3=(rear3+1)%5;
	}
	actual_yaw_rate_arr[rear3]=x;
}

void Dequeue3(){
	if (IsEmpty()){
		return;
	}else if(front3==rear3){
		rear3=front3=-1;
	}else{
		front3=(front3+1)%5;
	}
}

//lateral load transfer
float roll_grad=0.316, sprung_mass_total=524, sprung_mass_fdistance=1.59542;

float del_lat_load=(spring_roll_stiffness_rear*roll_grad/track_width)+
		((sprung_mass_total*sprung_mass_fdistance*rear_roll_center_height)/(track_width*wheel_base))+
		(unsprung_mass_raxle*wheel_radius/track_width);
float del_long_load=(total_mass*long_accel*cg_height)/(wheel_base);

/*float wheel_angle_in=16*steering_wheel_angle;
float wheel_angle_out=(30/22)*wheel_angle_in;

float steering(float wheel_angle_in, float lateral_accel){//turn radius calculation
	float turn_angle, inner_wheel_turn_radius, turn_radius;
	if(lateral_accel>0){//right turn
		turn_angle=wheel_angle_in;
	}else if(lateral_accel<0){//left turn
		turn_angle=wheel_angle_in;
	}

	inner_wheel_turn_radius=fabs(wheel_base/tan(turn_angle));
	return turn_radius=sqrt(pow((inner_wheel_turn_radius+(track_width/2)),2)+pow(rear_cg_dis,2));
}*/

float speed(float motor_speed_rr, float motor_speed_rl){
	float wheel_speed_fl, wheel_speed_fr, wheel_speed_rl, wheel_speed_rr, speed_actual;
	wheel_speed_rr=2*PI*wheel_radius*motor_speed_rr;
	wheel_speed_rl=2*PI*wheel_radius*motor_speed_rl;

	speed_actual=(wheel_speed_rl+wheel_speed_rr)/2;
	return speed_actual;
}

float yaw_rate(float wheel_speed_rl, float wheel_speed_rr, float wheel_base){

	float yaw_rate_desired=(wheel_speed_rr-wheel_speed_rl)/wheel_base;
	return yaw_rate_desired;
}

float speed_differentiation(float x){
	float t=0.005; //5ms
	Dequeue1();
	Enqueue1(x);
	float speed_differential=(4.0/3.0*(speed_arr[3]-speed_arr[1])/2*t)-(1.0/3.0*(speed_arr[4]-speed_arr[0])/4*t);
	return speed_differential;
}

float actual_yaw_rate_differentiation(float ayr){
	float t=0.005; //5ms
	Dequeue3();
	Enqueue3(ayr);
	float yaw_differential=(4.0/3.0*(actual_yaw_rate_arr[3]-actual_yaw_rate_arr[1])/2*t)-
		(1.0/3.0*(actual_yaw_rate_arr[4]-actual_yaw_rate_arr[0])/4*t);
	return yaw_differential;	
}

float steering_wheel_angle_differentiation(float ang){
	float t=0.005; //5ms
	Dequeue2();
	Enqueue2(ang);
	float angle_differential=(4.0/3.0*(steering_wheel_angle_arr[3]-steering_wheel_angle_arr[1])/2*t)-
		(1.0/3.0*(steering_wheel_angle_arr[4]-steering_wheel_angle_arr[0])/4*t);
	return angle_differential;
}

float sec(float x){
	float sec=1/cos(x);
	return sec;
}

int main(int argc, char const *argv[])
{
	float load_fl, load_fr, load_rl, load_rr, T_op_rr, T_op_rl, torque_rl, torque_rr, tp_rr, tp_rl, del_torque, speed_arr[5]={0};

	while(1){
		float T_max=throttle_pos*peak_torque;
	//load transfer
		if(lateral_accel>0&&long_accel>0){//right turn accelerating
			load_fl=(load_front/2)+del_lat_load+del_long_load;
			load_fr=(load_front/2)-del_lat_load+del_long_load;
			load_rl=(load_rear/2)+del_lat_load+del_long_load;
			load_rr=(load_rear/2)-del_lat_load+del_long_load;
		}else if(lateral_accel<0&&long_accel>0){//left turn accelerating
			load_fl=(load_front/2)-del_lat_load+del_long_load;
			load_fr=(load_front/2)+del_lat_load+del_long_load;
			load_rl=(load_rear/2)-del_lat_load+del_long_load;
			load_rr=(load_rear/2)+del_lat_load+del_long_load;
		}else if(lateral_accel>0&&long_accel<0){//right turn braking
			load_fl=(load_front/2)+del_lat_load-del_long_load;
			load_fr=(load_front/2)-del_lat_load-del_long_load;
			load_rl=(load_rear/2)+del_lat_load-del_long_load;
			load_rr=(load_rear/2)-del_lat_load-del_long_load;
		}else if(lateral_accel>0&&long_accel<0){//left turn braking
			load_fl=(load_front/2)+del_lat_load-del_long_load;
			load_fr=(load_front/2)-del_lat_load-del_long_load;
			load_rl=(load_rear/2)+del_lat_load-del_long_load;
			load_rr=(load_rear/2)-del_lat_load-del_long_load;
		}


	//torque vectoring
		float x=speed(motor_speed_rr, motor_speed_rl);
		float sp=speed_differentiation(x);
		float yr=actual_yaw_rate_differentiation(actual_yaw_rate);
		float ang=steering_wheel_angle_differentiation(steering_wheel_angle);
		float secant=sec(steering_wheel_angle);
    	float y=steering_wheel_angle;

		del_torque=((I_perp_raxle*wheel_radius)/(track_width*wheel_base))*((sp*tan(y))+(((x*secant*secant*ang)-(yr))));

		if(lateral_accel>0){//right
			T_op_rr=torque_rr-(del_torque);
			T_op_rl=torque_rl+(del_torque);
		}else if(lateral_accel<0){//left
			T_op_rr=torque_rr+(del_torque);
			T_op_rl=torque_rl-(del_torque);
		}

		//throttle o/ps
		tp_rr=(T_op_rr-((-peak_torque/peak_speed)*motor_speed_rr))/peak_torque;
		tp_rl=(T_op_rl-((-peak_torque/peak_speed)*motor_speed_rl))/peak_torque;

		cout<<"Torque o/p rr= "<<T_op_rr<<endl;
		cout<<"Torque o/p rl= "<<T_op_rl<<endl;
		cout<<"Throttle o/p rr= "<<tp_rr<<endl;
		cout<<"Throttle o/p rl= "<<tp_rl<<endl;
	}
	return 0;
}
