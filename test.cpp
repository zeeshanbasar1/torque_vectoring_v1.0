//THIS IS A TEST FILE
/******************************************************
This code is sole property of ne0n.
DO NOT CHANGE ANYTHING.
COPYRIGHT 2017
******************************************************/
//down and left is -ve
//up and right is +ve
#include <iostream>
#include <strings.h>
#include <math.h>
#include <stdlib.h>

using namespace std;
//defined values
float PI=3.1416, g=9.80665 ,spring_roll_stiffness_front, spring_roll_stiffness_rear, sprung_mass_faxle, sprung_mass_raxle, roll_axis_cg_pdistance, roll_center_height_fsuspension, roll_center_height_rsuspension, unsprung_mass_faxle, unsprung_mass_raxle, unsprung_mass_height_cg_faxle, unsprung_mass_height_cg_raxle, track_width, load_front, load_rear, wheel_base, rear_cg_dis, wheel_radius, peak_torque, peak_speed;
//sensor data
float roll_angle, long_accel, lateral_accel, wheel_angle_l, wheel_angle_r, motor_speed_rl, motor_speed_rr, throttle_pos;

//float total_load_trasfer(float roll_angle, float long_accel){
	//float load_transf, load_transr;

	float load_transf=((spring_roll_stiffness_front*roll_angle)+(sprung_mass_faxle*roll_center_height_fsuspension*roll_angle)+(sprung_mass_faxle*long_accel*roll_center_height_fsuspension)+(unsprung_mass_faxle*long_accel*unsprung_mass_height_cg_faxle))/track_width;
	float load_transr=((spring_roll_stiffness_rear*roll_angle)+(sprung_mass_raxle*roll_center_height_rsuspension*roll_angle)+(sprung_mass_raxle*long_accel*roll_center_height_rsuspension)+(unsprung_mass_raxle*long_accel*unsprung_mass_height_cg_raxle))/track_width;
	float load_transfer[2]={load_transf, load_transr};
	//return load_transfer;
//}

/*
float& set_val_load_transfer(int a[]){
	return load_transfer[i];
}
*/

void steering(float wheel_angle_r, float wheel_angle_l, float lateral_accel){
	float turn_angle, inner_wheel_turn_radius, turn_radius;
	if(lateral_accel>0){//right turn
		turn_angle=wheel_angle_r;
	}else if(lateral_accel<0){//left turn
		turn_angle=wheel_angle_l;
	}

	inner_wheel_turn_radius=fabs(wheel_base/tan(turn_angle));
	turn_radius=sqrt(pow((inner_wheel_turn_radius+(track_width/2)),2)+pow(rear_cg_dis,2));


}

void speed(float motor_speed_rr, float motor_speed_rl){
	float wheel_speed_fl, wheel_speed_fr, wheel_speed_rl, wheel_speed_rr, speed;
	wheel_speed_rr=2*PI*wheel_radius*motor_speed_rr;
	wheel_speed_rl=2*PI*wheel_radius*motor_speed_rl;

	speed=(wheel_speed_rl+wheel_speed_fr+wheel_speed_fl+wheel_speed_rr)/4;
}

int main(int argc, char const *argv[]){

	//float *lt;
	float load_fl, load_fr, load_rl, load_rr, T_op_rr, T_op_rl, torque_rl, torque_rr, tp_rr, tp_rl;
	//float , torque_fr, torque_fl;

	//total_load_trasfer(roll_angle, long_accel);

	float T_max=fabs(throttle_pos)*peak_torque;
	//float S_max=fabs(throttle_pos)*peak_speed;

	if(lateral_accel>0&&lateral_accel<g){//right turn
		load_fl=(load_front/2)+load_transfer[0];
		load_fr=(load_front/2)-load_transfer[0];
		load_rl=(load_rear/2)+load_transfer[1];
		load_rr=(load_rear/2)-load_transfer[1];
	}else if(lateral_accel<0&&lateral_accel>g){//left turn
		load_fl=(load_front/2)-load_transfer[0];
		load_fr=(load_front/2)+load_transfer[0];
		load_rl=(load_rear/2)-load_transfer[1];
		load_rr=(load_rear/2)+load_transfer[1];
	}else{

	}

	//torque transfer eqs 
	//float torque_req_f=torque_fl+torque_fr;
	float torque_req_r=torque_rl+torque_rr;

	float vectored_torque_rr=torque_req_r*(load_rr/load_rear);
	float vectored_torque_rl=torque_req_r*(load_rl/load_rear);

	if((vectored_torque_rr>T_max) || (vectored_torque_rl>T_max)){
		T_op_rr=((-peak_torque/peak_speed)*motor_speed_rr)+peak_torque;
		T_op_rl=((-peak_torque/peak_speed)*motor_speed_rl)+peak_torque;
	}
	else if((vectored_torque_rr>T_max) || (vectored_torque_rl<=T_max)){
		T_op_rr=((-peak_torque/peak_speed)*motor_speed_rr)+peak_torque;
		T_op_rl=vectored_torque_rl;	
	}
	else if((vectored_torque_rr<=T_max) || (vectored_torque_rl>T_max)){
		T_op_rr=vectored_torque_rr;
		T_op_rl=((-peak_torque/peak_speed)*motor_speed_rl)+peak_torque;
	}
	else if((vectored_torque_rr<=T_max) || (vectored_torque_rl<=T_max)){
		T_op_rr=vectored_torque_rr;
		T_op_rl=vectored_torque_rl;
	}

	//throttle o/ps
	tp_rr=(T_op_rr-((-peak_torque/peak_speed)*motor_speed_rr))/peak_torque;
	tp_rl=(T_op_rl-((-peak_torque/peak_speed)*motor_speed_rl))/peak_torque;

	return 0;
}