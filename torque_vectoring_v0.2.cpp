/******************************************************
This code is sole property of ne0n.
DO NOT CHANGE ANYTHING.
COPYRIGHT 2017
******************************************************/
//down and left and braking/deccelerating is -ve
//up and right and accelerating is +ve
#include <iostream>
#include <strings.h>
#include <math.h>
#include <stdlib.h>
#include <windows.h>

using namespace std;
//defined values
//all units in SI
float cg_height=0.432, total_mass=650, PI=3.142, g=9.80665, spring_roll_stiffness_front=246.518, spring_roll_stiffness_rear=425.510, sprung_mass_faxle=220, sprung_mass_raxle=304, rear_roll_center_height=0.062, roll_axis_cg_pdistance=.409011, roll_center_height_fsuspension=0.059, roll_center_height_rsuspension=0.062, unsprung_mass_faxle=40, unsprung_mass_raxle=86, track_width=1.500, wheel_base=2.750, rear_cg_dis=1.100, wheel_radius=0.275, peak_torque=272.700, peak_speed=97.91, roll_grad=0.316, sprung_mass_total=524, sprung_mass_fdistance=1.59542;
//sensor data
float long_accel=0, steering_wheel_angle=5, lateral_accel=0.214, motor_speed_rl=0, motor_speed_rr=0, throttle_pos=0.1, load_front=380, load_rear=424, SPEED=60;

float del_lat_load=(spring_roll_stiffness_rear*roll_grad/track_width)+((sprung_mass_total*sprung_mass_fdistance*rear_roll_center_height)/(track_width*wheel_base))+(unsprung_mass_raxle*wheel_radius/track_width);
float del_long_load=(total_mass*long_accel*cg_height)/(wheel_base);

float wheel_angle_in=16*steering_wheel_angle;
float wheel_angle_out=(30/22)*wheel_angle_in;

/*float speed(float motor_speed_rr, float motor_speed_rl){
	float wheel_speed_rl, wheel_speed_rr, speed_op;
	wheel_speed_rr=2*PI*wheel_radius*motor_speed_rr;
	wheel_speed_rl=2*PI*wheel_radius*motor_speed_rl;

	return speed_op=(wheel_speed_rl+wheel_speed_rr)/2;
}*/

float steering(float wheel_angle_in, float lateral_accel){
	float turn_angle, inner_wheel_turn_radius, turn_radius;
	if(lateral_accel>0){//right turn
		turn_angle=wheel_angle_in;
	}else if(lateral_accel<0){//left turn
		turn_angle=wheel_angle_in;
	}

	inner_wheel_turn_radius=fabs(wheel_base/tan(turn_angle));
	return turn_radius=sqrt(pow((inner_wheel_turn_radius+(track_width/2)),2)+pow(rear_cg_dis,2));


}

int main(int argc, char const *argv[]){

	float load_fl, load_fr, load_rl, load_rr, T_op_rr, T_op_rl, torque_rl, torque_rr, tp_rr, tp_rl;

	while(1){
		float T_max=fabs(throttle_pos)*peak_torque;

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

		//torque transfer eqs
		float torque_req_r=torque_rl+torque_rr;

		float vectored_torque_rr=torque_req_r*(load_rr/load_rear);
		float vectored_torque_rl=torque_req_r*(load_rl/load_rear);

		if((vectored_torque_rr>T_max)&&(vectored_torque_rl>T_max)){
			T_op_rr=((-peak_torque/peak_speed)*motor_speed_rr)+peak_torque;
			T_op_rl=((-peak_torque/peak_speed)*motor_speed_rl)+peak_torque;
		}else if((vectored_torque_rr>T_max)&&(vectored_torque_rl<=T_max)){
			T_op_rr=((-peak_torque/peak_speed)*motor_speed_rr)+peak_torque;
			T_op_rl=vectored_torque_rl;
		}else if((vectored_torque_rr<=T_max)&&(vectored_torque_rl>T_max)){
			T_op_rr=vectored_torque_rr;
			T_op_rl=((-peak_torque/peak_speed)*motor_speed_rl)+peak_torque;
		}else if((vectored_torque_rr<=T_max)&&(vectored_torque_rl<=T_max)){
			T_op_rr=vectored_torque_rr;
			T_op_rl=vectored_torque_rl;
		}

		//throttle o/ps
		tp_rr=(T_op_rr-((-peak_torque/peak_speed)*motor_speed_rr))/peak_torque;
		tp_rl=(T_op_rl-((-peak_torque/peak_speed)*motor_speed_rl))/peak_torque;

		cout<<"Speed= "<<SPEED/*speed(motor_speed_rr, motor_speed_rl)*/<<endl;
		cout<<"Turn radius= "<<steering(wheel_angle_in, lateral_accel)<<endl;
		cout<<"Torque o/p rear right= "<<T_op_rr<<endl;
		cout<<"Torque o/p rear left= "<<T_op_rl<<endl;
		cout<<"Throttle o/p rear right= "<<tp_rr<<endl;
		cout<<"Throttle o/p rear left= "<<tp_rl<<endl;
		Sleep(1000);
		}

	return 0;
}
