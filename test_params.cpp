/******************************************************
This code is sole property of Sheikh Zeeshan Basar.
DO NOT CHANGE ANYTHING.
COPYRIGHT 2017
******************************************************/
//down, left and clockwise is -ve
//up, right and anticlockwise is +ve
#include <iostream>
#include <strings.h>
#include <math.h>
#include <stdlib.h>
#include <cstdlib>
#include <ctime>
#include <conio.h>

using namespace std;

int front1, rear1, speed_arr[5]={0};

bool IsEmpty(){
	return(front1==-1&&rear1==-1);
}

bool IsFull(){
	return (rear1+1)%5==front1?true:false;
}

void Enqueue(float x){
	if (IsFull()){
		return;
	}
	else if (IsEmpty()){
		front1=rear1=0;
	}
	else{
		rear1=(rear1+1)%5;
	}
	speed_arr[rear1]=x;
}

void Dequeue(){
	if (IsEmpty()){
		return;
	}
	else if(front1==rear1){
		rear1=front1=-1;
	}
	else{
		front1=(front1+1)%5;
	}
}

float speed_differentiation(float x){
	float t=0.005; //5ms
	for(int i=0;i>=0;i++){
		Dequeue();
		Enqueue(x);
		float speed_differential=(4.0/3.0*(speed_arr[3]-speed_arr[1])/2*t)-(1.0/3.0*(speed_arr[4]-speed_arr[0])/4*t);
        return speed_differential;
	}
}

int main(){
    srand(time(0));
    speed_differentiation(rand()%60);

	return 0;
}
