//暂定坐标为uint16，角度为float，后续根据实际情况修改

#include "zf_common_headfile.h"
#include "PCA9685.h"


void arm_angle2pwm(float angle1,float angle2,float angle3,float angle4);//根据实际情况选择参数类型
void arm_poistion2angle(uint16 x,uint16 y,uint16 z);     //参数按照坐标是int还是float类型进行修改


#define arm1_length  1       //宏定义机械臂的长度及圆盘半径
#define arm2_length  1
#define arm3_length  1
#define arm4_length  1
#define circle_length  1

float jiao1,jiao2,jiao3,jiao4;                      //四个姿态角

void arm_move(uint16 x,uint16 y,uint16 z)
{
    arm_poistion2angle(x,y,z);
    arm_angle2pwm(jiao1,jiao2,jiao3,jiao4); 

}


void arm_poistion2angle(uint16 x,uint16 y,uint16 z)     //参数按照坐标是int还是float类型进行修改
{
    float a1,a2,a3,a4;                  //a1为底部圆台高度 剩下三个为三个机械臂长度 
	// float j1,j4,j2,j3;                      //四个姿态角
	float L,H,P;	             //L =a2*sin(j2) + a3*sin(j2 + j3);H = a2*cos(j2) + a3*cos(j2 + j3); P为底部圆盘半径R
	float j_all;	                             //j2,j3,j4之和
	float len,high;                       //总长度,总高度
	float Cosj3,Sinj3;                   //用来存储cosj3,sinj3数值
	float Cosj2,Sinj2;
	float K1,K2;
	float X,Y,Z;  	             //输入 （X,Y,Z）坐标
	int i;
	float n,m,q;
	n = 0;
	m = 0;
	q = 0;
	//目标点坐标（X，Y，Z）
	X = x;
	Y = y;
	Z = z;

	P = circle_length;     //底部圆盘半径
	a1 = arm1_length; 	//底部圆盘高度	            
	a2 = arm2_length;    //机械臂长度
	a3 = arm3_length;
	a4 = arm4_length;
	
	if (X == 0) 
    {
	    jiao1=90;
    }else {
	    jiao1 = atan((Y+P)/X)*(57.3);
    }

	for(i=0;i<=180;i++)
	{	
		j_all = 3.1415927*i/180;

		len = sqrt((Y+P)*(Y+P)+X*X);
		high = Z;
			
		L = len	- a4*sin(j_all);
		H = high - a4*cos(j_all) - a1;
		
		Cosj3 = ((L*L)+(H*H)-((a2)*(a2))-((a3)*(a3)))/(2*(a2)*(a3));
		Sinj3 = (sqrt(1-(Cosj3)*(Cosj3)));
		
		jiao3 = atan((Sinj3)/(Cosj3))*(57.3);
		
		K2 = a3*sin(jiao3/57.3);
		K1 = a2+a3*cos(jiao3/57.3);
		
		Cosj2 = (K2*L+K1*H)/(K1*K1+K2*K2);
		Sinj2 = (sqrt(1-(Cosj2)*(Cosj2)));
		
		jiao2 = atan((Sinj2)/(Cosj2))*57.3;
		jiao4 = j_all*57.3- jiao2 - jiao3;
		
		if(jiao2>=0&&jiao3>=0&&jiao4>=-90&&jiao2<=180&&jiao3<=180&&jiao4<=90)
		{
			n=n+1;
		}
    } 


    for(i=0;i<=180;i++)
	{
		j_all = 3.1415927*i/180;
		
		len = sqrt((Y+P)*(Y+P)+X*X);
		high = Z;

		L = len	- a4*sin(j_all);
		H = high - a4*cos(j_all) - a1;
		
		Cosj3 = ((L*L)+(H*H)-((a2)*(a2))-((a3)*(a3)))/(2*(a2)*(a3));
		Sinj3 = (sqrt(1-(Cosj3)*(Cosj3)));
		
		jiao3 = atan((Sinj3)/(Cosj3))*(57.3);
		
		K2 = a3*sin(jiao3/57.3);
		K1 = a2+a3*cos(jiao3/57.3);
		
		Cosj2 = (K2*L+K1*H)/(K1*K1+K2*K2);
		Sinj2 = (sqrt(1-(Cosj2)*(Cosj2)));
		
		jiao2 = atan((Sinj2)/(Cosj2))*57.3;
		jiao4 = j_all*57.3- jiao2 - jiao3;
		
	    if(jiao2>=0&&jiao3>=0&&jiao4>=-90&&jiao2<=180&&jiao3<=180&&jiao4<=90)
		{
			m=m+1;
			if(m==n/2||m==(n+1)/2)
			{	
				break;
			}
		}
    }

 //   arm_angle2pwm(j1,j2,j3,j4)

}

void arm_angle2pwm(float angle1,float angle2,float angle3,float angle4)//根据实际情况选择参数类型
{
	uint32 off1 = 0;  
    uint32 off2 = 0;
    uint32 off3 = 0;
    uint32 off4 = 0;

	off1 = (uint32)(103+angle1*1.52);  //270度舵机，每转动一度=1.52   0.5ms -180度起始位置：103
    off2 = (uint32)(103+angle2*1.52);
    off3 = (uint32)(103+angle3*1.52);
    off4 = (uint32)(103+angle4*1.52);
	PCA9685_setPWM(0,0,off1);
    PCA9685_setPWM(1,0,off2);
    PCA9685_setPWM(2,0,off3);
    PCA9685_setPWM(3,0,off4);

    
}