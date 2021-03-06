/*
 * canopen-interface.c
 *
 *  Created on: Mar 5, 2020
 *      Author: test
 */

#include "canopen_interface.h"

/*
 * author lhx
 *
 * @brief : statement in state.c.  callback when Already into preOperational state.
 * Window > Preferences > C/C++ > Editor > Templates.
 */

void _preOperational(CO_Data* d){

	MSG("complete preOperational\r\n");

}

void _initialisation(CO_Data* d)
{
	(void)d;
	MSG("complete initialization\r\n");

}
void _operational(CO_Data* d)
{
	(void)d;
	MSG("complete operational\r\n");
}
void _stopped(CO_Data* d)
{
	(void)d;
	MSG("complete stop\r\n");
}


/*
 * author lhx
 *
 * @brief : after sendPDOevent, so at beginning ,the first PDO's value is configured in the dict.
 * Window > Preferences > C/C++ > Editor > Templates.
 */


void assive (CO_Data* d);
void Test_curve(CO_Data* d);
void sin_cos_test (CO_Data* d);

void _post_TPDO(CO_Data* d)
{
	//assive(d);
	//sin_cos_test(d);
}



#include "gait.h"


#define ARRAY_K   knee_flexion
#define ARRAY_H   hip_flexion

int PERIOD = 0;							// 设定运行次数
int period = 0;						// 实际运行次数
uint16_t endP = 0;
uint8_t Index = 0;
int x=0, temp_x;						//extern int x=0;语法错误

extern INTEGER32 Pos_Actual_Val_node1, Pos_Actual_Val_node2, Pos_Actual_Val_node3, Pos_Actual_Val_node4, Pos_Actual_Val_node5, Pos_Actual_Val_node6;
extern INTEGER32 Actual_Velocity_VALUE_node1, Actual_Velocity_VALUE_node2, Actual_Velocity_VALUE_node3, Actual_Velocity_VALUE_node4, Actual_Velocity_VALUE_node5, Actual_Velocity_VALUE_node6;
extern INTEGER16 Current_Actual_Val_node2;
extern INTEGER32 Pos_SET_VALUE_node3;

Uint32 Position[6];
float Position_float[6];
int Position_int[6];

int epos_state = 50;

#include "func_CanOpen.h"
#define QC_TO_Degree_EC90 1//1820.44
#define QC_TO_Degree_RE40 1//2222.22
void assive (CO_Data* d)
{
	if(PERIOD != 0){
		
		endP = sizeof(ARRAY_H)/sizeof(*ARRAY_H);				// period size
		
		//Position_float[0] = (ARRAY_H[x]*QC_TO_Degree_EC90);
		Position_int[0] = (ARRAY_H[x]*QC_TO_Degree_EC90);		// postive means go forward
		Position[0] = (Uint32)Position_int[0];					//for node2
		
		//Position_float[1] = (ARRAY_K[x]*QC_TO_Degree_EC90);
		Position_int[1] = (ARRAY_K[x]*QC_TO_Degree_EC90);
		Position[1] = (Uint32)Position_int[1];					//for node3 counterclkwise
		
		temp_x = x + endP/2;									//start form half period. another side
		if(temp_x >= endP){
			temp_x = temp_x - endP;
		}
		
		//Position_float[2] = (ARRAY_H[temp_x]*QC_TO_Degree_EC90);
		Position_int[2] = (ARRAY_H[temp_x]*-QC_TO_Degree_EC90);
		Position[2] = (Uint32)Position_int[2];					//for node4
		
		//Position_float[3] = (ARRAY_K[temp_x]*QC_TO_Degree_RE40);
		Position_int[3] = (knee_flexion_RE40[temp_x]*QC_TO_Degree_RE40);
		Position[3] = (Uint32)Position_int[3];					//for node5  clkwise

		x++;
		if( x == endP){
			x = 0;
			period++;
			MYMSG("#%d\t%d\r\n",period,PERIOD);
		}
		
		Edit_Dict(d , 0x20620020, 0x00, &Position[0]);//Pos_SET_VALUE node_1
		Edit_Dict(d , 0x20630020, 0x00, &Position[1]);
		Edit_Dict(d , 0x20640020, 0x00, &Position[2]);
		Edit_Dict(d , 0x20650020, 0x00, &Position[3]);
		
		for(uint8_t i = 0;i<4;i++){
			ROW_MSG("%d\t", Position_int[i]);
		}
		//ROW_MSG("%d\t%d\t%d\t%d\r\n",Pos_Actual_Val,Pos_Actual_Val_node3,Pos_Actual_Val_node4, Pos_Actual_Val_node5);
	}
	
}

Uint32 pos;
int subI = 0;
void sin_cos_test (CO_Data* d)
{
	
	pos = (int)(30000*sin(5.0f*3.14f*0.005f*subI));
	
	//Edit_Dict(d , 0x20620020, 0x00, &pos);
	//Edit_Dict(d , 0x20630020, 0x00, &pos);
	//Edit_Dict(d , 0x20640020, 0x00, &pos);
	Edit_Dict(d , 0x20650020, 0x00, &pos);
	Edit_Dict(d , 0x20660020, 0x00, &pos);
	
	subI++;
	//ROW_MSG("%d\t%d\t%d\t%d\t%d\r\n",Pos_Actual_Val,Pos_Actual_Val_node3,Pos_Actual_Val_node4, Pos_Actual_Val_node5,pos);
}

void Test_curve (CO_Data* d)
{
	endP = sizeof(test_angle)/sizeof(*test_angle);
	pos = test_angle[x++]*20;
	if( x == endP){
		x = 0;
	}
	
	//re = Edit_Dict(d , 0x20620020, 0x00, &pos);
	//re = Edit_Dict(d , 0x20630020, 0x00, &pos);
	//re = Edit_Dict(d , 0x20640020, 0x00, &pos);
	Edit_Dict(d , 0x20650020, 0x00, &pos);
	Edit_Dict(d , 0x20660020, 0x00, &pos);
	
	ROW_MSG("%d\t%d\t%d\t%d\t%d\r\n",Pos_Actual_Val_node5, Pos_Actual_Val_node6, Actual_Velocity_VALUE_node5, Actual_Velocity_VALUE_node6, pos);
}


/*
 * author lhx
 *
 * @brief : in SYNC but before _sendPDOevent.
 * Window > Preferences > C/C++ > Editor > Templates.
 */


#include "pid.h"
extern PID_Regulator_t forceControlPID;


float CalI_target(float f_control ,float currentforce)
{
	forceControlPID.fdb = currentforce;
	forceControlPID.ref = f_control;
	PID_Calc(&forceControlPID);
	float I_control = forceControlPID.output;
	
	
	I_control = f_control;	//no torque feedback
	
	
	return I_control;
}


typedef struct
{
	float theta;
	float theta_dot;
	float theta_ddot;
	float torque;
} MotorState_t;

MotorState_t desiredTarget = {0,0,0,0};
MotorState_t CurrentState = {0};
float M = 0.01, L=0.3, J = 0.0037;
float wn=3,yita=0.707;
float Jm = 0.0037, Dm, Km;//2/3perfect

float CalForce_target(MotorState_t * MotorStateTarget, MotorState_t * CurrentMotorState, float currentContactForce)
{
	Dm = 2*yita*wn;
	Km = wn*wn;
	float f_control = J*MotorStateTarget->theta_ddot+
		J/Jm*(Dm*(MotorStateTarget->theta_dot-CurrentMotorState->theta_dot) + Km*(MotorStateTarget->theta-CurrentMotorState->theta))+
		(J/Jm - 1)*currentContactForce;
	
	return f_control;
}

float currenttheta, currenttheta_dot, currentforce;	//单位 

extern int32_t Pos_Actual_Val_node5, Actual_Velocity_VALUE_node5,Current_Actual_Val_node5, Actual_AVRVelocity_VALUE_node5;
extern int16_t Actual_Torque_VALUE_node5, Torque_SET_VALUE_node5;

void getMotorState(void)
{
	uint8_t time = 50;
	
	while(Pos_Actual_Val_node5 == 12358 && time >0){				//make sure data did refresh
		time --;
	}
	CurrentState.theta = Pos_Actual_Val_node5*360.0/4096.0/4.0;	//CANOpen dict 0x6064 convert to degree
	
	while(Actual_AVRVelocity_VALUE_node5 == 12358 && time < 50){
		time ++;
	}
	CurrentState.theta_dot = Actual_AVRVelocity_VALUE_node5;		//CANOpen dict 0x606C rpm
	
	while(Actual_Torque_VALUE_node5 == 12358 && time > 0){
		time --;
	}
	CurrentState.torque = Actual_Torque_VALUE_node5;			//CANOpen dict 	mNm
	
	//Current_Actual_Val_node5 mA
	printf("ActualState %.2f\t%.2f\t%.2f\t%d\t", CurrentState.theta, CurrentState.theta_dot, CurrentState.torque, Current_Actual_Val_node5);
	Pos_Actual_Val_node5=12358;
	Actual_AVRVelocity_VALUE_node5=12358;
	Actual_Torque_VALUE_node5 = 12358;
}

void _post_sync(CO_Data* d)
{
	(void)d;
	SYNC_MSG("-post_sync-\r\n");
	//waiting for sensor information
	//assive(d);
	//sin_cos_test(d);
	//Test_curve(d);

	getMotorState();
	// impedance control algorithm
	float f_control = CalForce_target(&desiredTarget, &CurrentState, 0);	//mNm
	// force control algorithm
	float I_control = CalI_target(f_control , CurrentState.torque);			//mA
	
//	if(I_control>0 && (CurrentState.theta<2 ||CurrentState.theta>-2  )){
//		I_control += 30;
//	}
//	if(I_control<0 && (CurrentState.theta<2 ||CurrentState.theta>-2  )){
//		I_control -= 30.0;
//	}
	
	if(I_control>500){
		I_control = 500.0;
	}
	if(I_control<-500){
		I_control = -500.0;
	}
	
	printf("Command %.2f\r\n", I_control);
	//I_control = -300;
	int I_temp = I_control;
	Torque_SET_VALUE_node5 = (int16_t)I_temp ;
	
	#ifdef REMOTE_APP
    if(Stop == 1){
         EPOSMaster_PDOStop();
         PeriodRun = 0;
    }
    else if(PeriodRun == 1){
        if(period > PERIOD){
            EPOSMaster_PDOStop();
            Stop = 1;Reset = 1;
        }
    }
	#endif
}



/*
 * author lhx
 *
 * @brief : life guard
 * Window > Preferences > C/C++ > Editor > Templates.
 */

void _heartbeatError(CO_Data* d, UNS8 heartbeatID)
{
	(void)d;
	(void)heartbeatID;
	MSG_WAR(0x44, "heart beat error", heartbeatID);
	EposMaster_Stop();
}
void _post_SlaveBootup(CO_Data* d, UNS8 SlaveID){(void)d;(void)SlaveID;}
void _post_SlaveStateChange(CO_Data* d, UNS8 nodeId, e_nodeState newNodeState){(void)d;(void)nodeId;(void)newNodeState;}
void _nodeguardError(CO_Data* d, UNS8 id){(void)d;(void)id;}


/*
 * author lhx
 *
 * @brief : receive a emcy
 * Window > Preferences > C/C++ > Editor > Templates.
 */
void _post_emcy(CO_Data* d, UNS8 nodeID, UNS16 errCode, UNS8 errReg, const UNS8 errSpec[5])
{
	(void)d;
	(void)nodeID;
	(void)errCode;
	(void)errReg;
	(void)errSpec;
}
