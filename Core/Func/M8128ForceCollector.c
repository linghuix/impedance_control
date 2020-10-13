#include "M8128ForceCollector.h"
//#define Force_Collect_TEST

#define FORCENUMMAX 27

uint8_t force[FORCENUMMAX];	// force raw data(hex of float) store 
float measureForce[6];		// force data store
CanTxMsg ForceCommand;		// command can frame
CanRxMsg ForceData;			// raw data can frame


//---------------------------------------------------------
// Brief - break the string into 8-byte packages and send ASCII of them through CAN frame. BIG-END Mode
// for example - To send 0x12 34 56 78 12 34 56 78 12 23 45
//             - can frame 1: 0x12 34 56 78 12 34 56 78 
//			   - can frame 2: 0x12 23 45 00 00 00 00 00 
//---------------------------------------------------------
void SendString (char * str)
{	
	uint8_t  i;
	uint32_t mailbox = 0;
	for(i=0; str[i]!='\0'; i++)
	{
		ForceCommand.Data[i%8] = str[i];
		if((i+1)%8 == 0){
			while(MX_CANx_send(&hcan1, &ForceCommand, mailbox) != HAL_OK);	//8 data per package
		}
//		printf("%d\r\n",i);
	}
//	printf("gg %d  %d\r\n",i,i%8);
	if((i)%8 != 0){									//send the rest data
		for(; i%8!=7; i++){
//			printf("%d\r\n",i);
			ForceCommand.Data[i%8] = 0x00;
		}
		ForceCommand.Data[i%8] = 0x00;
		
		while(MX_CANx_send(&hcan1, &ForceCommand, mailbox) != HAL_OK);
	}
}


void ForceCollector_Init (void)
{
	FORCE_MSG("ForceCollector init\r\n");
	for(int i=0; i<FORCENUMMAX; i++){
		force[i] = 0;
	}
	
//	MX_CAN1_Init(CAN_MODE_NORMAL);
//	CAN_Start(&hcan1);
	
	ForceCommand.head.StdId = 0x010;
	ForceCommand.head.DLC = 8;
	ForceCommand.head.RTR = CAN_RTR_DATA;
	ForceCommand.head.IDE = CAN_ID_STD;
	
	char ForceStartCommand[] = "AT+SMPF=200\r\n";
	SendString (ForceStartCommand);
}



void StartCollect (void)
{	
	FORCE_MSG("start collect\r\n");
	char ForceStartCommand[] = "AT+GSD\r\n";		// command string
	SendString (ForceStartCommand);
}


void StopCollect (void)
{	
	char ForceStartCommand[] = "AT+GSD=STOP\r\n";
	SendString (ForceStartCommand);
}


struct ForceBuffer forceBuffer = {{0.0,0.0,0.0,0.0,0.0},0,0};
/*
 * author lhx
 * May 13, 2020
 *
 * @brief : put data to buffer, buffer is FULL return 0.
 * Window > Preferences > C/C++ > Editor > Templates.
 */
uint8_t addForceBuffer(float data)
{
//	if((forceBuffer.in+1)%ForceBufferSize == forceBuffer.out)
//		return 0;
	forceBuffer.data[forceBuffer.in] = data;
	forceBuffer.in = (forceBuffer.in + 1)%ForceBufferSize;
	return 1;
}


//---------------------------------------------------------
// @brief: get the index-1 for queue. 
//---------------------------------------------------------

uint16_t getPreviousIndex(uint16_t index)
{

	uint16_t previous;
	if((uint16_t)(index-1) > index) {
		previous = (ForceBufferSize-1);
	}
	else{
		previous = (index-1);
	}
	//MMSG("index=%d previous=%d\r\n", index, previous);
	return previous;
}


//---------------------------------------------------------
// @brief: 添加基于之前数据的加权 add data weighted by previous data.
// @call : getPreviousIndex
//---------------------------------------------------------
uint8_t addWeightingForceBuffer(float data)
{
	uint16_t previous = getPreviousIndex(forceBuffer.in);
	uint16_t pprevious = getPreviousIndex(previous);
	forceBuffer.data[forceBuffer.in] = 0.9*data+0.1*forceBuffer.data[previous]+0*forceBuffer.data[pprevious];
	forceBuffer.in = (forceBuffer.in + 1)%ForceBufferSize;
	return 1;
}


extern float Offset;
float getMedianForce(void)
{
	uint16_t previous = getPreviousIndex(forceBuffer.in);
	uint16_t pprevious = getPreviousIndex(previous);
	float median = 0;
	if(forceBuffer.data[previous] > forceBuffer.data[pprevious])
	{
		if(median > forceBuffer.data[previous]){
			median = forceBuffer.data[previous];
		}
		else if(median < forceBuffer.data[pprevious]){
			median = forceBuffer.data[pprevious];
		}
	}
	else {
		if(median < forceBuffer.data[previous]){
			median = forceBuffer.data[previous];
		}
		else if(median > forceBuffer.data[pprevious]){
			median = forceBuffer.data[pprevious];
		}
	}

	return median - Offset;
}

//---------------------------------------------------------
// get the latest store weighted data which is filtered in addWeightingForceBuffer function.
// @call : getPreviousIndex
//---------------------------------------------------------
float getfilteredForce(void)
{
	return forceBuffer.data[getPreviousIndex(forceBuffer.in)] - Offset;
}

//---------------------------------------------------------
// brief - get average of the previous ForceBufferSize-number force data. 
// 		It is a average filiter and Regard average force as true force.
//---------------------------------------------------------

float AvgForceBuffer(void)
{
	float sum = 0;
	for(int i=0;i<ForceBufferSize;i++){
		sum += forceBuffer.data[i];
	}
	return sum/ForceBufferSize;
}

uint32_t n=0;		// num order of force data
float Offset=0;
float GetOffset(void)
{
	if(n == 500){
		Offset = getfilteredForce();
		//Offset += AvgForceBuffer();
		FORCE_MSG("------------------ offset get --------------------\r\n");
	}
	return Offset;
}

//---------------------------------------------------------
// brief -  get the current force.and of course offset is needed.
//---------------------------------------------------------

float getCurrentForce(void)
{
	return AvgForceBuffer()-Offset;
}


uint8_t OneframeDetected = 0;					// flag for one forcedata frame receive mark
uint16_t ordernum, dataLength;					// forcedata frame structure
uint16_t forceIndex = 0;						// row data index forstore 
void forceDispatch(CanRxMsg * ForceData)
{
	if(ForceData->Data[0] == 0xAA && ForceData->Data[1] == 0x55){		// one frame head
		FORCE_COMMUNICATE_MSG("receive force frame");
		forceIndex = 0;
		OneframeDetected = 1;
		dataLength = ((ForceData->Data[2]<<8)| ForceData->Data[3]) - 2;
		ordernum = (ForceData->Data[4]<<8) | ForceData->Data[5];
		while( (6+forceIndex) < 8){
			force[forceIndex] = ForceData->Data[6+forceIndex];
			forceIndex++; dataLength--;
		}
	}
	else{
		uint32_t index=0;
		while( index < 8 && dataLength != 0){
			force[forceIndex] = ForceData->Data[index];
			index++;forceIndex++;
			dataLength--;
		}
	}


	if(dataLength == 0 && OneframeDetected == 1){
		
		FORCE_DECODE_MSG("%d-%d\t\r\n",n,ordernum);
		float * f;
		int data;
		for(uint8_t j=5;j<6;j++){
			data = force[4*j] | force[4*j+1]<<8 | force[4*j+2]<<16 | force[4*j+3]<<24;
			f = (float*) &data;
			if(j==5){
				//addForceBuffer(*f);
				addWeightingForceBuffer(*f);
			}
	//				measureForce[j] = *f;
	//				printf("%.2f=%X\t", *f, data);
			FORCE_DECODE_MSG("\t%.1f\t", *f);
			FORCE_DECODE_MSG("%.2f\t", AvgForceBuffer()-GetOffset());
			FORCE_DECODE_MSG("offset%.3f", GetOffset());
		}
		FORCE_DECODE_MSG("\r\n");
		OneframeDetected = 0;
		
		GetOffset();
		++n;
		
		/*if(++n >= 10){
			StopCollect();
		}*/
	}
}


void WaitForCalibration(void)
{
	MMSG("WaitFor force Calibration\r\n");
	while(Offset == 0 && n < 300){
		OSTimeDlyHMSM(0, 0,0,2);		// can frame need to send while waiting for calibration
	}
	OSTimeDlyHMSM(0, 0,0,50);			// without it the following MMSG cannot print.
	MMSG("force Calibration finish\r\n");

}



#ifdef Force_Collect_TEST

TEST SendString_test(void)
{
	ForceCollector_Init();
	char ForceStartCommand1[] = "AT+GSD=STOP\r\n";	// 41 54 2B 47 53 44 3D 53		54 4F 50 0D 0A 00 00 00
	SendString (ForceStartCommand1);
	char ForceStartCommand2[] = "AT+GSD\r\n";		// 41 54 2B 47 53 44 0D 0A
	SendString (ForceStartCommand2);
	char ForceStartCommand3[] = "AT+SMPF=?\r\n";	// 41 54 2B 53 4D 50 46 3D		3F 0D 0A 00 00 00 00 00
	SendString (ForceStartCommand3);
	
	char ForceStartCommand5[] = "\r\n";				// send data length less than 8
	SendString (ForceStartCommand5);
	
	char ForceStartCommand4[] = "good for you, you ashole!\r\n";			// send data length more than 8
	SendString (ForceStartCommand4);
	char ForceStartCommand6[] = "";					//send nothing
	SendString (ForceStartCommand6);
}



TEST ForceCollector_test(void)
{
	ForceCollector_Init();
	StartCollect();
}
	

uint8_t OneframeDetected = 0;
uint16_t ordernum, dataLength,forceIndex = 0;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint32_t index=0;
	if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0)>0) {
		int canerror = MX_CANx_get(&hcan1, &ForceData, CAN_RX_FIFO0);
		
		if(ForceData.Data[0] == 0xAA && ForceData.Data[1] == 0x55){		// one frame 
			forceIndex = 0;
			OneframeDetected = 1;
			dataLength = ((ForceData.Data[2]<<8)| ForceData.Data[3]) - 2;
			ordernum = (ForceData.Data[4]<<8) | ForceData.Data[5];
			while( (6+forceIndex) < 8){
				force[forceIndex] = ForceData.Data[6+forceIndex];
				forceIndex++; dataLength--;
			}
		}
		else{
			while( index < 8 && dataLength != 0){
				force[forceIndex] = ForceData.Data[index];
				index++;forceIndex++;
				dataLength--;
			}
		}
		
		
		if(dataLength == 0 && OneframeDetected == 1){
			
			printf("%d-%d\t",n,ordernum);
			float * f;
			int data;
			for(uint8_t j=5;j<6;j++){
				data = force[4*j] | force[4*j+1]<<8 | force[4*j+2]<<16 | force[4*j+3]<<24;
				f = (float*) &data;
				if(j==5){
					addForceBuffer(*f);
				}
//				measureForce[j] = *f;
//				printf("%.2f=%X\t", *f, data);
				printf("%.1f\t", *f);
				printf("%.2f\t", AvgForceBuffer()-GetOffset());
				printf("offset%.3f", GetOffset());
			}
			printf("\r\n");
			OneframeDetected = 0;
			
			++n;
			
			/*if(++n >= 10){
				StopCollect();
			}*/
		}
	}
}


#endif
