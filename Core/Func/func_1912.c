
#include "func_1912.h"
#include "conf_gpio.h"
void ClearForce_Device(uint8_t ID);
/**
  * @brief  ChengTian ONE-AXIS Force. Only collecting force, without data processing.
  * @hardware 	RS485 , DERE引脚与PD7相连，正点原子战舰开发板
  *	@			D:\10-Chip_learning\0-硬件\100-硬件资料\单轴力矩传感器
  * @condition 12V
  * @year 2020/08/28
  */


#define F1912_TEST

int iii;
#define HAL_Delay(time) 	OSTimeDlyHMSM(0, 0, 0, time);//iii=time*100;while(iii--);
#define INF 0

float F_1912 = INF;
uint8_t CommandSend[8] = {0x55,0xAA};			// send command
uint8_t CommandReceive[30] = {0}, length = 0;	// receive command
uint8_t receivebyte;							// receive byte from uart
uint8_t forceUpdate_flag = 0;
uint8_t myID = 0x01;							//1912 sensor ID

#define TX_ON GPIO_PIN_SET
#define RX_ON GPIO_PIN_RESET
void RS485_init(void)
{
	GPIO_InitTypeDef GPIO_Initure;
	__HAL_RCC_GPIOD_CLK_ENABLE();
	
	//PD7推挽输出，485模式控制  
    GPIO_Initure.Pin=GPIO_PIN_7; 			//PD7
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //推挽输出
    GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;//高速
    HAL_GPIO_Init(GPIOD,&GPIO_Initure);
}

void RS485_TXMode(void)
{
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, (GPIO_PinState)TX_ON);
}

void RS485_RXMode(void)
{
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, (GPIO_PinState)RX_ON);
}


void Force_1912_Init(int ID)
{
	myID = ID;
	MX_USART2_UART_Init();
	RS485_init();
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);	// 空闲中断
}


#include "func_epos.h"
void Force_1912_Start(void)
{
	RS485_RXMode();
	HAL_UART_Receive_IT(&huart2, &receivebyte, 1);
	//OSTimeDlyHMSM(0, 0, 20, 0);
	HAL_Delay(50);									//cannot use in the UCOSII
	ClearForce_Device(myID);
	HAL_Delay(500);
}



/**
  * @brief  Check the sum of the data start from "start" to "end" index of array "data". 
  *			Pay attention to that "end" index is included.
  * @param  data. where the data store
  * @param  start. where the sum start from. 0 means start from the first data.
  * @param  end. where the sum start end. it is included(calculated).
  * @retval 8bit sum
  */

uint8_t CheckSum(uint8_t *data, uint8_t start, uint8_t end){
	uint8_t sum = 0;
	for(uint8_t i=start; i<=end;i++)
	{
		sum += *(data+i);
	}
	return sum;
}

/**
  * @brief  Generate Device ID Set CommandSend and send out.
  * @param  oldID	The previous ID of Sevice
  * @param  ID 		The ID you want to set.
  */
void write_DeviceID(uint8_t oldID, uint8_t ID)
{
	CommandSend[2] = 0x03;
	CommandSend[3] = 0x01;	
	CommandSend[4] = oldID;
	CommandSend[5] = ID;	
	CommandSend[6] = CheckSum(CommandSend,2,5);
	RS485_TXMode();
	HAL_UART_Transmit_IT(&huart2, CommandSend, 7);
}

/**
  * @brief  set force to zero. Generate Clear force Protocol CommandSend and send out.
  * @param  ID 		The ID of Device.
  */
void ClearForce_Device(uint8_t ID)
{
	CommandSend[2] = 0x03;
	CommandSend[3] = 0x02;	
	CommandSend[4] = ID;	
	CommandSend[5] = 0;	
	CommandSend[6] = CheckSum(CommandSend,2,4);
	RS485_TXMode();
	HAL_UART_Transmit_IT(&huart2, CommandSend, 7);
}

/**
  * @brief  Read force data. Generate Read force Protocol CommandSend and send out.
  * @param  ID 		The ID of Device.
  */
void ReadForce_Device(uint8_t ID)
{
	CommandSend[2] = 0x03;
	CommandSend[3] = 0x03;
	CommandSend[4] = ID;	
	CommandSend[5] = 0;							//reserved
	CommandSend[6] = CheckSum(CommandSend,2,4);
	RS485_TXMode();
	HAL_UART_Transmit_IT(&huart2, CommandSend, 7);
}



/**
  * @brief  get the current force.and no offset is needed for clear function in command.
  */
#include "func_epos.h"

float getCurrentForce(void)
{
//	TEST_MSG("getCurrentForce:");
	
	ReadForce_Device(myID);
	if(forceUpdate_flag == 0){
		HAL_Delay(10);
	}
	forceUpdate_flag = 0;
	
//	TEST_MSG("\r\ngetCurrentForce end");
	
	return F_1912;
}



//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if(huart->Instance == USART2){
//		TEST_MSG("Usart2 rcv %d\r\n", length);
////		TEST_MSG("0x%x\t", receivebyte);
//		CommandReceive[length] = receivebyte;
//		length++;
//	}
////	TEST_MSG("\r\n");
//	HAL_UART_Receive_IT(&huart2, &receivebyte, 1);
//}


//void USAR_UART_IDLECallback(UART_HandleTypeDef *huart)
//{
//	if(huart->Instance == USART2)
//	{
//		TEST_MSG("USART2 IDLE IQR Detected: ");
//		
//		for(int i=0; i<length; i++){
//			TEST_MSG("0x%X\t", CommandReceive[i]);
//		}
//		TEST_MSG("\r\n");
//		
//		if(length == 8 && CommandReceive[3] == 0x33)
//		{
//			if(CheckSum(CommandReceive, 2, 6) == CommandReceive[7]){
//				int force = (int16_t)(CommandReceive[6]<<8|CommandReceive[5]);
//				F_1912 = force*0.01;
//				forceUpdate_flag = 1;
//				TEST_MSG("%f\r\n", F_1912);
//			}
//			else{
//				TEST_MSG("SUM Check Err\r\n");
//			}
//		}
//		length = 0;
//	}
//}

//void IDLE_UART_IRQHandler (UART_HandleTypeDef *huart)
//{
//	MYMSG("IDLE Callback %d\r\n",length);
//	if(huart->Instance == USART2)
//	{
//		MYMSG("IDLE1 \r\n");
//        if(RESET != __HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE))   // IDLE ?
//        {
//			MYMSG("IDLE2 \r\n");
//            __HAL_UART_CLEAR_IDLEFLAG(huart);                     //
//            USAR_UART_IDLECallback(huart);                          //
//        }
//	}
//}


#ifdef F1912_TEST

/**
  * @brief test the CheckSum function
  */
TEST test_CheckSum(void)
{
	uint8_t sum = 0;
	uint8_t test[] = {0x55, 0xAA, 0x04, 0x33, 0x01, 0x23, 0x09, 0x64};
	sum = CheckSum(test, 2, 6);
	TEST_MSG("Get 0x%x ans=0x64H\r\n", sum);
}

/**
  * @brief print command on the uart. Check whether it is right. 
  */
TEST Prinf_Command(uint8_t * CommandSend)
{
		TEST_MSG("CommandSend ");
		for(uint8_t i=0;i<8;i++){
			TEST_MSG("0x%X\t", CommandSend[i]);
		}	
		TEST_MSG("\r\n");
}



TEST test_searchID(void)
{
	uint8_t myID = 0x01;
	Force_1912_Init(myID);
	Force_1912_Start();
	
	//Prinf_Command(CommandSend);
	
	while(1){
		write_DeviceID(myID, 0x01);
		Prinf_Command(CommandSend);
		myID++;
		HAL_Delay(100);
		if(myID == 0xFF){
			myID = 0x01;
		}
	}
}



TEST test_GetForce(void)
{
	uint8_t myID = 0x01;
	Force_1912_Init(myID);
	Force_1912_Start();
	HAL_Delay(50);
	ClearForce_Device(myID);
	HAL_Delay(50);

	while(1){
		HAL_Delay(10);
		ReadForce_Device(myID);		//waiting response
//		Prinf_Command(CommandSend);
	}
	
}


TEST test_GetCurrentForce(void)
{
	float Currentforce = 0.0;
	Force_1912_Init(0x01);
	Force_1912_Start();

	while(1){
		HAL_Delay(10);
		Currentforce = getCurrentForce();			//waiting response
		MSG("GetForce - %.4f \r\n", Currentforce);
	}
}
	
/**
  * @brief  方法一：空闲中断参考，将数据看做是一份一份的，这样每一份根据协议去操作就比较简单了
  *                     参考 https://blog.csdn.net/qq_17351161/article/details/90415407#_27
  *         方法二：将接收到的数据看作是数据流，利用协议头帧0x55,0xAA+数据长度+功能码+对应功能的数据+校验和，去一一解析
  * @param  hdac pointer to a DAC_HandleTypeDef structure that contains
  *         the configuration information for the specified DAC.
  * @param  Channel The selected DAC channel.
  *          This parameter can be one of the following values:
  *            @arg DAC_CHANNEL_1: DAC Channel1 selected
  *            @arg DAC_CHANNEL_2: DAC Channel2 selected
  * @param  Alignment Specifies the data alignment.
  *          This parameter can be one of the following values:
  *            @arg DAC_ALIGN_8B_R: 8bit right data alignment selected
  *            @arg DAC_ALIGN_12B_L: 12bit left data alignment selected
  *            @arg DAC_ALIGN_12B_R: 12bit right data alignment selected
  * @param  Data Data to be loaded in the selected data holding register.
  * @retval HAL status
  */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2){
		TEST_MSG("0x%x\t", receivebyte);
		CommandReceive[length] = receivebyte;
		length++;
	}
	TEST_MSG("\r\n");
	HAL_UART_Receive_IT(&huart2, &receivebyte, 1);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2){
		RS485_RXMode();
	}
}


TEST GetForce(UART_HandleTypeDef *huart)
{
	if(length == 8 && CommandReceive[3] == 0x33)
	{
		if(CheckSum(CommandReceive, 2, 6) == CommandReceive[7]){
			int force = (int16_t)(CommandReceive[6]<<8|CommandReceive[5]);
			F_1912 = force*0.01;
			TEST_MSG("Fq=%f\r\n", F_1912);
		}
		else{
			TEST_MSG("SUM Check Err\r\n");
		}
	}
}


TEST USAR_UART_IDLECallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2)
	{
//		TEST_MSG("USART2 IDLE IQR Detected: ");
//		
//		for(int i=0; i<length; i++){
//			TEST_MSG("0x%X\t", CommandReceive[i]);
//		}
//		TEST_MSG("\r\n");
		TEST_MSG("length = %d\r\n", length);
		
		GetForce(huart);
		
		length = 0;
	}
	
}


void IDLE_UART_IRQHandler (UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2)
	{
        if(RESET != __HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))   // IDLE ?
        {
            __HAL_UART_CLEAR_IDLEFLAG(huart);                     //
            USAR_UART_IDLECallback(huart);                          //
        }
	}
}

#endif
