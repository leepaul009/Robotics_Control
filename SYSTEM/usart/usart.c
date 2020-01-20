#include "usart.h"	  
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
	/* Whatever you require here. If the only file you are using is */ 
	/* standard output using printf() for debugging, no file handling */ 
	/* is required. */ 
}; 
/* FILE is typedef’ d in stdio.h. */ 
FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
_sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      

//	while((USART2->SR&0X40)==0);
//	USART2->DR = (u8) ch;      
  return ch;
}
#endif 
int Usart_Receive;
int Usart_Receive_Last;

//////////////////////////////////////////////////////////////////
/**************************实现函数**********************************************
*功    能:		usart1发送一个字节
*********************************************************************************/
void usart2_send(u8 data)
{
	USART2->DR = data;
	while((USART2->SR&0x40)==0);	
}
void uart2_init(u32 bound)
{  	 
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
 
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE);	//使能GPIOA时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);	//使能USART时钟
	//USART2_TX   GPIOA.2
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA.2
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.2
   
  //USART2_RX	  GPIOA.3初始化
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PA3
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.3  
   //USART 初始化设置

	  //UsartNVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
	
	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

  USART_Init(USART2, &USART_InitStructure); //初始化串口2
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启串口接受中断
  USART_Cmd(USART2, ENABLE);                    //使能串口2 
}
/**************************************************************************
函数功能：串口2接收中断
入口参数：无
返回  值：无
************************************************************************** */
typedef struct{
	uint8_t B0;
	uint8_t B1;
	uint8_t B2;
	uint8_t B3;	
}FLOAT,INT32,UINT32;


typedef union{
	float f1;
  uint32_t u32;
	int32_t s32;
	uint16_t u16[2];
	int16_t s16[2];
	uint8_t u8[4];
	int8_t s8[4];
}DataConverter;


DataConverter data_converter;
FLOAT test_source;
//--------------------------------------//
float Uint8ToFloat(FLOAT *source)
{
	data_converter.u8[0] = source->B0;
	data_converter.u8[1] = source->B1;
	data_converter.u8[2] = source->B2;
	data_converter.u8[3] = source->B3;
	return data_converter.f1;
}
//--------------------------------------//
void FloatToUint8(float source, FLOAT *target)
{
	data_converter.f1 = source;
	
	target->B0 = data_converter.u8[0];
	target->B1 = data_converter.u8[1];
	target->B2 = data_converter.u8[2];
	target->B3 = data_converter.u8[3];
}
//--------------------------------------//

#define RX_BUFFER_LENGTH 50
unsigned char rx_buffer[RX_BUFFER_LENGTH];  // Get commands from simulator
volatile unsigned char rx_head = 0;
volatile unsigned char rx_end = 0;
volatile unsigned char first_data_received = 0;

//---------------------------------//
float position_x,position_y,position_z;
float  roll, pitch, yaw;
unsigned char test_ptr;  

//rx_buffer reader 
static uint8_t reader_head = 0;
static uint8_t reader_end = 0;
	uint8_t boundary0 ;
	uint8_t boundary1 ;

unsigned char TxIMUData(void)
{
	
}


unsigned char CommandConfirmedPocess(void)
{
   
	 boundary0 = rx_buffer[rx_head % RX_BUFFER_LENGTH];
	 boundary1 = rx_buffer[(rx_head+1)% RX_BUFFER_LENGTH];
	 
	if(boundary0==0x55&&boundary1==0xaa){
		reader_head = (rx_head+2)% RX_BUFFER_LENGTH;
		reader_head += 9;//seq, funId, ts(4), payloadlen(2), versionId
		
		
		test_source.B0 = rx_buffer[(rx_head++) % RX_BUFFER_LENGTH];
		test_source.B1 = rx_buffer[(rx_head++) % RX_BUFFER_LENGTH];
		test_source.B1 = rx_buffer[(rx_head++) % RX_BUFFER_LENGTH];
		test_source.B1 = rx_buffer[(rx_head++) % RX_BUFFER_LENGTH];
		position_x = Uint8ToFloat(&test_source);	
		reader_head = (rx_head+3)% RX_BUFFER_LENGTH;
		return 1;
		
	}
	else {
		reader_head++;// = (reader_head+2)% RX_BUFFER_LENGTH;
		return 0;
		//skip
	}
	
}
//-----------------------------------------------------//
//-------------------------------------------------//
uint8_t flagReadPayLoad;
//uint8_t flagReadPayLoad;
uint8_t Usart_Receive_Test;
int indexDataRead = 0;
void USART2_IRQHandler(void)
{	
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) //接收到数据
	{	       
		  static u8 Flag_PID,i,j,Receive[50];
			static float Data;
  	  Usart_Receive=USART2->DR; 
		Usart_Receive_Test  = Usart_Receive;
			rx_buffer[rx_end] = Usart_Receive;
	    if (++rx_end >= RX_BUFFER_LENGTH) {
	        rx_end = 0;
	    }
	
	    if (rx_end == rx_head) {
	        if (++rx_head >= RX_BUFFER_LENGTH) {
	            rx_head = 0;
	        }
			}
			//-------------------------------------------------------------//
			if(flagReadPayLoad)
			{
				++indexDataRead;//start with Sequence(1)/Function(1)/TimeStamp(4)/PayloadLength(2)/Payload(n)->1byte versionId/Checksum(1)/FixTail(2)/
				//--------Position_x
				if(indexDataRead == 9)
				{
					test_source.B0 = Usart_Receive_Test;
				}
				else if(indexDataRead == 10)
				{
					test_source.B1 = Usart_Receive_Test;
				}
				else if(indexDataRead == 11)
				{
					test_source.B2 = Usart_Receive_Test;
				}
				else if(indexDataRead == 12)
				{
					test_source.B3 = Usart_Receive_Test;
					position_x = Uint8ToFloat(&test_source);
				}
				//--------Position_y
				if(indexDataRead == 13)
				{
					test_source.B0 = Usart_Receive_Test;
				}
				else if(indexDataRead == 14)
				{
					test_source.B1 = Usart_Receive_Test;
				}
				else if(indexDataRead == 15)
				{
					test_source.B2 = Usart_Receive_Test;
				}
				else if(indexDataRead == 16)
				{
					test_source.B3 = Usart_Receive_Test;
					position_y = Uint8ToFloat(&test_source);
				}				
				//--------Position_z
				if(indexDataRead == 17)
				{
					test_source.B0 = Usart_Receive_Test;
				}
				else if(indexDataRead == 18)
				{
					test_source.B1 = Usart_Receive_Test;
				}
				else if(indexDataRead == 19)
				{
					test_source.B2 = Usart_Receive_Test;
				}
				else if(indexDataRead == 20)
				{
					test_source.B3 = Usart_Receive_Test;
					position_z = Uint8ToFloat(&test_source);
				}					
				//--------Roll
				if(indexDataRead == 21)
				{
					test_source.B0 = Usart_Receive_Test;
				}
				else if(indexDataRead == 22)
				{
					test_source.B1 = Usart_Receive_Test;
				}
				else if(indexDataRead == 23)
				{
					test_source.B2 = Usart_Receive_Test;
				}
				else if(indexDataRead == 24)
				{
					test_source.B3 = Usart_Receive_Test;
					roll = Uint8ToFloat(&test_source);
				}		
				//--------Pitch
				if(indexDataRead == 25)
				{
					test_source.B0 = Usart_Receive_Test;
				}
				else if(indexDataRead == 26)
				{
					test_source.B1 = Usart_Receive_Test;
				}
				else if(indexDataRead == 27)
				{
					test_source.B2 = Usart_Receive_Test;
				}
				else if(indexDataRead == 28)
				{
					test_source.B3 = Usart_Receive_Test;
					pitch = Uint8ToFloat(&test_source);
				}			
				//--------Yaw
				if(indexDataRead == 29)
				{
					test_source.B0 = Usart_Receive_Test;
				}
				else if(indexDataRead == 30)
				{
					test_source.B1 = Usart_Receive_Test;
				}
				else if(indexDataRead == 31)
				{
					test_source.B2 = Usart_Receive_Test;
				}
				else if(indexDataRead == 32)
				{
					test_source.B3 = Usart_Receive_Test;
					yaw = Uint8ToFloat(&test_source);
					flagReadPayLoad = 0;
				}	
							
				if(indexDataRead > 33)
				{
					indexDataRead = 0;
					flagReadPayLoad = 0;
				}
			}			

			
			
			if(Usart_Receive_Last == 0x55 && Usart_Receive_Test == 0xAA && !flagReadPayLoad)
			{
				flagReadPayLoad = 1;
				indexDataRead = 0;
			}
			
			
			Usart_Receive_Last = Usart_Receive;
//			if(reader_head==0&&reader_end==0&&first_data_received==0){
//			first_data_received=1;
//			reader_head = rx_head;
//			reader_end = rx_end;
//		}
//---------------------------------------------------//
/*		
			if(Usart_Receive>=0x41&&Usart_Receive<=0x48)  
			Flag_Direction=Usart_Receive-0x40;
			else 	  
			Flag_Direction=0;	
		//以下是与APP调试界面通讯
		if(Usart_Receive==0x7B) Flag_PID=1;   //APP参数指令起始位
		if(Usart_Receive==0x7D) Flag_PID=2;   //APP参数指令停止位

		 if(Flag_PID==1)  //采集数据
		 {
			Receive[i]=Usart_Receive;
			i++;
		 }
		 if(Flag_PID==2)  //分析数据
		 {
			     if(Receive[3]==0x50) 	 PID_Send=1;
					 else  if(Receive[1]!=0x23) 
					 {								
						for(j=i;j>=4;j--)
						{
						  Data+=(Receive[j-1]-48)*pow(10,i-j);
						}
						switch(Receive[1])
						 {
							 case 0x30:  RC_Velocity=Data;break;
							 case 0x31:  Velocity_KP=Data;break;
							 case 0x32:  Velocity_KI=Data;break;
							 case 0x33:  break; //预留
							 case 0x34:  break; //预留
							 case 0x35:  break; //预留
							 case 0x36:  break; //预留
							 case 0x37:  break; //预留
							 case 0x38:  break; //预留
						 }
					 }				 
					 Flag_PID=0;//相关标志位清零
					 i=0;
					 j=0;
					 Data=0;
					 memset(Receive, 0, sizeof(u8)*50);//数组清零
		 } 	 
*/
   }
}



/**************************************************************************
函数功能：串口扫描
**************************************************************************/
u8 click_RC (void)
{
			static u8 flag_key=1;//按键按松开标志
	    u8 temp;
			if(flag_key&&Usart_Receive!=0x5A)
			{
			flag_key=0;
		  if(Usart_Receive>=0x01&&Usart_Receive<=0x08)temp=Usart_Receive;
		  else	if(Usart_Receive>=0x41&&Usart_Receive<=0x48)temp=Usart_Receive-0x40;	
		//	else 	temp=0;
			return temp;	// 按键按下
			}
			else if(Usart_Receive==0x5A)			flag_key=1;
			return 0;//无按键按下
}


/**************************************************************************
函数功能：通过串口把自身的传感器发送出去
**************************************************************************/
float sourceTest;
FLOAT targetTest;
void USART_TX(void)
{
//        u8 Direction_A,Direction_B,Direction_C,Direction_D;//发送串口数据到外部
//	      u16 Temp_GZ;
//	           if(Encoder_A>0) Direction_A=0;
//        else if(Encoder_A<0) Direction_A=2;
//	      else                 Direction_A=1;
//		         if(Encoder_B>0) Direction_B=0;
//        else if(Encoder_B<0) Direction_B=2;
//	      else                 Direction_B=1;     
//		         if(Encoder_C>0) Direction_C=0;
//        else if(Encoder_C<0) Direction_C=2;
//	      else                 Direction_C=1;
//      	Temp_GZ=Gryo_Z+32768;//角速度数据处理
				
				usart3_send(0x55); //帧头
				usart3_send(0xaa); //帧头 
				usart3_send(22);
				usart3_send(0x03);	
				usart3_send(0x01);	
				usart3_send(0x01);
				usart3_send(0x01);
				usart3_send(0x01);//timestamp

//void FloatToUint8(float source, FLOAT *target)
				usart3_send(0x00);	
				usart3_send(0x19);	//length
				usart3_send(0x00);	//payload : versionID
				sourceTest = 100.0;
				FloatToUint8(sourceTest,&targetTest);
				
				usart3_send(targetTest.B0);	//payload: Px
				usart3_send(targetTest.B1);	//payload: Px	
				usart3_send(targetTest.B2);	//payload: Px	
				usart3_send(targetTest.B3);	//payload: Px	

				sourceTest = 200.0;
				FloatToUint8(sourceTest,&targetTest);
				usart3_send(targetTest.B0);	//payload: Py
				usart3_send(targetTest.B1);	//payload: Py	
				usart3_send(targetTest.B2);	//payload: Py	
				usart3_send(targetTest.B3);	//payload: Py	

				sourceTest = 300.0;
				FloatToUint8(sourceTest,&targetTest);
				usart3_send(targetTest.B0);	//payload: Pz
				usart3_send(targetTest.B1);	//payload: Pz	
				usart3_send(targetTest.B2);	//payload: Pz
				usart3_send(targetTest.B3);	//payload: Pz		

				sourceTest = Roll;
				FloatToUint8(sourceTest,&targetTest);
				usart3_send(targetTest.B0);	//IMU: roll
				usart3_send(targetTest.B1);		
				usart3_send(targetTest.B2);		
				usart3_send(targetTest.B3);	

				sourceTest = Pitch;
				FloatToUint8(sourceTest,&targetTest);
				usart3_send(targetTest.B0);	//IMU: pitch
				usart3_send(targetTest.B1);		
				usart3_send(targetTest.B2);		
				usart3_send(targetTest.B3);	

				sourceTest = Yaw;
				FloatToUint8(sourceTest,&targetTest);
				usart3_send(targetTest.B0);	//IMU: yaw
				usart3_send(targetTest.B1);		
				usart3_send(targetTest.B2);		
				usart3_send(targetTest.B3);	
				
				usart3_send(0x33);	//crc
				usart3_send(0x0D);		
				usart3_send(0x0A);	
}
