#include "c4mlib.h"
#include "RC16M128_Lib.h"


// Global value define:
volatile uint8_t CaseCount = 0;
volatile uint8_t ServoPeriodCount = 1;
uint16_t ServoCommand[128] = {0};
uint16_t dat[6] = {0};

//除頻: 8 , ocr: 690 , 週期: 0.499855 ms , error: 0.145us
#define _case0_Timer_Init        \
	REGFPT(&ETIMSK, 0x10, 4, 0); \
	REGFPT(&TCCR3B, 0x07, 0, 2); \
	OCR3A = 690;                 \
	TCNT3 = 0;                   \
	REGFPT(&ETIMSK, 0x10, 4, 1); 

//除頻: 64 , ocr: 2 , 週期: 17.361*115 us(1.996528 us) , error: 3.472us
#define _case1_Timer_Init        \
	REGFPT(&ETIMSK, 0x10, 4, 0); \
	REGFPT(&TCCR3B, 0x07, 0, 3); \
	OCR3A = 2;                   \
	TCNT3 = 0;                   \
	REGFPT(&ETIMSK, 0x10, 4, 1); 

//除頻: 1024 , ocr: 188 , 週期: 17.5 ms , error: 0 us
#define _case2_Timer_Init        \
	REGFPT(&ETIMSK, 0x10, 4, 0); \
	REGFPT(&TCCR3B, 0x07, 0, 5); \
	OCR3A = 188;                 \
	TCNT3 = 0;                   \
	REGFPT(&ETIMSK, 0x10, 4, 1); 


void Basic_Timer_Init(void)
{
    /*計時器中斷設定*/	
	TIM_fpt(&TCCR3A,0x03,0,0);//set timer3 CTC可調方波step1
	TIM_fpt(&TCCR3B,0x18,3,1);//set timer3 CTC可調方波step2
	_case0_Timer_Init;
	TIM_fpt(&ETIMSK,0x10,4,1);//timer3 interrupt enable
}

void RC16M128_Servo_Init(void)
{
	Basic_Timer_Init();
	ServoDDRL  = 0x00;
	ServoPORTL = 0x00;
	ServoDDRH  = 0x00;
	ServoPORTH = 0x00;
}

char ASA_RC16M128_set(void)
{
	RC16M128_Servo_Init();
	sei();
	return 0;
}

char RC16M128_Servo_put(char LSByte, char Bytes, void* Data_p)
{
    static uint8_t last_value[16] = {0};
    int i = 0;
    // 參數檢查
	if( LSByte > 15 || LSByte < 0)
	return 1;
	if( (LSByte+Bytes) > 16 || (LSByte+Bytes) < 1 )
	return 2;

	// 輸出資料
	for(i=0; i<Bytes; i++)
    {
        ServoCommand[last_value[LSByte + i]] &= ~(1 << i);
        ServoCommand[((uint8_t *)Data_p)[LSByte + i]] |= (1 << i);
		last_value[LSByte + i] = ((uint8_t *)Data_p)[LSByte + i];
	}
    return 0;
}

char RC16M128_Servo_get(char LSByte, char Bytes, void* Data_p)
{
	unsigned char i, RegMode=0;	// RegMode=0 is Put Reg., RegMode=1 is Get Reg., RegMode=2 is Set Reg.;

	// 參數檢查
	if( LSByte > 15 || LSByte < 0)
	return 1;
	if( (LSByte+Bytes) > 16 || (LSByte+Bytes) < 1 )
	RegMode = 0;
	//else if( LSByte >=216 && LSByte <= 201 )
	//	RegMode = 1;	// Get register mode (RegMode = 1) is reserved for the function of servo that can be read position in the future.
	else if( LSByte == 200 || LSByte == 201)
	RegMode = 2;
	else
	return 2;

	// 輸出資料
	for(i=0; i<Bytes; i++)
	((unsigned char*)Data_p)[i] = ServoCommand[LSByte+i];

	// Put跟Set回應
	if( RegMode == 0 )
	for(i=0; i<Bytes; i++)
	((char*)Data_p)[i] = ServoCommand[LSByte+i];
	else
	{
		if( (LSByte+Bytes) == 201 )
		((char*)Data_p)[0] = ServoDDRL;
		if( (LSByte+Bytes) == 202 )
		{
			((char*)Data_p)[1] = ServoDDRH;
			if( Bytes == 2 )
			((char*)Data_p)[0] = ServoDDRL;
		}
	}
	return 0;
}

char RC16M128_Servo_set(char LSByte, char Mask, char shift, char Data)
{
	char set_Data=0;

	// 參數檢查
	if( shift > 7 || shift < 0 )
	return 2;

	// 設定資料
	if( LSByte == 200 )
	{
		set_Data = ServoDDRL;
		set_Data = (set_Data & (~Mask)) | ((Data<<shift) & Mask);
		ServoDDRL = set_Data;
	}
	else if( LSByte == 201 )
	{
		set_Data = ServoDDRH;
		set_Data = (set_Data & (~Mask)) | ((Data<<shift) & Mask);
		ServoDDRH = set_Data;
	}
	else
	return 1;

	return 0;
}

// 16通道伺服機PWM訊號產生器
ISR( TIMER3_COMPA_vect )
{
	// RC Servo PWM Command Timing Diagram
	//       _____ _________                       _____ _________                       _____ ______
	// _____|     |_________|_____________________|     |_________|_____________________|     |______
	//      |<-0->|<---1--->|<---------2--------->|
	// 
	// 0: 電位拉高   
	//    經過時間: 0.5 ms  
	// 1: 依角度拉低
	//    分割: 115份
	//    經過時間: 2 ms 
	// 2: 電位拉低
	//    經過時間: 17.5 ms
	// PWM one Wave(2) Frequency: 50Hz(20ms)


	if (CaseCount==0)
	{
		_case1_Timer_Init;
		ServoPORTL &= ~((ServoCommand[ServoPeriodCount--] & 0xff));
		ServoPORTH &= ~(((ServoCommand[ServoPeriodCount--] >> 8) & 0xff));
		CaseCount = 1;
	}
	else if (CaseCount==1)
	{
		ServoPORTL &= ~((ServoCommand[ServoPeriodCount] & 0xff));
		ServoPORTH &= ~(((ServoCommand[ServoPeriodCount] >> 8) & 0xff));

		if (ServoPeriodCount==115)
		{
			ServoPORTL = 0;
			ServoPORTH = 0;
			_case2_Timer_Init;
			CaseCount = 2;
			ServoPeriodCount = 0;
			realTimeFunc();
		}
		ServoPeriodCount++;
	}
	else if (CaseCount==2)
	{
		ServoPORTL = 0xff;
		ServoPORTH = 0xff;
		_case0_Timer_Init;
		CaseCount = 0;
	}
	
}