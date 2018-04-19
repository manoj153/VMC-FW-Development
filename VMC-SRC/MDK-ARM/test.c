/*List of functions for testing VMC IO*/
#include "C:\Users\TheVeganElectrolance\Documents\Electrolance Company\Projects\04042018-JAAVIN-VMC\VMC-SRC\MDK-ARM\appGPIO.h"
#define FUNC_MAX 8
void func(void); 
uint32_t array[10] ;


void (* fparr[8]) () = {RELAY1_ON,RELAY2_ON,RELAY3_ON,RELAY4_ON,RELAY5_ON,RELAY6_ON,RELAY7_ON,RELAY8_ON};
//typedef void* (*IntFunc)(GPIO_TypeDef , GPIO_TypeDef , GPIO_TypeDef );

//IntFunc fparr[5] = { RELAY1ON, func, func, func, func };
//array[0] = &HAL_GPIO_TogglePin;




//static void *funcAry[FUNC_MAX]();
//funcAry[0] 

//*funcAry = 

void RELAY1_ON()
{
	HAL_GPIO_WritePin(OUT1_GPIO_Port, OUT1_Pin, GPIO_PIN_SET);
}

void RELAY2_ON(void)
{
	HAL_GPIO_WritePin(OUT2_GPIO_Port, OUT2_Pin, GPIO_PIN_SET);
}

void RELAY3_ON(void)
{
	HAL_GPIO_WritePin(OUT3_GPIO_Port, OUT3_Pin, GPIO_PIN_SET);
}

void RELAY4_ON(void)
{
	HAL_GPIO_WritePin(OUT4_GPIO_Port, OUT4_Pin, GPIO_PIN_SET);
}

void RELAY5_ON(void)
{
	HAL_GPIO_WritePin(OUT5_GPIO_Port, OUT5_Pin, GPIO_PIN_SET);
}

void RELAY6_ON(void)
{
	HAL_GPIO_WritePin(OUT6_GPIO_Port, OUT6_Pin, GPIO_PIN_SET);
}

void RELAY7_ON(void)
{
	HAL_GPIO_WritePin(OUT7_GPIO_Port, OUT7_Pin, GPIO_PIN_SET);
}

void RELAY8_ON(void)
{
	HAL_GPIO_WritePin(OUT8_GPIO_Port, OUT8_Pin, GPIO_PIN_SET);
}



