#include "appGPIO.h"
#include "stm32f1xx_hal.h"
#include "stm32f100xb.h"
extern uint32_t manualSWV;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim16;
extern TIM_OC_InitTypeDef sConfigOC;
void readmanualSW()
{
	//manualSWV = 0xFFFFFFFF;
	manualSWV = 0xFFFFFFFF;
	manualSWV &= ~(HAL_GPIO_ReadPin(SW_F1_GPIO_Port, SW_F1_Pin) << 19);//SW-F1
	manualSWV &= ~(HAL_GPIO_ReadPin(SW_F2_GPIO_Port, SW_F2_Pin) << 18);
	manualSWV &= ~(HAL_GPIO_ReadPin(SW_F3_GPIO_Port, SW_F3_Pin) << 17);
	manualSWV &= ~(HAL_GPIO_ReadPin(SW_F4_GPIO_Port, SW_F4_Pin) << 16);
	manualSWV &= ~(HAL_GPIO_ReadPin(SW_F5_GPIO_Port, SW_F5_Pin) << 15);
	manualSWV &= ~(HAL_GPIO_ReadPin(SW_F6_GPIO_Port, SW_F6_Pin) << 14);
	
	manualSWV &= ~(HAL_GPIO_ReadPin(SW_R1_GPIO_Port, SW_R1_Pin) << 13); //SW-R1
	manualSWV &= ~(HAL_GPIO_ReadPin(SW_R2_GPIO_Port, SW_R2_Pin) << 12);
	manualSWV &= ~(HAL_GPIO_ReadPin(SW_R3_GPIO_Port, SW_R3_Pin) << 11);
	manualSWV &= ~(HAL_GPIO_ReadPin(SW_R4_GPIO_Port, SW_R4_Pin) << 10);
	manualSWV &= ~(HAL_GPIO_ReadPin(SW_R5_GPIO_Port, SW_R5_Pin) << 9);
	manualSWV &= ~(HAL_GPIO_ReadPin(SW_R6_GPIO_Port, SW_R6_Pin) << 8);
	
	manualSWV &= ~(HAL_GPIO_ReadPin(SW_1_GPIO_Port,SW_1_Pin) << 7);//SW-1
	manualSWV &= ~(HAL_GPIO_ReadPin(SW_2_GPIO_Port,SW_2_Pin) << 6);
	manualSWV &= ~(HAL_GPIO_ReadPin(SW_3_GPIO_Port,SW_3_Pin) << 5);
	manualSWV &= ~(HAL_GPIO_ReadPin(SW_4_GPIO_Port,SW_4_Pin) << 4);
	manualSWV &= ~(HAL_GPIO_ReadPin(SW_5_GPIO_Port,SW_5_Pin) << 3);
	manualSWV &= ~(HAL_GPIO_ReadPin(SW_6_GPIO_Port,SW_6_Pin) << 2);
	manualSWV &= ~(HAL_GPIO_ReadPin(SW_7_GPIO_Port,SW_7_Pin) << 1);
	manualSWV &= ~(HAL_GPIO_ReadPin(SW_8_GPIO_Port,SW_8_Pin) << 0);
	
	
}

void trigmanualSW()
{
		if((manualSWV >> 0) & 1) //SW8 Rly //Signal,0, 1 READ
		{
			//on the relay
			RELAY8_ON();
		}
		else
		{
			//Off the relay 
			RELAY8_OFF();
		}
		
		if((manualSWV >> 1) & 1) //SW8 Rly
		{
			//on the relay
			RELAY7_ON();
		}
		else
		{
			//Off the relay 
			RELAY7_OFF();
		}
		
		if((manualSWV >> 2) & 1) //SW8 Rly
		{
			//on the relay
			RELAY6_ON();
		}
		else
		{
			//Off the relay 
			RELAY6_OFF();
		}
		
		if((manualSWV >> 3) & 1) //SW8 Rly
		{
			//on the relay
			RELAY5_ON();
		}
		else
		{
			//Off the relay 
			RELAY5_OFF();
		}
		
		if((manualSWV >> 4) & 1) //SW8 Rly
		{
			//on the relay
			RELAY4_ON();
		}
		else
		{
			//Off the relay 
			RELAY4_OFF();
		}
		
		if((manualSWV >> 5) & 1) //SW8 Rly
		{
			//on the relay
			RELAY3_ON();
		}
		else
		{
			//Off the relay 
			RELAY3_OFF();
		}
		
		if((manualSWV >> 6) & 1) //SW8 Rly
		{
			//on the relay
			RELAY2_ON();
		}
		else
		{
			//Off the relay 
			RELAY2_OFF();
		}
		
		if((manualSWV >> 7) & 1) //SW8 Rly
		{
			//on the relay
			RELAY1_ON();
		}
		else
		{
			//Off the relay 
			RELAY1_OFF();
		}
		
		if((manualSWV >> 19) & 1) //SW F1 MTR 
		{
			//Set 90% DutyCycle Should be 10% in code for clockwise cos there is inversion smitt on circuit.
			//on the PWM
			//On the SD Line High
			TIM1->CCR1 = 5; //Chip sees 90% 
			HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
			HAL_GPIO_WritePin(SD1_GPIO_Port, SD1_Pin, GPIO_PIN_RESET); // chip see HIGH 
			
			
		}
		else
		{
			
			HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
			HAL_GPIO_WritePin(SD1_GPIO_Port, SD1_Pin, GPIO_PIN_SET); // CHIP See low 
			//off the PWM
			//Off the SD Line by LOW
		}
		
		if((manualSWV >> 18) & 1) //SW F2 MTR 
		{
			//Set 90% DutyCycle Should be 10% in code for clockwise cos there is inversion smitt on circuit.
			//on the PWM
			//On the SD Line High
			TIM2->CCR1 = 5; //Chip sees 90% 
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
			HAL_GPIO_WritePin(SD2_GPIO_Port, SD2_Pin, GPIO_PIN_RESET); // chip see HIGH 
			
			
		}
		else
		{
			
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
			HAL_GPIO_WritePin(SD2_GPIO_Port, SD2_Pin, GPIO_PIN_SET); // CHIP See low 
			//off the PWM
			//Off the SD Line by LOW
		}

		if((manualSWV >> 17) & 1) //SW F3 MTR 
		{
			//Set 90% DutyCycle Should be 10% in code for clockwise cos there is inversion smitt on circuit.
			//on the PWM
			//On the SD Line High
			TIM3->CCR1 = 5; //Chip sees 90% 
			HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
			HAL_GPIO_WritePin(SD3_GPIO_Port, SD3_Pin, GPIO_PIN_RESET); // chip see HIGH 
			
			
		}
		else
		{
			
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
			HAL_GPIO_WritePin(SD3_GPIO_Port, SD3_Pin, GPIO_PIN_SET); // CHIP See low 
			//off the PWM
			//Off the SD Line by LOW
		}		
		
		if((manualSWV >> 16) & 1) //SW F4 MTR 
		{
			//Set 90% DutyCycle Should be 10% in code for clockwise cos there is inversion smitt on circuit.
			//on the PWM
			//On the SD Line High
			TIM4->CCR1 = 5; //Chip sees 90% 
			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
			HAL_GPIO_WritePin(SD4_GPIO_Port, SD4_Pin, GPIO_PIN_RESET); // chip see HIGH 
			
			
		}
		else
		{
			
			HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
			HAL_GPIO_WritePin(SD4_GPIO_Port, SD4_Pin, GPIO_PIN_SET); // CHIP See low 
			//off the PWM
			//Off the SD Line by LOW
		}		

		if((manualSWV >> 15) & 1) //SW F5 MTR 
		{
			//Set 90% DutyCycle Should be 10% in code for clockwise cos there is inversion smitt on circuit.
			//on the PWM
			//On the SD Line High
			TIM15->CCR1 = 5; //Chip sees 90% 
			HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
			HAL_GPIO_WritePin(SD5_GPIO_Port, SD5_Pin, GPIO_PIN_RESET); // chip see HIGH 
			
			
		}
		else
		{
			
			HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_1);
			HAL_GPIO_WritePin(SD5_GPIO_Port, SD5_Pin, GPIO_PIN_SET); // CHIP See low 
			//off the PWM
			//Off the SD Line by LOW
		}		

		if((manualSWV >> 14) & 1) //SW F6 MTR 
		{
			//Set 90% DutyCycle Should be 10% in code for clockwise cos there is inversion smitt on circuit.
			//on the PWM
			//On the SD Line High
			TIM16->CCR1 = 5; //Chip sees 90% 
			HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
			HAL_GPIO_WritePin(SD6_GPIO_Port, SD6_Pin, GPIO_PIN_RESET); // chip see HIGH 
			
			
		}
		else
		{
			
			HAL_TIM_PWM_Stop(&htim16, TIM_CHANNEL_1);
			HAL_GPIO_WritePin(SD6_GPIO_Port, SD6_Pin, GPIO_PIN_SET); // CHIP See low 
			//off the PWM
			//Off the SD Line by LOW
		}						
		
}