#include "main.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal.h"
#include "main.h"
//#define HAL_GP  
void RELAY1_ON(void);
void RELAY2_ON(void);
void RELAY3_ON(void);
void RELAY4_ON(void);
void RELAY5_ON(void);
void RELAY6_ON(void);
void RELAY7_ON(void);
void RELAY8_ON(void);

void RELAY1_OFF(void);
void RELAY2_OFF(void);
void RELAY3_OFF(void);
void RELAY4_OFF(void);
void RELAY5_OFF(void);
void RELAY6_OFF(void);
void RELAY7_OFF(void);
void RELAY8_OFF(void);

void readmanualSW(void);
void trigmanualSW(void);


//#define RELAY1ON const HAL_GPIO_WritePin(OUT1_GPIO_Port, OUT1_Pin, GPIO_PIN_SET)

//#define RELAY1_ON() HAL_GPIO_WritePin(OUT1_GPIO_Port, OUT1_Pin, GPIO_PIN_SET);
//#define RELAY1_OFF() HAL_GPIO_WritePin(OUT1_GPIO_Port, OUT1_Pin, GPIO_PIN_RESET);

//#define RELAY2_ON() HAL_GPIO_WritePin(OUT2_GPIO_Port, OUT2_Pin, GPIO_PIN_SET);
//#define RELAY2_OFF() HAL_GPIO_WritePin(OUT2_GPIO_Port, OUT2_Pin, GPIO_PIN_RESET);

//#define RELAY3_ON() HAL_GPIO_WritePin(OUT3_GPIO_Port, OUT3_Pin, GPIO_PIN_SET);
//#define RELAY3_OFF() HAL_GPIO_WritePin(OUT3_GPIO_Port, OUT3_Pin, GPIO_PIN_RESET);

//#define RELAY4_ON() HAL_GPIO_WritePin(OUT4_GPIO_Port, OUT4_Pin, GPIO_PIN_SET);
//#define RELAY4_OFF() HAL_GPIO_WritePin(OUT4_GPIO_Port, OUT4_Pin, GPIO_PIN_RESET);

//#define RELAY5_ON() HAL_GPIO_WritePin(OUT5_GPIO_Port, OUT5_Pin, GPIO_PIN_SET);
//#define RELAY5_OFF() HAL_GPIO_WritePin(OUT5_GPIO_Port, OUT5_Pin, GPIO_PIN_RESET);

//#define RELAY6_ON() HAL_GPIO_WritePin(OUT6_GPIO_Port, OUT6_Pin, GPIO_PIN_SET);
//#define RELAY6_OFF() HAL_GPIO_WritePin(OUT6_GPIO_Port, OUT6_Pin, GPIO_PIN_RESET);

//#define RELAY7_ON() HAL_GPIO_WritePin(OUT7_GPIO_Port, OUT7_Pin, GPIO_PIN_SET);
//#define RELAY7_OFF() HAL_GPIO_WritePin(OUT7_GPIO_Port, OUT7_Pin, GPIO_PIN_RESET);

//#define RELAY8_ON() HAL_GPIO_WritePin(OUT8_GPIO_Port, OUT8_Pin, GPIO_PIN_SET);
//#define RELAY8_OFF() HAL_GPIO_WritePin(OUT8_GPIO_Port, OUT8_Pin, GPIO_PIN_RESET);





