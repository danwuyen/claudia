#ifndef SRC_APP_H_
#define SRC_APP_H_

#include "stm32l4xx_hal.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

void App_Init(void);
void App_MainLoop(void);

#endif /* SRC_APP_H_ */
