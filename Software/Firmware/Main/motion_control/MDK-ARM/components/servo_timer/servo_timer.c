#include "servo_timer.h"
static void Servo_SetAngle(uint8_t angle)
{
    if(angle > 180) angle = 180;
    uint32_t ccr = 500 + ((angle * 2000) / 180);
    
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ccr);
}

void ServoTimer_Control(void)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    for(;;)
    {
        
        Servo_SetAngle(0);  
        osDelay(1000);      
        Servo_SetAngle(45);  
        osDelay(1000);
			  Servo_SetAngle(90);  
        osDelay(1000);
				Servo_SetAngle(135);  
        osDelay(1000);
				Servo_SetAngle(180);  
        osDelay(1000);
    }
}