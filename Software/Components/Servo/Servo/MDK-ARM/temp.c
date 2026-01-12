/* tasks_manager.c */
#include "tasks_manager.h"
#include "main.h"  

extern I2C_HandleTypeDef hi2c1; 

#define PCA9685_ADDRESS       0x80 
#define PCA9685_MODE1         0x0
#define PCA9685_PRE_SCALE     0xFE
#define PCA9685_LED0_ON_L     0x6
#define PCA9685_MODE1_SLEEP_BIT      4
#define PCA9685_MODE1_AI_BIT         5
#define PCA9685_MODE1_RESTART_BIT    7

static void PCA9685_SetBit(uint8_t Register, uint8_t Bit, uint8_t Value)
{
    uint8_t readValue;
    HAL_I2C_Mem_Read(&hi2c1, PCA9685_ADDRESS, Register, 1, &readValue, 1, 10);
    
    if (Value == 0) readValue &= ~(1 << Bit);
    else readValue |= (1 << Bit);
    HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS, Register, 1, &readValue, 1, 10);
    HAL_Delay(1); 
}

static void PCA9685_SetPWMFrequency(uint16_t frequency)
{
    uint8_t prescale;
    if(frequency >= 1526) prescale = 0x03;
    else if(frequency <= 24) prescale = 0xFF;
    else prescale = 25000000 / (4096 * frequency); // 25MHz internal clock

    PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_SLEEP_BIT, 1);
    HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS, PCA9685_PRE_SCALE, 1, &prescale, 1, 10);
    PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_SLEEP_BIT, 0);
    PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_RESTART_BIT, 1);
}

static void PCA9685_Init(uint16_t frequency)
{
    PCA9685_SetPWMFrequency(frequency); 
    PCA9685_SetBit(PCA9685_MODE1, PCA9685_MODE1_AI_BIT, 1); 
}

static void PCA9685_SetPWM(uint8_t Channel, uint16_t OnTime, uint16_t OffTime)
{
    uint8_t registerAddress;
    uint8_t pwm[4];
    
    registerAddress = PCA9685_LED0_ON_L + (4 * Channel);
    
    pwm[0] = OnTime & 0xFF;
    pwm[1] = OnTime >> 8;
    pwm[2] = OffTime & 0xFF;
    pwm[3] = OffTime >> 8;
    
    HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS, registerAddress, 1, pwm, 4, 10);
}

static void PCA9685_SetServoAngle(uint8_t Channel, float Angle)
{
    if (Angle < 0) Angle = 0;
    if (Angle > 180) Angle = 180;

    float Value;
    Value = (Angle * (511.9 - 102.4) / 180.0) + 102.4;
    
    PCA9685_SetPWM(Channel, 0, (uint16_t)Value);
}
void Servo_Control(void)
{
    PCA9685_Init(50); 
    
    
    for(;;)
    {
        PCA9685_SetServoAngle(0, 0);   
        PCA9685_SetServoAngle(1, 0);   
        PCA9685_SetServoAngle(2, 0);   
        
        osDelay(1000); 

        PCA9685_SetServoAngle(0, 90);  
        PCA9685_SetServoAngle(1, 90);
        PCA9685_SetServoAngle(2, 90);
        
        osDelay(1000);

        PCA9685_SetServoAngle(0, 180); 
        PCA9685_SetServoAngle(1, 180);
        PCA9685_SetServoAngle(2, 180);
        
        osDelay(1000);
    }
}