
#include "servo_pca9685.h"

#define PCA9685_ADDRESS       0x80 
#define PCA9685_MODE1         0x0
#define PCA9685_PRE_SCALE     0xFE
#define PCA9685_LED0_ON_L     0x6
#define PCA9685_MODE1_SLEEP_BIT      4
#define PCA9685_MODE1_AI_BIT         5
#define PCA9685_MODE1_RESTART_BIT    7

ServoConfig_t servo[] = {
    {0, 102, 570}, 
    {1, 110, 580}, 
    {2, 95,  560}
};
float angle[3] = {0, 0, 0}; 
static void PCA9685_SetBit(uint8_t Register, uint8_t Bit, uint8_t Value)
{
    uint8_t readValue;
    HAL_I2C_Mem_Read(&hi2c1, PCA9685_ADDRESS, Register, 1, &readValue, 1, 10);
    
    if (Value == 0) readValue &= ~(1 << Bit);
    else readValue |= (1 << Bit);
    
    HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS, Register, 1, &readValue, 1, 10);
}

static void PCA9685_SetPWMFrequency(uint16_t frequency)
{
    uint8_t prescale;
    if(frequency >= 1526) prescale = 0x03;
    else if(frequency <= 24) prescale = 0xFF;
    else prescale = 25000000 / (4096 * frequency);

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

void PCA9685_SetServoAngle(ServoConfig_t *servo, float Angle)
{
    if (Angle < 0) Angle = 0;
    if (Angle > 180) Angle = 180;
    uint16_t value = (uint16_t)((Angle / 180.0f * ((*servo).value_180 - (*servo).value_0)) + (*servo).value_0);

    PCA9685_SetPWM((*servo).channel, 0, value);
}

void ServoPCA9685_Control(void)
{
    PCA9685_Init(50); 

    for(;;) 
    {
        PCA9685_SetServoAngle(&servo[0], angle[0]);
        PCA9685_SetServoAngle(&servo[1], angle[1]);
        PCA9685_SetServoAngle(&servo[2], angle[2]);
        
        osDelay(20); 
    }
}