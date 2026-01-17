#ifndef INC_SERVO_PCA9685_H_
#define INC_SERVO_PCA9685_H_

#include "main.h"
#include "cmsis_os.h" 
extern float angle[3];
typedef struct {
    uint8_t channel;       
    uint16_t value_0;     
    uint16_t value_180;     
} ServoConfig_t;
extern ServoConfig_t servo[];
void ServoPCA9685_Control(void);
void PCA9685_SetServoAngle(ServoConfig_t *servo, float Angle);
#endif 