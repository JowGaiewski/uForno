// PWM.h
// Software functions to configure the PWM.

#define PERIOD 1249		//(PLL/PWMDIV)/(FREQ_PWM)-1

extern volatile unsigned short Duty;

void PWM_Init(void);

void PWM_UpdateDuty(void);
