// PWM.h
// Software functions to configure the PWM.

#define PERIOD 500		//(PLL/PWMDIV)/(FREQ_PWM)

extern volatile unsigned long Duty;

void PWM_Init(void);
