// PWM.c
// Software functions to configure the PWM.

#define SYSCTL_RCGC0_R          (*((volatile unsigned long *)0x400FE100))
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))
#define SYSCTL_RCC_R						(*((volatile unsigned long *)0x400FE060))
#define GPIO_PORTE_DIR_R        (*((volatile unsigned long *)0x40024400))
#define GPIO_PORTE_AFSEL_R      (*((volatile unsigned long *)0x40024420))
#define GPIO_PORTE_DEN_R        (*((volatile unsigned long *)0x4002451C))
#define GPIO_PORTE_AMSEL_R      (*((volatile unsigned long *)0x40024528))
#define GPIO_PORTE_PCTL_R       (*((volatile unsigned long *)0x4002452C))
#define PWM2_CTL_R 			        (*((volatile unsigned long *)0x400280C0))
#define PWM_ENABLE_R			      (*((volatile unsigned long *)0x40028008))
#define PWM2_GENA_R				      (*((volatile unsigned long *)0x400280E0))
#define PWM2_LOAD_R				      (*((volatile unsigned long *)0x400280D0))
#define PWM2_CMPA_R				      (*((volatile unsigned long *)0x400280D8))

void PWM_Init(void){
	volatile unsigned long delay;
	SYSCTL_RCGC2_R |= 0x00000010;   // 1) activate clock for Port E
	delay = SYSCTL_RCGC2_R;         //    allow time for clock to stabilize
	GPIO_PORTE_DIR_R |= 0x10;       // 2) make PE4 output
	GPIO_PORTE_AFSEL_R |= 0x10;     // 3) enable alternate function on PE4
	GPIO_PORTE_DEN_R &= ~0x10;      // 4) disable digital I/O on PE4
	GPIO_PORTE_AMSEL_R |= 0x10;     // 5) enable analog function on PE4
	GPIO_PORTE_PCTL_R = (GPIO_PORTE_PCTL_R&0xFFF0FFFF)+0x00040000;
	SYSCTL_RCGC0_R |= 0x00100000;   // 6) activate PWM0
	delay = SYSCTL_RCGC2_R;
	SYSCTL_RCC_R |= 0x00100000;			// 7) enable PWM clock divisor
	SYSCTL_RCC_R &= ~0x00020000;		// 8) divide by 4
	PWM2_CTL_R &= ~0x00000000;	  	// 9) update PWM generator 2
	PWM2_GENA_R |= 0x0000008C;
	PWM2_LOAD_R = (PWM2_LOAD_R&0xFFFF0000)+0x01F3;
	PWM2_CMPA_R = (PWM2_CMPA_R&0xFFFF0000)+0x00F9;
	PWM2_CTL_R &= ~0x00000001;	  	// 9) update PWM generator 2
	PWM_ENABLE_R |= 0x00000010;			// enable M0PWM4
}
