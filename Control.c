// Control.c
// Controle de temperatura com controle PID
// João Vicente Balvedi Gaiewski
// 12/08/2014

#include "PLL.h"
#include "UART.h"
#include "Nokia5110.h"
#include "ADC.h"

#define GPIO_PORTA_DATA_R       (*((volatile unsigned long *)0x400043FC))
#define GPIO_PORTA_DIR_R        (*((volatile unsigned long *)0x40004400))
#define GPIO_PORTA_AFSEL_R      (*((volatile unsigned long *)0x40004420))
#define GPIO_PORTA_DR8R_R       (*((volatile unsigned long *)0x40004508))
#define GPIO_PORTA_DEN_R        (*((volatile unsigned long *)0x4000451C))
#define GPIO_PORTA_AMSEL_R      (*((volatile unsigned long *)0x40004528))
#define GPIO_PORTA_PCTL_R       (*((volatile unsigned long *)0x4000452C))
#define GPIO_PORTF_DATA_R       (*((volatile unsigned long *)0x400253FC))
#define GPIO_PORTF_DIR_R        (*((volatile unsigned long *)0x40025400))
#define GPIO_PORTF_IS_R         (*((volatile unsigned long *)0x40025404))
#define GPIO_PORTF_IBE_R        (*((volatile unsigned long *)0x40025408))
#define GPIO_PORTF_IEV_R        (*((volatile unsigned long *)0x4002540C))
#define GPIO_PORTF_IM_R         (*((volatile unsigned long *)0x40025410))
#define GPIO_PORTF_RIS_R        (*((volatile unsigned long *)0x40025414))
#define GPIO_PORTF_ICR_R        (*((volatile unsigned long *)0x4002541C))
#define GPIO_PORTF_AFSEL_R      (*((volatile unsigned long *)0x40025420))
#define GPIO_PORTF_PUR_R        (*((volatile unsigned long *)0x40025510))
#define GPIO_PORTF_DEN_R        (*((volatile unsigned long *)0x4002551C))
#define GPIO_PORTF_LOCK_R       (*((volatile unsigned long *)0x40025520))
#define GPIO_PORTF_CR_R         (*((volatile unsigned long *)0x40025524))
#define GPIO_PORTF_AMSEL_R      (*((volatile unsigned long *)0x40025528))
#define GPIO_PORTF_PCTL_R       (*((volatile unsigned long *)0x4002552C))
#define GPIO_LOCK_KEY           0x4C4F434B  // Unlocks the GPIO_CR register
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))
#define NVIC_ST_CTRL_R          (*((volatile unsigned long *)0xE000E010))
#define NVIC_ST_RELOAD_R        (*((volatile unsigned long *)0xE000E014))
#define NVIC_ST_CURRENT_R       (*((volatile unsigned long *)0xE000E018))
#define NVIC_EN0_R              (*((volatile unsigned long *)0xE000E100))
#define NVIC_PRI7_R             (*((volatile unsigned long *)0xE000E41C))
#define NVIC_SYS_PRI3_R         (*((volatile unsigned long *)0xE000ED20))

#define SAMPLE_FRQ 50000			// PLL/SAMPLE_FREQ = 1KHz
#define SAMPLE_T_MS 1
	
// funções definidas no startup_TMC123.s
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
void WaitForInterrupt(void);  // low power mode

extern volatile unsigned long ADCvalue;

volatile unsigned long H,L;
volatile unsigned long ticker;
volatile float ek = 0;				// error
volatile float ek1 = 0;				// last error
float Ik1 = 0;	 				     	// last integral sum
volatile float Uk1 = 0;	      // last integral sum
volatile float P = 0, I = 0, D = 0;
volatile float uk = 0;
volatile unsigned long setpoint = 5000;
volatile long cycle;

void PWM_Init(void){
  SYSCTL_RCGC2_R |= 0x00000001; // activate clock for port A
  H = L = SAMPLE_FRQ/2;             // 50%
  GPIO_PORTA_AMSEL_R &= ~0xA0;      // disable analog functionality on PA5 PA7
  GPIO_PORTA_PCTL_R &= ~0xF0F00000; // configure PA5 PA7 as GPIO
  GPIO_PORTA_DIR_R |= 0xA0;     // make PA5 PA7 out
  GPIO_PORTA_DR8R_R |= 0xA0;    // enable 8 mA drive on PA5 PA7
  GPIO_PORTA_AFSEL_R &= ~0xA0;  // disable alt funct on PA5 PA7
  GPIO_PORTA_DEN_R |= 0xA0;     // enable digital I/O on PA5 PA7
  GPIO_PORTA_DATA_R &= ~0x20;   // make PA5 low
	GPIO_PORTA_DATA_R |= 0x80;    // make PA7 high
  NVIC_ST_CTRL_R = 0;           // disable SysTick during setup
  NVIC_ST_RELOAD_R = L-1;       // reload value for 500us
  NVIC_ST_CURRENT_R = 0;        // any write to current clears it
  NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&0x00FFFFFF)|0x40000000; // priority 2
  NVIC_ST_CTRL_R = 0x00000007;  // enable with core clock and interrupts
}

void Switch_Init(void){  
	unsigned long volatile delay;
  SYSCTL_RCGC2_R |= 0x00000020; // activate clock for port F
  delay = SYSCTL_RCGC2_R;
  GPIO_PORTF_LOCK_R = 0x4C4F434B; // unlock GPIO Port F
  GPIO_PORTF_CR_R = 0x11;         // allow changes to PF4,0
  GPIO_PORTF_DIR_R &= ~0x11;    // make PF4,0 in (built-in button)
  GPIO_PORTF_AFSEL_R &= ~0x11;  //     disable alt funct on PF4,0
  GPIO_PORTF_DEN_R |= 0x11;     //     enable digital I/O on PF4,0
  GPIO_PORTF_PCTL_R &= ~0x000F000F; //  configure PF4,0 as GPIO
  GPIO_PORTF_AMSEL_R &= ~0x11;  //     disable analog functionality on PF4,0
  GPIO_PORTF_PUR_R |= 0x11;     //     enable weak pull-up on PF4,0
  GPIO_PORTF_IS_R &= ~0x11;     // PF4,PF0 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x11;    //     PF4,PF0 is not both edges
  GPIO_PORTF_IEV_R &= ~0x11;    //     PF4,PF0 falling edge event
  GPIO_PORTF_ICR_R = 0x11;      //  clear flags 4,0
  GPIO_PORTF_IM_R |= 0x11;      //  arm interrupt on PF4,PF0
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00400000; // (g) priority 2
  NVIC_EN0_R = 0x40000000;      //  enable interrupt 30 in NVIC
}

void SysTick_Handler(void){
  if(GPIO_PORTA_DATA_R&0x20){   // toggle PA5
    GPIO_PORTA_DATA_R &= ~0x20; // make PA5 low
    NVIC_ST_RELOAD_R = L-1;     // reload value for low phase
  } else{
    GPIO_PORTA_DATA_R |= 0x20;  // make PA5 high
    NVIC_ST_RELOAD_R = H-1;     // reload value for high phase
  }
	ticker += 1;
}

void GPIOF_Handler(void){ // called on touch of either SW1 or SW2
  if(GPIO_PORTF_RIS_R&0x01){  // SW2 touch
    GPIO_PORTF_ICR_R = 0x01;  // acknowledge flag0
    setpoint += 100;    // heat up
  }
  if(GPIO_PORTF_RIS_R&0x10){  // SW1 touch
    GPIO_PORTF_ICR_R = 0x10;  // acknowledge flag4
    setpoint -= 100;  //cool down
  }
}

void Duty_Cycle (float cycle){
	cycle = 1000 - cycle;  
	if (cycle <= 1){
		cycle = 1;
	}
	if (cycle >= 999) {
		cycle = 999;
	}
	L = cycle*SAMPLE_FRQ/1000;
	H = SAMPLE_FRQ - L;
}
	
long ControlPID (long temp, long setpoint){
	volatile float Kp, Ki, Kd;
	volatile float T = SAMPLE_T_MS*1e-3;
	Kp = 3;
	Ki = 0.1;
	Kd = 1;
	
	ek = setpoint - temp;
	
	P = Kp*ek;
	
	if (Uk1 >= 999 || Uk1 <= 0){
		I = Ik1;
	}else{
		I = ((Ki*T*(ek + ek1)/2) + Ik1);
	}
	
	D = (Kd*(ek - ek1)*100)/SAMPLE_T_MS;
	
	ek1 = ek;
	uk = P+I+D;
	
	Ik1 = I;
	Uk1 = uk;
	
	if (uk <= 1 ){
		uk = 1;
	}
	if (uk >= 999){
		uk = 999;
	}
	cycle = uk;
	
	return cycle;
}

void SystemInit(){
}

int main(void){
	
	volatile float sensor_volt;
	unsigned long integer;
	
  DisableInterrupts();  	// disable interrupts while initializing
  PLL_Init();			// bus clock at 80 MHz
	UART_Init();		// initialize UART
  Nokia5110_Init();	// initialize Nokia 5110
	PWM_Init();			// output from PA5, SysTick interrupts
  Switch_Init();	// arm PF4, PF0 for falling edge interrupts
	ADC0_Init();		// ADC initialization PE2/AIN1
  EnableInterrupts();   	// enable after all initialization are done
  
	Nokia5110_DrawFullImage(UTFPR);
	
	while(1){
		if(ticker >= 100){	//100ms
			ADC0_Get();
			sensor_volt = ADCvalue*33000;
			sensor_volt /= 4095;
			cycle = ControlPID(sensor_volt,setpoint);
			Duty_Cycle(cycle);
			
			integer = (int)sensor_volt;
			UART_OutUDec(sensor_volt/100);
			UART_OutChar('.');
			sensor_volt = (sensor_volt-integer)*100;
			integer = (int)sensor_volt;
			if (integer < 10){
				UART_OutChar('0');
			}
			UART_OutUDec(sensor_volt);
			UART_OutChar('\t');
			UART_OutUDec(cycle/10);
			UART_OutChar('.');
			UART_OutUDec(cycle - ((cycle/10)*10));
			UART_OutChar('\t');
			UART_OutUDec(setpoint/100);
			//UART_OutChar('\t');
			//UART_OutUDec(I);
			//UART_OutChar('\t');
			//UART_OutUDec();
			OutCRLF();
			ticker = 0;
		}
    WaitForInterrupt(); // low power mode
  }
}
