// Control.c
// Controle de temperatura com controle PID
// João Vicente Balvedi Gaiewski
// 12/08/2014

#include "PLL.h"
#include "UART.h"
#include "Nokia5110.h"
#include "ADC.h"
#include "PWM.h"

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

#define SAMPLE_T_MS 100
	
// funções definidas no startup_TMC123.s
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
void WaitForInterrupt(void);  // low power mode

volatile unsigned long ticker;

volatile float Kp, Ki, Kd;		// coeficients
volatile float ek = 0;				// error
volatile float ek1 = 0;				// last error
volatile float Ik1 = 0;	 			// last integral sum
volatile float uk = 0;				// control effort

volatile unsigned long setpoint = 900;
volatile float temperature;

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

void GPIOF_Handler(void){ // called on touch of either SW1 or SW2
  if(GPIO_PORTF_RIS_R&0x01){  // SW2 touch
    GPIO_PORTF_ICR_R = 0x01;  // acknowledge flag0
    setpoint += 100;    // heat up
		if (setpoint > 1100) setpoint = 1100;
  }
  if(GPIO_PORTF_RIS_R&0x10){  // SW1 touch
    GPIO_PORTF_ICR_R = 0x10;  // acknowledge flag4
    setpoint -= 100;  //cool down
		if (setpoint < 100) setpoint = 100;
  }
}

void SysTick_Init(void){
	NVIC_ST_CTRL_R = 0;           // disable SysTick during setup
  NVIC_ST_RELOAD_R = 50000-1;   // reload value for 500us
  NVIC_ST_CURRENT_R = 0;        // any write to current clears it
  NVIC_SYS_PRI3_R = (NVIC_SYS_PRI3_R&0x00FFFFFF)|0x40000000; // priority 2
  NVIC_ST_CTRL_R = 0x00000007;  // enable with core clock and interrupts
}

void SysTick_Handler(void){
	ticker += 1;
}
	
long ControlPID (long temp, long setpoint){
	
	volatile float T = SAMPLE_T_MS*1e-3;
	volatile float P = 0, I = 0, D = 0;
	
	// Coeficients Values
	Kp = 1;
	Ki = 0;
	Kd = 0;
	
	ek = setpoint - temp;		// erro
	
	// Proportional term
	P = Kp*ek;
	
	// Integral term
	if (Ik1 >= 99){		//anti wind-up
		I = 99;
	}
	else if (Ik1 <= 0){
		I = 0;
	}
	else{
		I = (Ki*T*(ek + ek1)/2) + Ik1;
	}
	
	// Derivative term
	D = (Kd*(ek - ek1))/T;
	
	uk = P+I+D;		// Control Effort
	
	if (uk <= 1 ){
		uk = 1;
	}
	if (uk >= 99){
		uk = 99;
	}
	
	ek1 = ek;			// Last Error
	Ik1 = I;			// Last Integral Term
	
	return uk;
}

//  function delays 3*ulCount cycles
void Delay_ms(unsigned ulCount){
	unsigned long count = 16666;
	count *= ulCount;
	do{
		count--;
	}while(count);
}

float getTemp(){		// 150 Ohms -> 0.6 V ~ 3V (745 ~ 3723)
	ADC0_Get();
	if (ADCvalue < 740){
		temperature = -10;
		}
	else {
		temperature = ADCvalue * 0.403 - 300.052;
		}
	return temperature;
}

void SystemInit(){
}

int main(void){
	
  DisableInterrupts();	// disable interrupts while initializing
  PLL_Init();						// bus clock at 50 MHz
	UART_Init();					// initialize UART
  Nokia5110_Init();			// initialize Nokia 5110
	PWM_Init();						// output from PE4
	SysTick_Init();				// initialize periodic interrupt (SysTick)
  Switch_Init();				// arm PF4, PF0 for falling edge interrupts
	ADC0_Init();					// ADC initialization PE2/AIN1
  EnableInterrupts();		// enable after all initialization are done
  
	PWM_UpdateDuty();
	
	Nokia5110_DrawFullImage(UTFPR);
	Delay_ms(1000);
	Nokia5110_Clear();
	Nokia5110_OutString("   uForno   ");
	Nokia5110_OutString("------------");
	Nokia5110_OutString("SetPt:      ");
	Nokia5110_OutString("Temp:       ");
	Nokia5110_OutString("Duty:       ");
	
	while(1){
		if(ticker >= 500){	//500ms
			
			temperature = getTemp();
			ControlPID(temperature,setpoint);
			Duty = uk;
			PWM_UpdateDuty();
			
			UART_OutUDec(setpoint);
			UART_OutChar('\t');
			if (temperature < -1){
				UART_OutString("Erro");
			}
			else {
				UART_OutUDec(temperature);
			}
			UART_OutChar('\t');
			UART_OutUDec(Duty);
			OutCRLF();
			
			Nokia5110_SetCursor(6, 2);
			Nokia5110_OutUDec(setpoint);
			Nokia5110_OutChar(127);
			Nokia5110_OutChar('C');
			Nokia5110_SetCursor(6, 3);
			if (temperature < -1){
				Nokia5110_OutString("Erro");
			}
			else {
				Nokia5110_OutUDec(temperature);
			}
			Nokia5110_OutChar(127);
			Nokia5110_OutChar('C');
			Nokia5110_SetCursor(6, 4);
			Nokia5110_OutUDec(Duty);
			Nokia5110_OutChar(' ');
			Nokia5110_OutChar('%');
			
			ticker = 0;
		}
    WaitForInterrupt(); // low power mode
  }
}
