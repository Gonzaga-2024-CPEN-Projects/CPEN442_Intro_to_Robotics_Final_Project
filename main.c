// main program for a PID DC motor controller implementation on 
// the tm4c123gh6m eval board.

#include "TM4C123GH6PM.h"
#include "tm4c123gh6pm_def.h"
#include <stdio.h>


// define the number of ticks per thread time (2ms thread switching time)
#define TIMESLICE 32000

// LCD functs
void Init_LCD_Ports(void);
void Init_LCD(void);
void Set_Position(uint32_t POS);
void Display_Msg(char *Str);

// Operating System Functions
void OS_Init(void);
void OS_AddThreads(void f1(void), void f2(void));

void OS_Launch(uint32_t);
void OS_Wait(int32_t *S);
void OS_Signal(int32_t *S);
void OS_Sleep(uint32_t);
void OS_Suspend(void);

void delayMs(int n)
{
	for (int i = 0; i < n; i++) {
		for (int j = 0; j < 6265; j++) {
			// do nothing for 1 ms
		}
	}
}

/*
    Application specific functions
*/

// configure PWM module 1 to generate a PWM signal on
// PF2. PB2-3 control motor direction.
void Init_M1PWM6(int period, int duty)
{
	SYSCTL->RCGCPWM |= 0x02; // enable clock to PWM1
	SYSCTL->RCGCGPIO |= 0x20; // enable clock to GPIOF
	SYSCTL->RCGCGPIO |= 0x02; // enable clock to GPIOB
	delayMs(1); // PWM1 seems to take a while to start
	SYSCTL->RCC &= ~0x00100000; // use system clock for PWM
	PWM1->_3_CTL = 0; // disable PWM1_3 during configuration
	PWM1->_3_GENA = 0x00000C08; // output low for load, high for match
	PWM1->_3_LOAD = period-1; 
	PWM1->_3_CMPB = duty-1;
	PWM1->_3_CTL = 1; // enable PWM1_3
	PWM1->ENABLE |= 0x40; // enable PWM1
	GPIOF->DIR |= 0x04; // set PF3 pins as output (LED) pin
	GPIOF->DEN |= 0x04; // set PF3 pins as digital pins
	GPIOF->AFSEL |= 0x04; // enable alternate function
	GPIOF->PCTL &= ~0x00000F00; // clear PF2 alternate function
	GPIOF->PCTL |= 0x00000500; // set PF2 alternate function to PWM
	GPIOB->DEN |= 0x03; // PB2-3 as digital pins
	GPIOB->DIR |= 0x03; // set PB2-3 as output
}

void MOT34_Dir_Set(void)
{
	GPIOB->DATA &= ~0x04;
	GPIOB->DATA |= 0x08;
}

void MOT34_Speed_Set(uint16_t duty)
{
	PWM1->_3_CMPB = duty - 1;
}

// Init Timer0A to generate interrupts every 100 us. 
// The handler will sample the voltage output by the
// ADC.
void Init_Timer0A(uint32_t period_us)
{
  SYSCTL->RCGCTIMER |= 0x01; 	// activate timer0
	TIMER0->CTL &= ~0x01;				// disable timer during setup
	TIMER0->CFG = 0x00000000;		// put timer in 16 bit mode
	TIMER0->TAMR = 0x00000002;				// config timer for periodic mode
	TIMER0->TAILR = period_us * 16;				// set the count limit
	TIMER0->IMR |= 0x01;				// timer interrupts on time-out event
	NVIC_EN0_R |= 1 << 19;
	NVIC_PRI4_R |= 0x40000000;	// set priority level 2
	TIMER0->ICR = 0x01;					// clear the status bit
	TIMER0->CTL |= 0x01;
	// NOTE: here, right before this function returns, we would normally enable
	// interrupts globally, but that is done in StartOS in osasm_V2.s (so we don't)
}

int timer_count = 0;
int t1 = 0;
int t2 = 0;

void TIMER0A_Handler(void)
{
	// NOTE: we should either make this interrupt the highest priority in the system,
	//			 or disable all other interrupts while servicing it to prevent a critical code
	// 			 section from being interrupted.
	TIMER0->ICR = 0x01;		// ack the interrupt.
	timer_count++;
}

void thread1(void)
{
    while (1) {
			t1++;
		};
}

// LED thread responsible for dequeuing and outputting to led.
void thread2(void)
{
    while (1) {
			t2++;
		};

}

int main(void)
{
    Init_Timer0A(100); 	// initalize for 100us interrupts
		Init_M1PWM6(4000, 2000);
			
		Init_LCD_Ports();
    Init_LCD();

    OS_Init();                 // initialize, disable interrupts, 16 MHz
    SYSCTL_RCGCGPIO_R |= 0x28; // activate clock for Ports F and D
    while ((SYSCTL_RCGCGPIO_R & 0x28) == 0)
    {
    }                          // allow time for clock to stabilize
    GPIO_PORTD_DIR_R &= ~0x0F; // make PD3-0 input
    GPIO_PORTD_DEN_R |= 0x0F;  // enable digital I/O on PD3-0
    GPIO_PORTF_DIR_R |= 0x0E;  // make PF3-1 output
    GPIO_PORTF_DEN_R |= 0x0E;  // enable digital I/O on PF3-1

    OS_AddThreads(&thread1, &thread2);

    OS_Launch(TIMESLICE); // doesn't return, interrupts enabled in here
    return 0;             // this never executes
}
