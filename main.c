// main program for a PID DC motor controller implementation on 
// the tm4c123gh6m eval board.

#include "TM4C123GH6PM.h"
#include "tm4c123gh6pm_def.h"
#include <stdio.h>


#define TIMESLICE 32000 // thread switch time in system time units
                        // clock frequency is 16 MHz, switching time is 2ms
#define THREAD_SW_PER_SEC 500
#define COLOR_QUEUE_SIZE 10



// Operating System Functions
void OS_Init(void);
void OS_AddThreads(void f1(void));
void OS_Launch(uint32_t);
void OS_Wait(int32_t *S);
void OS_Signal(int32_t *S);
void OS_Sleep(uint32_t);
void OS_Suspend(void);

void TIMER0A_Handler(void) {
	
}

void timer_thread(void)
{
	while (1) {};
}

int main(void)
{

    OS_Init();                 // initialize, disable interrupts, 16 MHz
    SYSCTL_RCGCGPIO_R |= 0x28; // activate clock for Ports F and D
    while ((SYSCTL_RCGCGPIO_R & 0x28) == 0)
    {
    }                          // allow time for clock to stabilize
    GPIO_PORTD_DIR_R &= ~0x0F; // make PD3-0 input
    GPIO_PORTD_DEN_R |= 0x0F;  // enable digital I/O on PD3-0
    GPIO_PORTF_DIR_R |= 0x0E;  // make PF3-1 output
    GPIO_PORTF_DEN_R |= 0x0E;  // enable digital I/O on PF3-1

    OS_AddThreads(&timer_thread);// , &led_thread, &lcd_thread, &sw_thread);
    OS_Launch(TIMESLICE); // doesn't return, interrupts enabled in here
    return 0;             // this never executes
}
