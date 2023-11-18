#include "TM4C123GH6PM.h"
#include "tm4c123gh6pm_def.h"
#include <stdio.h>

// LCD functs
void Init_LCD_Ports(void);
void Init_LCD(void);
void Set_Position(uint32_t POS);
void Display_Msg(char *Str);

// Operating System Functions
void OS_Init(void);
void OS_AddThreads(void f1(void), void f2(void), void f3(void), void f4(void));
void OS_Launch(uint32_t);
void OS_Wait(int32_t *S);
void OS_Signal(int32_t *S);
void OS_Sleep(uint32_t);
void OS_Suspend(void);

/*
    Application specific functions
*/

// Init Timer0A to generate interrupts every 100 us. 
// The handler will sample the voltage output by the
// ADC.
void Init_Timer0A(uint32_t period_us)
{
    
}

void timer_thread(void)
{
    while (1) {};
}

// LED thread responsible for dequeuing and outputting to led.
void led_thread(void)
{
    while (1) {};
}

// output to LCD thread
void lcd_thread(void)
{
    while (1) {};
}

// Read switches thread
void sw_thread(void)
{
    while (1) {};
}
int main(void)
{
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

    OS_AddThreads(&timer_thread, &led_thread, &lcd_thread, &sw_thread);
    OS_Launch(TIMESLICE); // doesn't return, interrupts enabled in here
    return 0;             // this never executes
}
