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

extern void Init_LCD_Ports(void);
extern void Display_Msg(char* Str);
extern void Set_Position(uint32_t POS);
extern void Init_LCD(void);
extern void Init_Keypad();

extern void Hex2ASCII(char* output, int hex);
extern uint8_t Read_Key();
uint8_t Key_ASCII;

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
	
    Init_LCD_Ports(); //init LCD	
	Init_LCD();
	
	Init_Keypad();
	
	//uint8_t key = Read_Key();
	
	
	
	/*Routine to print all necessary values to LCD*/
	Set_Position(0x00);
	Display_Msg("Input RPM:");
	
	int input_RPM =0x0A; //replace this value with average speed
	char RPM_str[4];
	char* RPM_ptr = RPM_str;
	Hex2ASCII(RPM_ptr, input_RPM);
	Display_Msg(RPM_ptr);
	
	
	Set_Position(0x40);
	Display_Msg("T:");
	
	int average_speed =0x270F; //replace this value with average speed
	char avg_str[4];
	char* avg_ptr = avg_str;
	Hex2ASCII(avg_ptr, average_speed);
	Display_Msg(avg_ptr);
	
	Set_Position(0x48);
	Display_Msg("C:");
	
	int current_speed =0x0A0A; //replace this value with current speed
	char cur_str[4];
	char* cur_ptr = cur_str;
	Hex2ASCII(cur_ptr, current_speed);
	Display_Msg(cur_ptr);
	




    OS_AddThreads(&timer_thread);// , &led_thread, &lcd_thread, &sw_thread);
    OS_Launch(TIMESLICE); // doesn't return, interrupts enabled in here
    return 0;             // this never executes
}
