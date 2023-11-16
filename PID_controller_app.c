// *****************************************************************************
// user.c
// Runs on LM4F120/TM4C123
// An example user program that initializes the simple operating system
//   Schedule three independent threads using preemptive round robin
//   Each thread rapidly toggles a pin on Port D and increments its counter
//   TIMESLICE is how long each thread runs

// Daniel Valvano
// January 29, 2015
// Modified by Ian Flury
// Oct. 3, 2023

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",

 Copyright 2015 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains

 */


#include "TM4C123GH6PM.h"
#include "tm4c123gh6pm_def.h"
#include <stdio.h>


#define TIMESLICE 32000 // thread switch time in system time units
                        // clock frequency is 16 MHz, switching time is 2ms
#define THREAD_SW_PER_SEC 500
#define COLOR_QUEUE_SIZE 10


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


// circular array, pointers, and functs
uint8_t color_queue[COLOR_QUEUE_SIZE];
uint8_t queue_size = 0;
int8_t front_ptr = -1; // init front and back pointers to -1 for empty queue
int8_t back_ptr = -1;
uint8_t color_displayed; // store the color currently on the LED

// full flag and current color
uint8_t queue_full_flag;
uint8_t current_color;

int Q_isEmpty(void);
int Q_isFull(void);
int8_t enqueue(uint8_t);
uint8_t dequeue(void);


// color strings for LCD
char *colors[] = {{"Red"}, {"Blu"}, {"Pur"}, {"Grn"}, {"Yel"}, {"Cyn"}, {"Wht"}};


// semaphores
int32_t queue_sem = 1;
int32_t current_color_sem = 1;
int32_t SW_sem = 1;
int32_t timer_sem = 1;


// Counter vars
int8_t timer_count = 15;
char *timer_chars[] = {{"00"}, {"01"}, {"02"}, {"03"}, {"04"}, {"05"}, {"06"}, {"07"}, {"08"}, {"09"}, {"10"}, {"11"}, {"12"}, {"13"}, {"14"}};


void timer_thread(void)
{
    OS_Wait(&timer_sem);
    timer_count--;
    if (timer_count == 0)
    {
        timer_count = 14;
    }
    OS_Signal(&timer_sem);
    OS_Sleep(THREAD_SW_PER_SEC); // ADC0 sleep for 1 second and decrement the counter
}

// LED thread responsible for dequeuing and outputting to led.
void led_thread(void)
{
    while (1)
    {
        OS_Wait(&queue_sem);
        int q_empty = Q_isEmpty();
        OS_Signal(&queue_sem);

        if (q_empty)
        {
            // if the queue is empty we have nothing to do, so forfeit our time.
            OS_Suspend();
        }
        else
        {
            // if the queue is not empty, pop the next color off
            // and display it on the led.
            OS_Wait(&queue_sem);
            uint8_t color_to_disp = dequeue();
            OS_Signal(&queue_sem);
            if (color_to_disp == 0)
            {
                OS_Suspend();
            }
            else
            {
                GPIO_PORTF_DATA_R = color_to_disp;
                color_displayed = color_to_disp;
                OS_Wait(&timer_sem);
                int8_t sleep_time = timer_count;
                OS_Signal(&timer_sem);
                // put the thread to sleep for timer_count seconds.
                OS_Sleep(sleep_time * THREAD_SW_PER_SEC);
                GPIO_PORTF_DATA_R = 0;
                color_displayed = 0;
            }
        }
    }
}

// output to LCD thread
void lcd_thread(void)
{
    while (1)
    {
        OS_Wait(&current_color_sem);
        uint8_t inst_color = (current_color >> 1);
        OS_Signal(&current_color_sem);

        Set_Position(0x00);
        OS_Wait(&queue_sem);
        uint8_t q_full = Q_isFull();
        uint8_t q_empty = Q_isEmpty();
        uint8_t color_live = color_displayed;
        OS_Signal(&queue_sem);

        if (color_displayed == 0)
        {
            Set_Position(0x40);
            Display_Msg("Input a Color!");
        }
        else
        {
            Set_Position(0x40);
            Display_Msg("C:");
            Display_Msg(colors[(color_live >> 1) - 1]);
            if (!q_empty)
            {
                Display_Msg(" N:");
                Display_Msg(colors[(color_queue[front_ptr] >> 1) - 1]);
            }
            else
            {
                Display_Msg(" N:?? ");
            }
            Display_Msg("   ");
        }

        // LCD counter logic
        OS_Wait(&timer_sem);
        int8_t timer_cur = timer_count;
        OS_Signal(&timer_sem);
        Display_Msg(timer_chars[timer_cur]);

        // logic responsible for displaying the instantanious color on the LCD.
        if (q_full)
        {
            Set_Position(0x00);
            Display_Msg("  BUFFER FULL!");
            OS_Suspend();
        }
        else
        {
            Set_Position(0x00);
            Display_Msg("Switches: ");
            switch (inst_color)
            {
            case 0x00:
                Display_Msg("     ");
                break;
            case 0x01:
                Display_Msg("red");
                break;
            case 0x02:
                Display_Msg("blu");
                break;
            case 0x03:
                Display_Msg("pur");
                break;
            case 0x04:
                Display_Msg("grn");
                break;
            case 0x05:
                Display_Msg("yel");
                break;
            case 0x06:
                Display_Msg("cyn");
                break;
            case 0x07:
                Display_Msg("wht");
                break;
            default:
                break;
            };
        }
    }
}

// Read switches thread
void sw_thread(void)
{
    uint32_t SW5_flag = 0;
    uint32_t last_SW5 = 0;
    while (1)
    {
        uint32_t last = GPIO_PORTD_DATA_R;
        while (last == 0)
        {
            last = GPIO_PORTD_DATA_R;
        }
        // button has been pressed, wait for a bit and then read again.
        OS_Sleep(50);
        uint32_t switches = GPIO_PORTD_DATA_R;
        SW5_flag = switches & 0x01;

        OS_Wait(&current_color_sem);
        current_color = (switches & 0x0E);
        OS_Signal(&current_color_sem);

        if (((switches & 0x01) == 1) && (last_SW5 == 0))
        {
            OS_Wait(&queue_sem);
            if (Q_isFull())
            {
                queue_full_flag = 1;
            }
            else
            {
                enqueue(switches & 0x0E);
            }
            OS_Signal(&queue_sem);
        }
        last_SW5 = SW5_flag;
    }
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

// returns 1 if queue is full, 0 otherwise
int Q_isFull()
{
    if ((front_ptr == back_ptr + 1) || (front_ptr == 0 && back_ptr == COLOR_QUEUE_SIZE - 1))
    {
        return 1;
    }
    return 0;
}

// returns 1 if queue is empty, 0 otherwise
int Q_isEmpty()
{
    if (front_ptr == -1)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

// insert an item into the queue
int8_t enqueue(uint8_t itemIn)
{
    if (Q_isFull())
    {
        return -1;
    }
    else
    {
        if (front_ptr == -1)
        {
            front_ptr = 0;
        }
        back_ptr = (back_ptr + 1) % COLOR_QUEUE_SIZE;
        color_queue[back_ptr] = itemIn;
        queue_size++;
        return 0;
    }
}

// returns the item at the front of the queue or 0 if empty
uint8_t dequeue()
{
    if (Q_isEmpty())
    {
        return 0;
    }
    else
    {
        uint8_t itemOut = color_queue[front_ptr];
        if (front_ptr == back_ptr)
        {
            front_ptr = -1;
            back_ptr = -1;
        }
        else
        {
            front_ptr = (front_ptr + 1) % COLOR_QUEUE_SIZE;
        }
        queue_size--;
        return itemOut;
    }
}
