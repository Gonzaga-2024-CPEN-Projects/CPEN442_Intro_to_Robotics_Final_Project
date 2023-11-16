// os.c
// Runs on LM4F120/TM4C123
// A very simple real time operating system with minimal features.
// Daniel Valvano
// January 29, 2015
// Edited by John Tadrous
// June 25, 2020

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2015


 Copyright 2015 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

#include "TM4C123GH6PM.h"
#include "tm4c123gh6pm_def.h"

/* #define NVIC_ST_CTRL_R          (*((volatile uint32_t *)0xE000E010))
#define NVIC_ST_CTRL_CLK_SRC    0x00000004  // Clock Source
#define NVIC_ST_CTRL_INTEN      0x00000002  // Interrupt enable
#define NVIC_ST_CTRL_ENABLE     0x00000001  // Counter mode
#define NVIC_ST_RELOAD_R        (*((volatile uint32_t *)0xE000E014))
#define NVIC_ST_CURRENT_R       (*((volatile uint32_t *)0xE000E018))
#define NVIC_INT_CTRL_R         (*((volatile uint32_t *)0xE000ED04))
#define NVIC_INT_CTRL_PENDSTSET 0x04000000  // Set pending SysTick interrupt
#define NVIC_SYS_PRI3_R         (*((volatile uint32_t *)0xE000ED20))  // Sys. Handlers 12 to 15 Priority
*/


// function definitions in osasm.s
void OS_DisableInterrupts(void); // Disable interrupts
void OS_EnableInterrupts(void);  // Enable interrupts
int32_t StartCritical(void);
void EndCritical(int32_t primask);
void Clock_Init(void);
void StartOS(void);

#define NUMTHREADS  4        // maximum number of threads
#define STACKSIZE   256      // number of 32-bit words in stack
struct tcb{
  int32_t *sp;       // pointer to stack (valid for threads not running
  struct tcb *next;  // linked-list pointer
	int32_t *Blocked;	// non zero if blocked by sem
	uint32_t Sleep;
};
typedef struct tcb tcbType;
tcbType tcbs[NUMTHREADS];
tcbType *RunPt;
int32_t Stacks[NUMTHREADS][STACKSIZE];

// ******** OS_Init ************
// initialize operating system, disable interrupts until OS_Launch
// initialize OS controlled I/O: systick, 16 MHz clock
// input:  none
// output: none
void OS_Init(void){
  OS_DisableInterrupts();
  Clock_Init();                 // set processor clock to 16 MHz
  NVIC_ST_CTRL_R = 0;         // disable SysTick during setup
  NVIC_ST_CURRENT_R = 0;      // any write to current clears it
  NVIC_SYS_PRI3_R =(NVIC_SYS_PRI3_R&0x00FFFFFF)|0xE0000000; // priority 7
}

void SetInitialStack(int i){
  tcbs[i].sp = &Stacks[i][STACKSIZE-16]; // thread stack pointer
  Stacks[i][STACKSIZE-1] = 0x01000000;   // thumb bit
  Stacks[i][STACKSIZE-3] = 0x14141414;   // R14
  Stacks[i][STACKSIZE-4] = 0x12121212;   // R12
  Stacks[i][STACKSIZE-5] = 0x03030303;   // R3
  Stacks[i][STACKSIZE-6] = 0x02020202;   // R2
  Stacks[i][STACKSIZE-7] = 0x01010101;   // R1
  Stacks[i][STACKSIZE-8] = 0x00000000;   // R0
  Stacks[i][STACKSIZE-9] = 0x11111111;   // R11
  Stacks[i][STACKSIZE-10] = 0x10101010;  // R10
  Stacks[i][STACKSIZE-11] = 0x09090909;  // R9
  Stacks[i][STACKSIZE-12] = 0x08080808;  // R8
  Stacks[i][STACKSIZE-13] = 0x07070707;  // R7
  Stacks[i][STACKSIZE-14] = 0x06060606;  // R6
  Stacks[i][STACKSIZE-15] = 0x05050505;  // R5
  Stacks[i][STACKSIZE-16] = 0x04040404;  // R4
}

//******** OS_AddThread ***************
// add three foregound threads to the scheduler
// Inputs: three pointers to a void/void foreground tasks
// Outputs: 1 if successful, 0 if this thread can not be added
int OS_AddThreads(void(*task0)(void),
                  void(*task1)(void),
                  void(*task2)(void),
								  void(*task3)(void))
{ 
	int32_t status;
  status = StartCritical();
  tcbs[0].next = &tcbs[1]; // 0 points to 1
  tcbs[1].next = &tcbs[2]; // 1 points to 2
  tcbs[2].next = &tcbs[3]; // 2 points to 3
	tcbs[3].next = &tcbs[0]; // 3 points to 0
									 
	// set thread Sleep and Blocked to default
	
	tcbs[0].Sleep = 0;
  tcbs[1].Sleep = 0;
  tcbs[2].Sleep = 0;
	tcbs[3].Sleep = 0;
	tcbs[0].Blocked = 0;
  tcbs[1].Blocked = 0;
  tcbs[2].Blocked = 0;
	tcbs[3].Blocked = 0;
	
  SetInitialStack(0); Stacks[0][STACKSIZE-2] = (int32_t)(task0); // PC
  SetInitialStack(1); Stacks[1][STACKSIZE-2] = (int32_t)(task1); // PC
  SetInitialStack(2); Stacks[2][STACKSIZE-2] = (int32_t)(task2); // PC
	SetInitialStack(3); Stacks[3][STACKSIZE-2] = (int32_t)(task3); // PC
  RunPt = &tcbs[0];       // thread 0 will run first
  EndCritical(status);
  return 1;               // successful
}

///******** OS_Launch ***************
// start the scheduler, enable interrupts
// Inputs: number of 60ns clock cycles for each time slice
//         (maximum of 24 bits)
// Outputs: none (does not return)
void OS_Launch(uint32_t theTimeSlice){
  NVIC_ST_RELOAD_R = theTimeSlice - 1; // reload value
  NVIC_ST_CTRL_R = 0x00000007; // enable, core clock and interrupt arm
  StartOS();                   // start on the first task
}


void Clock_Init(void){
	SYSCTL_RCC_R|=0x810;
	SYSCTL_RCC_R&=~(0x400020);
}

void Scheduler(void) {
	tcbType *pt;
	pt=RunPt;
	if (NVIC_ST_CTRL_R & 0x10000){
	// full thread time (TIMESLICE) has passed
		while (pt->next != RunPt){
			pt=pt->next;
			if (pt->Sleep){
				pt->Sleep=(pt->Sleep)-1;
			}
		}
	}
	
	// Call happened due to OS_Suspend() do this:
	RunPt = RunPt->next; // skip at least one
	while((RunPt->Sleep)||(RunPt->Blocked)){
		RunPt = RunPt->next;
		// find one not sleeping and not blocked
	}
}

void OS_Suspend(void) {
	NVIC_ST_CURRENT_R = 0;
	NVIC_INT_CTRL_R |= 0x04000000;
}

void OS_Wait(int32_t *S) {
	OS_DisableInterrupts();
	(*S) = (*S) - 1;
	if ((*S) < 0) {
		RunPt->Blocked = S;
		OS_EnableInterrupts();
		OS_Suspend();
	}
	OS_EnableInterrupts();
}

void OS_Signal(int32_t *S) {
	tcbType *Pt;
	OS_DisableInterrupts();
	(*S) = (*S) + 1;
	if ((*S) <= 0) {
		Pt = RunPt->next;
		while (Pt->Blocked != S) {
			Pt = Pt->next;
		}
		Pt->Blocked = 0;
	}
	OS_EnableInterrupts();
}

void OS_Sleep(uint32_t SleepCtr) {
	RunPt->Sleep = SleepCtr;
	OS_Suspend();
}
