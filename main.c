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
void OS_AddThreads(void f1(void), void f2(void), void f3(void), void f4(void));

void OS_Launch(uint32_t);
void OS_Wait(int32_t *S);
void OS_Signal(int32_t *S);
void OS_Sleep(uint32_t);
void OS_Suspend(void);
void OS_Signal(int32_t *s);
void OS_Wait(int32_t *s);

extern void Init_LCD_Ports(void);
extern void Display_Msg(char *Str);
extern void Set_Position(uint32_t POS);
extern void Init_LCD(void);
extern void Init_Keypad(void);
extern void Scan_Keypad(void);


extern void Hex2ASCII(char* output, int hex);
extern int ASCII2Hex(uint8_t* input);
extern uint8_t Read_Key(void);

uint8_t Key_ASCII;
uint32_t keypadASCII_buf[8];
int keypad_idx = 0;

uint32_t input_RPM; //input speed from keypad when entered
uint32_t display_input_RPM; //updated as user is pressing keys

int32_t Current_speed(int32_t Avg_volt);




void delayMs(int n)
{
	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < 6265; j++)
		{
			// do nothing for 1 ms
		}
	}
}

/*
	Application specific functions
*/

void init_adc_pins(void)
{
	// the ADC uses ports A,B,C,E
	SYSCTL->RCGCGPIO |= 0x17;
	while ((SYSCTL->PRGPIO & 0x17) == 0)
	{
	};
	GPIOC->DIR |= 0x80; // set PC7 for output
	GPIOC->DEN |= 0x80;
	GPIOC->DATA |= 0x80; 

	GPIOE->DIR &= ~0x0E; // set PE1-3 to be input NICK added the NOT
	GPIOE->DEN |= 0x0E; // DEN PE1-3

	GPIOB->DIR &= ~0x3C; // PB2-5 as input
	GPIOB->DEN |= 0x3C;

	GPIOA->DIR &= ~0xC0;
	GPIOA->DEN |= 0xC0; // PA6-7 as input
}

// configure PWM module 1 to generate a PWM signal on
// PF2. PB2-3 control motor direction.
void Init_M1PWM6(int period_ms, int duty_cycle)
{
	// given a wave period in ms, compute the load value
	int load_val = (250000 / period_ms) - 1;

	SYSCTL->RCGCPWM |= 0x02;	// enable clock to PWM1
	SYSCTL->RCGCGPIO |= 0x20;	// enable clock to GPIOF
	SYSCTL->RCGCGPIO |= 0x02;	// enable clock to GPIOB
	delayMs(1);					// PWM1 seems to take a while to start
	SYSCTL->RCC |= 0x001E0000;	// use divider 64
	PWM1->_3_CTL = 0;			// disable PWM1_3 during configuration
	PWM1->_3_GENA = 0x00000C08; // low on load, high on CPMA down
	PWM1->_3_LOAD = load_val;
	PWM1->_3_CMPB = duty_cycle;
	PWM1->_3_CTL = 1;			// enable PWM1_3
	PWM1->ENABLE |= 0x40;		// enable PWM1
	GPIOF->DIR |= 0x04;			// set PF3 pins as output (LED) pin
	GPIOF->DEN |= 0x04;			// set PF3 pins as digital pins
	GPIOF->AFSEL |= 0x04;		// enable alternate function
	GPIOF->PCTL &= ~0x00000F00; // clear PF2 alternate function
	GPIOF->PCTL |= 0x00000500;	// set PF2 alternate function to PWM
	GPIOB->DEN |= 0x03;			// PB0-1 as digital pins
	GPIOB->DIR |= 0x03;			// set PB0-1 as output
}

void MOT34_Dir_Set(void)
{
	GPIOB->DATA &= ~0x01;
	GPIOB->DATA |= 0x02;
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
	SYSCTL->RCGCTIMER |= 0x01;		// activate timer0
	TIMER0->CTL &= ~0x01;			// disable timer during setup
	TIMER0->CFG = 0x00000000;		// put timer in 16 bit mode
	TIMER0->TAMR = 0x00000002;		// config timer for periodic mode
	TIMER0->TAILR = period_us * 16; // set the count limit
	TIMER0->IMR |= 0x01;			// timer interrupts on time-out event
	NVIC_EN0_R |= 1 << 19;
	NVIC_PRI4_R |= 0x40000000; // set priority level 2
	TIMER0->ICR = 0x01;		   // clear the status bit
	TIMER0->CTL |= 0x01;
	// NOTE: here, right before this function returns, we would normally enable
	// interrupts globally, but that is done in StartOS in osasm_V2.s (so we don't)
}

int timer_count = 0;
int t2 = 0;
int t3 = 0;

// init to unlikely val
int8_t ADC_OUTPUT = 0xFF;
int ADC_OUTPUT_32 = 0xFFFFFFFF;
long ADC_sum = 0;
int current_speed = 0x0A0A; // replace this value with current speed
long speed_acc = 0;

// Read the value output by the ADC
void TIMER0A_Handler(void)
{
	// NOTE: we should either make this interrupt the highest priority in the system,
	//			 or disable all other interrupts while servicing it to prevent a critical code
	// 			 section from being interrupted.
	TIMER0->ICR = 0x01; // ack the interrupt.

	GPIOC->DATA &= ~0x80;
	GPIOC->DATA |= 0x80;

	while((GPIOE->DATA &0x02) == 0)
	{
	}
	ADC_OUTPUT = (GPIOA->DATA & 0xC0) | (GPIOB->DATA & 0x3C) | ((GPIOE->DATA & 0x0C) >> 2);
	
	ADC_OUTPUT_32 = 0;
	if(ADC_OUTPUT >= 0)
	{
		ADC_OUTPUT_32 = ADC_OUTPUT;
	}
	else
	{
		ADC_OUTPUT_32 = ADC_OUTPUT | 0xFFFFFF00;
	}
	ADC_sum += ADC_OUTPUT_32;
	timer_count++;

	if (timer_count > 99)
	{
		float ADC_avg = (float)ADC_sum / (float)timer_count;
		int v_avg = (int)(ADC_avg * 10000.0 / 127.0);
		current_speed = Current_speed(v_avg);
		speed_acc = speed_acc + current_speed;
		ADC_sum = 0;
		timer_count = 0;
	}
}

void lcd_thread(void)
{

  while (1) {
	  
	  
		/*Routine to print all necessary values to LCD*/
		Set_Position(0x00);
		Display_Msg("Input RPM:");


		
		char RPM_str[10];
		char* RPM_ptr = RPM_str;
		//RPM_str[4] = '\0';
		Hex2ASCII(RPM_ptr, display_input_RPM);

		Display_Msg(RPM_ptr);

		Set_Position(0x40);
		Display_Msg("T:");


		//int target_speed =input_RPM;
		char tgt_str[10];
		char* tgt_ptr = tgt_str;
		//tgt_str[4] = '\0'; //to fix overflow error
		Hex2ASCII(tgt_ptr, input_RPM);
		Display_Msg(tgt_ptr);


		Set_Position(0x48);
		Display_Msg("C:");

		int cur_spd_avg = speed_acc / 100;
		speed_acc = 0;
		char cur_str[5];
		char* cur_ptr = cur_str;
		cur_str[4] = '\0';
		
		Hex2ASCII(cur_ptr, cur_spd_avg);
		Display_Msg(cur_ptr);
		
		OS_Sleep(500); //sleep for 1s is 500
	};
}

// LED thread responsible for dequeuing and outputting to led.
void thread2(void)
{

    while (1) {
		t2++;
		
			
		};
}

// Keypad thread responsible for reading and storing keypad input.
void keypad_thread(void)
{
	int save = 0; //flag to update target speed
    while (1) {
		//TODO: take care of case where enter is pressed 
			t3++;
			Scan_Keypad();
			
			if (Key_ASCII == '#') {
				save = 1;
			}
			else{
				if (keypad_idx >= 3){
					save = 1;
				}
				keypadASCII_buf[keypad_idx] = Key_ASCII;
				keypad_idx++;
			
					
			}
			
		
		//fill byte buffer
		uint8_t key_buf[] = {keypadASCII_buf[0], keypadASCII_buf[1], keypadASCII_buf[2], keypadASCII_buf[3]};
		
		//take care of 4th input separately since broken in code
		if (keypad_idx >= 4){
			display_input_RPM = display_input_RPM * 10 + (Key_ASCII)-48;
			
		}
		else{
			display_input_RPM = ASCII2Hex(key_buf);
		}
		
		
				
		if (save == 1){
			
			input_RPM = display_input_RPM;
			
			if(input_RPM >2400){
				input_RPM = 2400;
			}
			else if ((input_RPM < 400) && (input_RPM > 0)){
				input_RPM = 400;
			}
			keypad_idx = 0;
			save = 0;
			keypadASCII_buf[0] = 0;
			keypadASCII_buf[1] = 0;
			keypadASCII_buf[2] = 0;
			keypadASCII_buf[3] = 0;
		}
		
		OS_Sleep(50);//delay to act as debouncer Might change later for better implementation
			
	
		}

}
//thread for PI controller, should run every 10ms

int32_t speed_error = 0;
int32_t last_error = 0;
int32_t U,I,P,D;
int last_input_rpm;
float k_p = 0.2;// // original value:105/20;
float k_i = 0.05;// original value: 101.0/640;
float k_d = 0.001;
void controller_thread(void)
{
    while (1) {
			last_error = speed_error;
			speed_error = input_RPM - current_speed;
			P = k_p*speed_error;
			I = I + k_i*speed_error;
			D = k_d * ((speed_error - last_error)/ 0.01);
		
//			if(I < -500){
//				I = -500;
//			}
//			if (I > 4000){
//				I = 4000;
//			}
			
			U = P + I;
			
			if(U<400){
				U = 400;
			}
			if (U > 2400){
				U = 2400;
			}
			
			//manaully stop the motor
			if(input_RPM == 0){
				MOT34_Speed_Set(1);
			}
			else{
				MOT34_Speed_Set(U);
				if(last_input_rpm != input_RPM)
				{
					I = 0;
				}
				last_input_rpm = input_RPM;
			}
			//MOT34_Speed_Set(1200);
			OS_Sleep(5);
		};
}

void motor_init()
{
	Init_M1PWM6(100, 0);	   // init the motor speed to zero.
	MOT34_Dir_Set();		   // set the direction using PB0-1
	MOT34_Speed_Set(2500 / 2); // set the motor speed to 50%
}

int main(void)
{

	Init_LCD_Ports();
    Init_LCD();
	Init_Keypad();
	init_adc_pins();
	motor_init();
	
	OS_Init(); // initialize, disable interrupts, 16 MHz
	Init_Timer0A(100); 			// initalize for 100us interrupts

	OS_AddThreads(&lcd_thread, &thread2, &keypad_thread, &controller_thread);


	OS_Launch(TIMESLICE); // doesn't return, interrupts enabled in here
	return 0;			  // this never executes
}
