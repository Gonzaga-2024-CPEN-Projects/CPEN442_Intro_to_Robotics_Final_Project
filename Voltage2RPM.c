// The function Current_speed converts an input voltage in mVolts (between 0 and 95000)
// To the corresponding DC motor speed in RPM. This is the speed of unloaded
// motor.

// John Tadrous
// October 2, 2020

#include "tm4c123gh6pm.h"
#include "tm4c123gh6pm_def.h"


int32_t Current_speed(int32_t Avg_volt){ // This function returns the current
                                           // DC motor RPM given the voltage in mV
  if (Avg_volt<= 1200) {return 0;}
  else {return ((21408*Avg_volt)>>16)-225;}
}
