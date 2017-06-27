  //Contains the declaration of the state variables for the control loop  
#include <stdint.h>

//interrupt vars

volatile int U = 0;       //control effort (abs)
volatile uint32_t setpoint = 0;
volatile float r = 0.0;   //setpoint
volatile float y = 0.0;   // measured angle
volatile float v = 0.0;  // estimated velocity  (velocity loop)
volatile float yw = 0.0;  // "wrapped" angle (not limited to 0-360)
volatile float yw_1 = 0.0;
volatile float e = 0.0;   // e = r-y (error)
volatile float p = 0.0;   // proportional effort
volatile float i = 0.0;   // integral effort


volatile float u = 0.0;     //real control effort (not abs)
volatile float u_1 = 0.0;   //value of u at previous time step, etc...
volatile float e_1 = 0.0;   //these past values can be useful for more complex controllers/filters     
volatile float u_2 = 0.0;
volatile float e_2 = 0.0;
volatile float u_3 = 0.0;
volatile float e_3 = 0.0;
volatile long counter = 0;

volatile int32_t wrapCount = 0;  //keeps track of how many revolutions the motor has gone though (so you can command angles outside of 0-360)
volatile float y_1 = 0;

  
volatile long step_count = 0;  //For step/dir interrupt (closed loop)

volatile float ITerm;
volatile float DTerm;
