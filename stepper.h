/*
 *  SwitecX25 Arduino Library
 *  Guy Carpenter, Clearwater Software - 2012
 *
 *  Licensed under the BSD2 license, see license.txt for details.
 *
 *  All text above must be included in any redistribution.
 */

#ifndef stepper_h
#define stepper_h

#define MAX_STEPS (315*3)

static const unsigned char stateCount;
unsigned char currentState;    // 6 steps 
long currentStep;      // step we are currently at
long targetStep;       // target we are moving to
unsigned int steps;            // total steps available
unsigned long time0;                  // time when we entered this state
unsigned int microDelay;       // microsecs until next state
unsigned short (*accelTable)[2]; // accel table can be modified.
unsigned int maxVel;           // fastest vel allowed
unsigned int vel;              // steps travelled under acceleration
int dir;                      // direction -1,0,1  
unsigned short stopped;               // true if stopped

unsigned long micros();
void stepper_init();
void writeIO();
void stepUp();
void stepDown();
void stepper_zero();
void advance();
void stepper_goto(unsigned int pos);
void stepper_update();
void updateBlocking();
void stepper_off();


#endif

