/*
 *  SwitecX25 Arduino Library
 *  Guy Carpenter, Clearwater Software - 2012
 *
 *  Licensed under the BSD2 license, see license.txt for details.
 *
 *  All text above must be included in any redistribution.
 */

#include "stepper.h"
#include <util/delay.h>
#include "tiny485_syscfg.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdint.h>
// During zeroing we will step the motor CCW 
// with a fixed step period defined by RESET_DELAY_MICROSEC
#define RESET_DELAY_MICROSEC 950
// It is important to keep this large enough for a tender crashing against zero at stepper_zero().
//Is set too small the motor will probably not start to move correctly.

// This table defines the acceleration curve.
// 1st value is the speed step, 2nd value is delay in microseconds
// 1st value in each row must be > 1st value in subsequent row
// 1st value in last row should be == maxVel, must be <= maxVel
static unsigned short defaultAccelTable[][2] = {
  {   20, 3000},
  {   50, 1500},
  {  100, 1000},
  {  150,  800},
  {  300,  600}
};
#define DEFAULT_ACCEL_TABLE_SIZE (sizeof(defaultAccelTable)/sizeof(*defaultAccelTable))

// experimentation suggests that 400uS is about the step limit 
// with my hand-made needles made by cutting up aluminium from
// floppy disk sliders.  A lighter needle will go faster.

// partial step bit pattern for 6 steps/ 2degree step from datasheet
/*             PIN1
           PIN2 |
         PIN3 | |
       PIN4 | | |
          | | | |
          | | | |          COIL POLARITY
// State  3 2 1 0   Value coil2 coil1
// 0      1 0 0 1   0x9    +      +
// 1      0 0 0 1   0x1    0      +
// 2      0 1 1 1   0x7    -      0
// 3      0 1 1 0   0x6    -      -
// 4      1 1 1 0   0xE    0      -
// 5      1 0 0 0   0x8    +      0
*/
static const unsigned short PROGMEM stateMap[] = {0x9, 0x1, 0x7, 0x6, 0xE, 0x8};

static const unsigned char stateCount = 6;

#define PORT_P1  PORTD
#define DDR_P1   DDRD
#define BIT_P1   _BV(3)

#define PORT_P2  PORTD
#define DDR_P2   DDRD
#define BIT_P2   _BV(4)

#define PORT_P3  PORTD
#define DDR_P3   DDRD
#define BIT_P3   _BV(5)

#define PORT_P4  PORTD
#define DDR_P4   DDRD
#define BIT_P4   _BV(6)

#define PIN1_MASK  0b1000
#define PIN2_MASK  0b0100
#define PIN3_MASK  0b0010
#define PIN4_MASK  0b0001

#define MAX_STEPS (315*3) //315 degs full range, 3steps per deg

unsigned long ovf_cnt = 0;

ISR(TIMER0_OVF_vect) {
	ovf_cnt++; //overflow at TCNT0==0xFF
}

unsigned long micros()
/* microeconds since timer 0 start*/
{
       //256+4µs per overflow count + timer 0 counter *4µs
	return ( (ovf_cnt<<8) + TCNT0)*4;
}

void stepper_init()
{

// timer init for appropriate speed 
/* prescaler 64 for 4µs per cnt.
 overflow after 256*4µs = 1024µs*/
  TCCR0A = 0;
  TCCR0B = _BV(CS01)|_BV(CS00);  /* clkIO/64 */


  TIMSK0 = _BV(TOIE0); /* timer overflow interrupt enable */
  TCNT0 = 0;
 
 //output pins
  DDR_P1 |= BIT_P1;
  DDR_P2 |= BIT_P2;
  DDR_P3 |= BIT_P3;
  DDR_P4 |= BIT_P4;
  
  currentState=0;
  steps = MAX_STEPS;
  sei();
  dir = 0;
  vel = 0; 
  stopped = 1;
  currentStep = 0;
  targetStep = 0;

  accelTable = defaultAccelTable;
  maxVel = defaultAccelTable[DEFAULT_ACCEL_TABLE_SIZE-1][0]; // last value in table.
  
  stepper_zero();
  stepper_goto(MAX_STEPS/2);
}

void writeIO()
//set pins acc. to current state
{
     unsigned short mask = pgm_read_byte(stateMap+currentState);  
     
	if (mask & PIN1_MASK)
		PORT_P1 |= BIT_P1;
	else
		PORT_P1 &= ~BIT_P1;

	if (mask & PIN2_MASK) {
		PORT_P2 |= BIT_P2;
	} else {
		PORT_P2 &= ~BIT_P2;
	}
	if (mask & PIN3_MASK) {
		PORT_P3 |= BIT_P3;
	} else {
		PORT_P3 &= ~BIT_P3;		
	}
	if (mask & PIN4_MASK)
		PORT_P4 |= BIT_P4;
	else
		PORT_P4 &= ~BIT_P4;

}

void stepper_off() {
//turn off pins to reduce current through coils
	stopped=1;
	PORT_P1 &= ~BIT_P1;
	PORT_P2 &= ~BIT_P2;
	PORT_P3 &= ~BIT_P3;		
	PORT_P4 &= ~BIT_P4;
}
	
void stepUp()
{
  if (currentStep < steps) {
    currentStep++;
    currentState = (currentState + 1) % stateCount;
    writeIO();
  }
}

void stepDown()
{ 
  if (currentStep > 0) {
    currentStep--;
    currentState = (currentState + 5) % stateCount;
    writeIO();
  }
}

void stepper_zero()
{
  currentStep = steps - 1;
  unsigned int i;
  for (i=0;i<steps;i++) {
    stepDown();
    _delay_us(RESET_DELAY_MICROSEC);
  }
  currentStep = 0;
  targetStep = 0;
  vel = 0;
  dir = 0;
}

// This function determines the speed and accel
// characteristics of the motor.  Ultimately it 
// steps the motor once (up or down) and computes
// the delay until the next step.  Because it gets
// called once per step per motor, the calcuations
// here need to be as light-weight as possible, so
// we are avoiding floating-point arithmetic.
//
// To model acceleration we maintain vel, which indirectly represents
// velocity as the number of motor steps travelled under acceleration
// since starting.  This value is used to look up the corresponding
// delay in accelTable.  So from a standing start, vel is incremented
// once each step until it reaches maxVel.  Under deceleration 
// vel is decremented once each step until it reaches zero.

void advance()
{
  // detect stopped state
  if (currentStep==targetStep && vel==0) {
  //target reached
    stepper_off();
    stopped = 1;
    dir = 0;
    time0 = micros();
    return;
  }
  
  // if stopped, determine direction
  if (vel==0) {
    dir = currentStep<targetStep ? 1 : -1;
    // do not set to 0 or it could go negative in case 2 below
    vel = 1; 
  }
  
  if (dir>0) {
    stepUp();
  } else {
    stepDown();
  }
  
  // determine delta, number of steps in current direction to target.
  // may be negative if we are headed away from target
  int delta = dir>0 ? targetStep-currentStep : currentStep-targetStep;
  
  if (delta>0) {
    // case 1 : moving towards target (maybe under accel or decel)
    if (delta < vel) {
      // time to declerate
      vel--;
    } else if (vel < maxVel) {
      // accelerating
      vel++;
    } else {
      // at full speed - stay there
    }
  } else {
    // case 2 : at or moving away from target (slow down!)
    vel--;
  }
    
  // vel now defines delay
  unsigned char i = 0;
  // this is why vel must not be greater than the last vel in the table.
  while (accelTable[i][0]<vel) {
    i++;
  }
  microDelay = accelTable[i][1];
  time0 = micros();
}

void stepper_goto(unsigned int pos)
{
  // pos is unsigned so don't need to check for <0
  if (pos >= steps) pos = steps-1;
  targetStep = pos;
  if (stopped) {
    // reset the timer to avoid possible time overflow giving spurious deltas
    stopped = 0;
    time0 = micros();
    microDelay = 0;
  }
}

void stepper_update()
{
  if (!stopped) {
    unsigned long delta = micros() - time0;
    if (delta >= microDelay) {
      advance();
    }
  }
}


//This updateMethod is blocking, it will give you smoother movements, but your application will wait for it to finish
void updateBlocking()
{
  while (!stopped) {
    unsigned long delta = micros() - time0;
    if (delta >= microDelay) {
      advance();
    }
  }
}

