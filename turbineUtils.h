/*
 * turbineUtils.h
 *
 *  Created on: Nov 6, 2021
 *      Author: jens
 */

#ifndef TURBINECONTROLLER_TURBINEUTILS_H_
#define TURBINECONTROLLER_TURBINEUTILS_H_

#include <stdbool.h>

#define dumploadPin 2
#define fanPin 13
#define efficiency 0.9

#define VoutMax_ID 0
#define VinMax_ID 1
#define VoutMin_ID 2
#define IoutMax_ID 3
#define TipSpeedRatio_ID 4
#define Losses_ID 5
#define WattMax 6

#define RPM_TURN_ON 150
#define RPM_TURN_OFF 147
#define START_CHARGING_VOLTAGE_DIFF 2.

//#define BENCH_TEST

static float valFloat[] = { 14.4, 40., 9., 20., 5., 35., 500. };


#define myConstrain(x, a, b)\
  if (x<a) x=a; else if (x>b) x=b;

#define whileMS(x)\
  unsigned long xxxTime = millis() + x*TT;\
  while (millis() < xxxTime)

#define whileS(x)\
  unsigned long xxxTime = millis() + x*long(TT)*1000;\
  while (millis() < xxxTime)*/


bool spinUp_blocking(void);
void setPower(float val);
void enableBuck(void);
void disableBuck(void);
void freeWheel(void);
void controlledStop_blocking(void);
void setDumpload(bool val);
void shortPhases(void);
void makeAcknowledge(int directio, int commutations, int delayTime);
bool isRunning(unsigned long* refTime);
void breakNow(int voltageBegin, int voltageEnd, bool dump);

float getExpectedPower(float rpm);

void printHelp(void);
void dump(void);


#endif /* TURBINECONTROLLER_TURBINEUTILS_H_ */
