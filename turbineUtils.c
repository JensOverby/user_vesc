/*
 * turbineUtils.c
 *
 *  Created on: Nov 7, 2021
 *      Author: jens
 */

#include "turbineUtils.h"
#include <math.h>

//unsigned long timeStamp_verbose = 0;
//unsigned long rpm_time = 0;

//valFloat = float[7]; //{ 14.4, 40., 9., 20., 5., 35., 500. };


//bool positive = true;

bool spinUp_blocking()
{
	return true;
}

void setPower(float val)
{

}

void enableBuck()
{
  /*whileMS(300)
    sample;
  duty_cycle = INIT_DUTY_CYCLE;
  //duty_cycle = min_sync_pwm + 5;
  if (duty_cycle < 180)
    duty_cycle = 180;
  buckEnabled = true;
  pwm(duty_cycle);*/
}

void disableBuck()
{
  //pwmDisable();
  //buckEnabled = false;
}

void freeWheel()
{
  /*digitalWrite(A_SD, LOW);
  digitalWrite(A_IN, LOW);

  digitalWrite(B_SD, LOW);
  digitalWrite(B_IN, LOW);

  digitalWrite(C_SD, LOW);
  digitalWrite(C_IN, LOW);*/
}

void controlledStop_blocking()
{
/*  for (int h=1; h<10; h++)
  {
    for (int i=0; i<100; i++)
    {
      digitalWrite(A_SD, HIGH);
      digitalWrite(B_SD, HIGH);
      digitalWrite(C_SD, HIGH);
      waitMS(h);
      digitalWrite(A_SD, LOW);
      digitalWrite(B_SD, LOW);
      digitalWrite(C_SD, LOW);
      waitMS((10-h));
    }
  }*/
}

void setDumpload(bool val)
{

}

void shortPhases()
{
  freeWheel();
/*
  digitalWrite(A_SD, HIGH);
  digitalWrite(A_IN, LOW);

  digitalWrite(B_SD, HIGH);
  digitalWrite(B_IN, LOW);

  digitalWrite(C_SD, HIGH);
  digitalWrite(C_IN, LOW);*/
}

void makeAcknowledge(int directio, int commutations, int delayTime)
{
#ifdef BENCH_TEST
  Serial.println(F("BENCH TEST"));
  return;
#endif
  /*waitMS(300);

  for (int j = directio==1? 0 : commutations-1; j>-1 && j<commutations; j += directio)
  {
    bldc_step = j % 6;
    if (delayTime != 1)
    {
      Serial.print(F("Step = "));
      Serial.println(bldc_step);
    }
    for (motor_speed=10; motor_speed<50; motor_speed++)
    {
      bldc_move();
      waitMS(10);
    }
    waitMS(delayTime);
  }
  bldc_step = (bldc_step+directio) % 6;

  freeWheel();*/
}

bool isRunning(unsigned long* refTime)
{
	return false;
  /*unsigned int valuePhaseB = analogRead(A2);
  unsigned long now = millis();
  if (valuePhaseB > PHASE_THRESHOLD || simRun)
  {
    refTime = now;
    return true;
  }
  int count = (now-refTime)/(1000*TT);
  if (count > 10)
    return false;
  if (count > 2)
  {
    if (count != lastCount)
    {
      if (verboseLevel) Serial.print(F("countdown = "));
      if (verboseLevel) Serial.println(25 - count);
      lastCount = count;
    }
  }
  return true;*/
}



void printHelp()
{
  /*Serial.println(F("Commands:"));
  Serial.println(F("-----------------"));
  Serial.println(F("help | prints this message"));
  Serial.println(F("ack | toggle self startup"));
  Serial.println(F("spinup | spinup turbine now"));
  Serial.println(F("initstartwind | sample startup wind"));
  Serial.println(F("windsense <1/0> | wind sensor on/off"));
  Serial.println(F("sample | sample Vin,Iout,Vout"));
  Serial.println(F("verbose <1/0> | verbose on/off"));
  Serial.println(F("duty <value> | set duty (0-255)"));
  Serial.println(F("dumpload <V-begin> <V-end> | use dump resistor (optional params)"));
  Serial.println(F("simrun <1/0> | simulate turbine run"));
  Serial.println(F("settings | print settings"));
  Serial.println(F("setsetting <id> <val> | set setting by id,value"));
  Serial.println(F("storesettings | store settings"));
  Serial.println(F("sensorparams <threshold> <gustminsecs> <gustmaxsecs> <gustcount> | set wind sensor parameters"));
  Serial.println(F("minruntime <sec> | set minimum ok run time"));
  Serial.println(F("commutate <cnt> | commutate motor forward"));
  Serial.println(F("poweroffset <value> | spinup pwm offset"));
  Serial.println(F("stop <offset> | break to offset voltage above Vout. Default 0"));
  Serial.println();*/
}

#define RPM_TO_TIP_SPEED_FACTOR 0.120427718 // = circumference / 60, where circum=2*pi*r
//#define RPM_TO_WINDSPEED_PROPORTIONAL_FACTOR 0.01720396 // = circumference / (tip_speed_ratio*60), where tsr=7 and circum=2*pi*r
#define POWER_FACTOR 0.7935   // = 0.5*blade_efficiency*Area, where blade_efficiency Cp=0.38197 and Area=pi*r^2
// Radius r = 1.15 meter

//#define LOSSES 35

// Cp=0.38197 is Hugh Piggotts value, but is 0.3 in http://www.windandwet.com/windturbine/power_calc/index.php
// and 0.4 in http://www.ijsrp.org/research_paper_feb2012/ijsrp-feb-2012-06.pdf

float getExpectedPower(float rpm)
{
  float windspeed = rpm * RPM_TO_TIP_SPEED_FACTOR / valFloat[TipSpeedRatio_ID]; // Unit is meter/sec
  //float windspeed = rpm * RPM_TO_WINDSPEED_PROPORTIONAL_FACTOR; // Unit is meter/sec
  float power = POWER_FACTOR * powf(windspeed, 3) - valFloat[Losses_ID];
  if (power < 0)
    return 0.;
  return power;
}


float interpolate(float x0, float y0, float x1, float y1, float x) { return y0 + (x-x0)/(x1-x0) * (y1-y0); }
float getInterpolatedY(float x, float* pData, int* data_ptr)
{
  if (x > *pData)
    return *(pData+1);
  int data_sz = sizeof(pData)/4;
  if (x <= *(pData+data_sz-2))
    return *(pData+data_sz-1);
  while (*(pData+(*data_ptr)*2+2) > x)
    (*data_ptr)++;
  while (*(pData+(*data_ptr)*2) <= x)
    (*data_ptr)--;
  float y = interpolate(*(pData+(*data_ptr)*2),*(pData+(*data_ptr)*2+1),*(pData+(*data_ptr)*2+2),*(pData+(*data_ptr)*2+3),x);
  return y;
}

void breakNow(int voltageBegin, int voltageEnd, bool dump)
{
  /*digitalWrite(enableDriverPin, false);
  unsigned long lastTimeDuty = 0;
  float ramped_duty = 0;

  if (dump)
    if (verboseLevel) Serial.println(F("Dumping V -> amps/volt (dI/dV) curve (voltage,A/V):"));
  
  unsigned long now = millis(), nowInSeconds = now/(1000*TT);
  unsigned long timeOut = nowInSeconds + 20;
  while (true)
  {
    sample(0.1);
    now = millis(); nowInSeconds = now/(1000*TT);
    if (nowInSeconds > timeOut)
    {
      if (verboseLevel) Serial.println(F("Timeout"));
      break;
    }

    if (Vin_filter > voltageBegin)
      break;
  }

  timeOut = nowInSeconds + 20;
  digitalWrite(dumploadPin, true);
  while (true)
  {
    sample(0.1);
    now = millis(); nowInSeconds = now/(1000*TT);
    if (nowInSeconds > timeOut)
    {
      if (verboseLevel) Serial.println(F("Timeout"));
      break;
    }

    if((Vin_filter < voltageEnd) || (Vout_filter < valFloat[VoutMin_ID]) || (Vin_filter < (Vout_filter+0.4)))
      break;

    if (now > (lastTimeDuty + 50*TT))
    {
      lastTimeDuty = now;

      if (dump)
      {
        if (verboseLevel) Serial.print(Vin_filter);
        if (verboseLevel) Serial.print(F(","));
        if (verboseLevel) Serial.println(Iout_filter);
      }
    }
  }
  digitalWrite(dumploadPin, false);*/
}

void dump()
{
  /*float t = (millis() - timeStamp_verbose) / (1000.*TT);
  if (t > 2)
  {
    if (Serial.availableForWrite() >= 48)
    {
      switch (verboseLevel)
      {
        case 1:
          Serial.println(int(duty_cycle));
          break;
        case 2:
          {
            float outputWatt = Vout_filter*Iout_filter;
            if (duty_cycle == min_sync_pwm)
              Serial.print(F("duL"));
            else
              Serial.print(F("du"));
            Serial.print(int(duty_cycle));
            Serial.print(F(" Vi"));
            Serial.print(Vin_filter,1);
            Serial.print(F(" Vo"));
      #ifdef DEBUG
            Serial.print(Vout_filter_dbg,1);
      #else
            Serial.print(Vout_filter,1);
      #endif
            Serial.print(F(" Io"));
            Serial.print(Iout_filter,2);
            Serial.print(F(" W"));
            Serial.print(outputWatt,0);
            if (state == RUNNING_CHARGE)
            {
              Serial.print(F(" Ie"));
              Serial.print(proportionalError,2);
            }
            Serial.print(F(" rpm"));
            Serial.println(rpm_filter,0);
          }
          break;
        default:
          break;
      }
      //commutations = 0;
      timeStamp_verbose = millis();
      //Serial.println(timeStamp_verbose);
    }
    return true;
  }
  return false;*/
}
