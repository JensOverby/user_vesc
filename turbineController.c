/*
 * turbineController.c
 *
 *  Created on: Nov 7, 2021
 *      Author: jens
 */

#include "ch.h"
#include "hal.h"
#include "turbineController.h"
#include "turbineUtils.h"
#include "turbine.h"
#include "mc_interface.h"

// Private types
typedef enum {
	INIT,
	WAITING,
	SPINUP,
	RUNNING_NO_CHARGE,
	CHARGING,
	STOPPED
	//BREAKING,
	//TEST
} tb_state_t;

// Private variables
static THD_WORKING_AREA(timer_thread_wa, 256);
static THD_FUNCTION(timer_thread, arg);
static THD_WORKING_AREA(control_loop_thread_wa, 256);
static THD_FUNCTION(control_loop_thread, arg);
static TB_Command_t cmd = CMD_IDLE;
static tb_state_t state = INIT;
static const float k = 0.02;
static const float k1 = 0.98;
static unsigned long millis = 0;

// Private variables (Flags)
static bool controlledStopFlag = true;
static bool shutDownFlag = false;
static bool noSpinUpFlag = false;
static bool spinUpFlag = false;
static bool powerMaxReachedFlag = false;
static bool spinUpSuccessFlag = false;


// Private variables (sampled values)
static float tb_converter_current_out = 0;
static float tb_converter_current_in = 0;
static float tb_rpm = 0;
static float tb_converter_voltage_out = 0;
static float tb_mosfets_temperature = 0;



void turbineController_data_handler(unsigned char* data, unsigned int len)
{
	unsigned int hashValue = 0;

	for (unsigned int idx=0; idx<len; idx++)
	{
		char c = (char)data[idx-1];
		hashValue += c;
		hashValue += (hashValue << 10);
		hashValue ^= (hashValue >> 6);

		cmd = (TB_Command_t)hashValue;

		switch (cmd)
		{
		case CMD_HELP: // help
			turbine_printf("CMD_HELP");
			return;
		case CMD_DUTY: // duty
			turbine_printf("CMD_DUTY");
			return;
		case CMD_DUMPLOAD: // dumpload
			turbine_printf("CMD_DUMPLOAD");
			return;
		case CMD_ACKNOWLEDGE:
			turbine_printf("CMD_ACKNOWLEDGE");
			return;
		case CMD_SPINUP:
			turbine_printf("CMD_SPINUP");
			spinUpFlag = false;
			return;
		case CMD_SAMPLE:
			turbine_printf("CMD_SAMPLE");
			return;
		case CMD_PRINTSETTINGS:
			turbine_printf("CMD_PRINTSETTINGS");
			return;
		case CMD_SET_SETTING:
			turbine_printf("CMD_SET_SETTING");
			return;
		case CMD_STORE_SETTINGS:
			turbine_printf("CMD_STORE_SETTINGS");
			return;
		case CMD_COMMUTATE:
			turbine_printf("CMD_COMMUTATE");
			return;
		case CMD_STOP:
			turbine_printf("CMD_STOP");
			return;
		default:
			break;
		}
	}
	cmd = CMD_ERROR;
}


/*
SETUP_VALUES() {
    temp_mos = 0.0;
    temp_motor = 0.0;
    current_motor = 0.0;
    current_in = 0.0;
    duty_now = 0.0;
    rpm = 0.0;
    speed = 0.0;
    v_in = 0.0;
    battery_level = 0.0;
    amp_hours = 0.0;
    amp_hours_charged = 0.0;
    watt_hours = 0.0;
    watt_hours_charged = 0.0;
    tachometer = 0.0;
    tachometer_abs = 0.0;
    position = 0.0;
    fault_code = FAULT_CODE_NONE;
    vesc_id = 0;
    num_vescs = 0;
    battery_wh = 0.0;
    odometer = 0;
    */

void sample(void)
{
	// Turbine Converter Output Current
	float tmp = mc_interface_get_tot_current_in_filtered();
	tb_converter_current_out = k1*tb_converter_current_out + k*tmp;

	// Turbine Converter Input Current
	tmp = mc_interface_read_reset_avg_motor_current();
	tb_converter_current_in = k1*tb_converter_current_in + k*tmp;

	// Turbine RPM
	tmp = mc_interface_get_rpm();
	tb_rpm = k1*tb_rpm + k*tmp;

	// Turbine Converter Output Voltage
	tmp = GET_INPUT_VOLTAGE();
	tb_converter_voltage_out = k1*tb_converter_voltage_out + k*tmp;

	// Turbine Converter Mosfets Temperature
	tmp = mc_interface_temp_fet_filtered();
	tb_mosfets_temperature = k1*tb_mosfets_temperature + k*tmp;


	/*float fanAverageWatt = 0;
	float Iout_raw, Vin_raw, Vout_raw;
	pinMode(fanPin, INPUT);
	fanAverageWatt = 0;

	{
	  fanAverageWatt += (0.95*fanAverageWatt + 0.05*Vout_filter*Iout_filter);
	  if (fanAverageWatt > 50)
	  {
		pinMode(fanPin, OUTPUT);
		digitalWrite(fanPin, false);
	  }
	  else
	  {
		pinMode(fanPin, INPUT);
	  }
	}*/
}

	// Låsning af faser: CONTROL_MODE_HANDBRAKE


static THD_FUNCTION(timer_thread, arg) {
	(void)arg;
	for(;;) {
		millis++;
		chThdSleepMilliseconds(1);
	}
}


#define EVERY_X_SECS if (millis > nextVerboseTime)
#define EVERY_X_SECS_DUMP if ((millis+1000) > nextVerboseTime)

static THD_FUNCTION(control_loop_thread, arg) {
	(void)arg;

	chRegSetThreadName("TurbineController timer");
	const float power_step = 0.1;
	float power_setpoint = 0;
	//unsigned long millis = chTimeI2MS( chVTGetSystemTime() );
	unsigned int minRunTimeInSeconds = 30;
	state = INIT;


	unsigned long spinupTime, runningTime=0;
	unsigned int noPowerTimeStampInSeconds = 0;
	//unsigned int powerRiseTimeStampInSeconds = 0;
	unsigned int voltageTooHighTimeStampInSeconds = 0;
	unsigned int nextVerboseTime = 0;


	for(;;) {

		sample();

		switch(state)
		{
			case INIT:
			{
				//EVERY_X_SECS turbine_printf("INIT state");

				if (isRunning(&millis))
					controlledStop_blocking();
				controlledStopFlag = false;
				shortPhases();

				if (spinUpFlag) {
					runningTime = millis;
					state = WAITING;
					//turbine_printf("Entering WAITING state");
					break;
				}
			}
			break;

			case WAITING:
			{
				EVERY_X_SECS turbine_printf("WAITING state");

				if (controlledStopFlag || noSpinUpFlag) {
					controlledStopFlag = false;
					noSpinUpFlag = false;
					state = INIT;
					turbine_printf("Entering INIT state");
					break;
				}

				if (spinUpFlag)
				{
					spinUpFlag = false;
					state = SPINUP;
					turbine_printf("Entering SPINUP state");
					break;
				}
			}
			break;

			case SPINUP:
			{
				#ifdef BENCH_TEST
				break;
				#endif

				freeWheel();

				spinUpSuccessFlag = spinUp_blocking();

				freeWheel();

				/*for (int i=0; i<counter; ++i)
				{
				  Serial.println(debArray[i]);
				}
				Serial.print("comstate = ");
				Serial.println(com_state);*/

				spinupTime = millis;
				runningTime = millis;

				turbine_printf("Entering RUNNING_NO_CHARGE state");
				state = RUNNING_NO_CHARGE;
			}
			break;

			case RUNNING_NO_CHARGE:
			{
				EVERY_X_SECS turbine_printf("RUNNING_NO_CHARGE state");

				if (!isRunning(&runningTime))
				{
					turbine_printf("Entering STOPPED state");
					state = STOPPED;
					break;
				}

				if (tb_rpm > RPM_TURN_ON)
				//if (Vin_filter > Vout_filter + START_CHARGING_VOLTAGE_DIFF)
				{
					if ((tb_converter_voltage_out > valFloat[VoutMin_ID]) && (tb_converter_voltage_out < valFloat[VoutMax_ID]))
					{
						turbine_printf("Entering CHARGING state");
						enableBuck();
						state = CHARGING;

						noPowerTimeStampInSeconds = millis;
						//powerRiseTimeStampInSeconds = millis;
						power_setpoint = 0;
						break;
					}
				}

				EVERY_X_SECS_DUMP dump();
			}
			break;

			case CHARGING:
			{
				if (!isRunning(&runningTime)) {
					setDumpload(false);
					disableBuck();
					turbine_printf("Entering STOPPED state");
					state = STOPPED;
					break;
				}

				bool ok = true;

				if (tb_converter_current_out < -0.5) {
					turbine_printf("I_out < -0.5");
					ok = false;
				}

				/*if (Vin_filter <= (Vout_filter+0.1)) {
					turbine_printf("V_in <= 0.1+V_out: %.2f <= 0.1+%.2f", Vin_filter, Vout_filter);
					ok = false;
				}*/

				if (tb_converter_voltage_out < valFloat[VoutMin_ID]) {
					turbine_printf("Vout_filter < VoutMin: %.2f < %.2f", (double)tb_converter_voltage_out, (double)valFloat[VoutMin_ID]);
					ok = false;
				}

				if (tb_rpm < RPM_TURN_OFF) {
					turbine_printf("rpm < RPM_TURN_OFF: %d < %d", (int)tb_rpm, RPM_TURN_OFF);
					ok = false;
				}

				if (shutDownFlag) {
					if (tb_converter_current_out < 0.08f) {
						if (millis > noPowerTimeStampInSeconds+8000) {
							turbine_printf("I < 0.08 timeout");
							ok = false;
						}
					}
					else {
						shutDownFlag = false;
						noPowerTimeStampInSeconds = millis;
					}
				}
				else if (tb_converter_current_out < 0.08) {
					if (millis > noPowerTimeStampInSeconds+10000) {
						noPowerTimeStampInSeconds = millis;
						shutDownFlag = true;
						turbine_printf("NoPower timestamp at %.2f sec", (double)(noPowerTimeStampInSeconds/1000.));
					}
				}

				if (!ok) {
					setDumpload(false);
					disableBuck();
					state = RUNNING_NO_CHARGE;
					shutDownFlag = false;
					chThdSleepMilliseconds(1000); // Give open circuit voltage time to rise
					turbine_printf("Entering RUNNING_NO_CHARGE state");
				}
				else {





				  // Real real-time section

				  float power_update = 0;

				  float powerReal = tb_converter_voltage_out*tb_converter_current_out;

		#ifndef BENCH_TEST
				  if (tb_converter_voltage_out > valFloat[VoutMax_ID] || powerReal > valFloat[WattMax]) {
					if (!powerMaxReachedFlag) {
					  voltageTooHighTimeStampInSeconds = millis;
					  powerMaxReachedFlag = true;
					}
					else {
					  if (tb_converter_voltage_out > valFloat[VoutMax_ID]+0.5)
						setDumpload(true);

					  if (millis > voltageTooHighTimeStampInSeconds+1000)
						setDumpload(true);
					}
					power_update = +1;
				  }
				  else
		#endif
				  {
					if (powerMaxReachedFlag) {
						setDumpload(false);
						powerMaxReachedFlag = false;
					}
					float powerExp = getExpectedPower(tb_rpm);
					power_update = powerExp - powerReal;
				  }


				  // Update of power control setpoint
				  if (power_update > 0)
					power_setpoint += power_step;
				  else
					power_setpoint -= power_step;
				  if (power_setpoint < 0)
					  power_setpoint = 0;

				  setPower(power_setpoint);
				}

				EVERY_X_SECS_DUMP dump();
			}
			break;

			case STOPPED:
			{

				unsigned int timeOfRunInSeconds = (runningTime - spinupTime)/1000;
				turbine_printf("Turbine running time: %d sec", timeOfRunInSeconds);

				if (!spinUpSuccessFlag)
					turbine_printf("commutation error");
				else if (timeOfRunInSeconds < minRunTimeInSeconds)
					turbine_printf("Up time < %d sec -> startup failed", minRunTimeInSeconds);
				else
					turbine_printf("Up time > %d sec -> startup succeeded", minRunTimeInSeconds);

				turbine_printf("Entering WAITING state");
				runningTime = millis;
				state = WAITING;
			}
			break;
		}


		EVERY_X_SECS nextVerboseTime += 2000;
		chThdSleepMilliseconds(2);
		//millis = chTimeI2MS( chVTGetSystemTime() );
	}
}

void turbineController_init() {
	/*comm_usb_serial_init();
	packet_init(send_packet_raw, process_packet, PACKET_HANDLER);

	chMtxObjectInit(&send_mutex);*/

	turbine_set_callbacks(&sample, &turbineController_data_handler);

	// Threads
	chThdCreateStatic(timer_thread_wa, sizeof(timer_thread_wa), NORMALPRIO, timer_thread, NULL);
	chThdCreateStatic(control_loop_thread_wa, sizeof(control_loop_thread_wa), NORMALPRIO, control_loop_thread, NULL);
}







/*PROGMEM const float factory_powercurve[] =  {26, 17.29,
                                             25, 17.21,
                                             24, 16.48,
                                             23, 14.77,
                                             22, 12.02,
                                             21, 8.75,
                                             20, 5.77,
                                             19, 3.6,
                                             18, 2.25,
                                             17, 1.5,
                                             16, 1.11,
                                             15, 0.92,
                                             14, 0.85};*/
/*
bool execCommand()
{
  switch (commandBuffer[0])
  {
  case CMD_VERBOSE:
    {
      sscanf((char*)(commandBuffer+1), "%d", &verboseLevel);
      verboseLevel_old = verboseLevel;
    }
    break;
  case CMD_DUTY:
    {
      freeWheel();
      
      int pwmVal = -1;
      sscanf((char*)(commandBuffer+1), "%d", &pwmVal);

      if (pwmVal == -1)
        break;
      
      Serial.print(F("pwm duty = "));
      Serial.println(pwmVal);

      for (int i=0; i<100; ++i)
        sample();

      power_setpoint = pwmVal;
      pwm(power_setpoint);

      int counter = 0;
      while (!getCommand())
      {
        sample();

        int minimum_pwm = 255. * Vout_filter / Vin_filter;
        if (minimum_pwm < pwmMin)
          minimum_pwm = pwmMin;
        if (minimum_pwm > pwmMax || pwmVal < minimum_pwm)
        {
          pwmDisable();
          Serial.println(F("aborting, input voltage too low"));
          commandBuffer[0] = CMD_IDLE;
          return false;
        }

        counter++;
        if (counter>100)
        {
          counter=0;
          if (dump())
          {
            Serial.print(F("minpwm="));
            Serial.println(minimum_pwm);
          }
        }
        waitMS(1);
      }
      pwmDisable();
      //digitalWrite(enableDriverPin, false);
    }
    return true;
  case CMD_DUMPLOAD:
    {
      int voltageBegin = -1, voltageEnd = -1;
      sscanf((char*)(commandBuffer+1), "%d %d", &voltageBegin, &voltageEnd);
      if (voltageBegin == -1 || voltageEnd == -1)
        breakNow();
      else
        breakNow(voltageBegin, voltageEnd, true);
    }
    break;
  case CMD_STOP:
    {
      char offs[5]="0";
      sscanf((char*)(commandBuffer+1), "%s", offs);
      float off = atof(offs);
      digitalWrite(enableDriverPin, false);
      //commutations = 0;
      unsigned long stopTimeInMillis = millis()/TT + 10000;
      while (!getCommand())
      {
        sample(0.1);
        if((Vout_filter < valFloat[VoutMin_ID]) || (Vin_filter < (Vout_filter+off)))
          digitalWrite(dumploadPin, false);
        else
          digitalWrite(dumploadPin, true);

        unsigned long nowInMillis = millis()/TT;
        if (rpm_filter < 70 || nowInMillis > stopTimeInMillis)
          break;
      }
      digitalWrite(dumploadPin, false);

      controlledStop();

      digitalWrite(A_SD, HIGH);
      digitalWrite(B_SD, HIGH);
      digitalWrite(C_SD, HIGH);
      waitMS(2000);
      
      freeWheel();

    }
    noSpinUpFlag = true;
    break;
    
  case CMD_SAMPLE:
    {
      if (state > WAITING)
      {
        Serial.println(F("Command forbidden in charging state!"));
        break;
      }

      freeWheel();

      unsigned long nextDump = millis() + 1000*TT;
      while (!getCommand())
      {
        sample();
        if (millis() > nextDump)
        {
          nextDump = millis() + 500*TT;
          Serial.print(F("V_in="));
          Serial.print(Vin_filter);
          Serial.print(F(" I_out="));
          Serial.print(Iout_filter);
          Serial.print(F(" V_out="));
          Serial.print(Vout_filter);
          Serial.print(F(" rpm="));
          Serial.println(rpm_filter);
        }
      }
    }
    break;

  case CMD_DIAGNOSTICS:
    {
      if (state > WAITING)
      {
        Serial.println(F("Command forbidden in charging state!"));
        break;
      }

      int mode=0;
      sscanf((char*)(commandBuffer+1), "%d", &mode);

      freeWheel();

      while (!getCommand())
      {
        switch (mode)
        {
          case 0:
            digitalWrite(enableDriverPin, false);
            Serial.println(F("No mosfets conducting"));
            sampleAndDump(9, 500);
      
            Serial.println(F("High mosfet conducting"));
            pwmHigh(240);
            sampleAndDump(9, 500);

            pwmDisable();
            Serial.println(F("No mosfets conducting"));
            sampleAndDump(9, 500);
      
            Serial.println(F("Low mosfet conducting"));
            pwmLow(15);
            sampleAndDump(9, 500);

            pwmDisable();
            Serial.println(F("No mosfets conducting"));
            sampleAndDump(9, 500);
            break;
  
          case 1:
            checkPhase("A", A_IN, A_SD);
            checkPhase("B", B_IN, B_SD);
            checkPhase("C", C_IN, C_SD);
          break;
  
          case 2:
            freeWheel();
            Serial.println(F("Buck converter low mosfet ON"));
            pwmLow(15);
            sampleAndDump(9, 500);
          
            Serial.println(F("A_low & B_low"));
            digitalWrite(A_SD, HIGH);
            digitalWrite(B_SD, HIGH);
            sampleAndDump(9, 500);
      
            freeWheel();
            Serial.println(F("A_low & C_low"));
            digitalWrite(A_SD, HIGH);
            digitalWrite(C_SD, HIGH);
            sampleAndDump(9, 500);
      
            freeWheel();
            Serial.println(F("B_low & C_low"));
            digitalWrite(B_SD, HIGH);
            digitalWrite(C_SD, HIGH);
            sampleAndDump(9, 500);
  
            Serial.println(F("Buck converter no mosfets ON"));
            pwmDisable();
            sampleAndDump(9, 500);
            break;
        }
      }
    }
    break;

  case CMD_ACKNOWLEDGE:
    noSpinUpFlag = true;
    break;
  case CMD_SPINUP:
    spinupNow = true;
    break;
  case CMD_ENABLE_WIND_SENSOR:
    sscanf((char*)(commandBuffer+1), "%d", &enableWindSensor);
    break;
  case CMD_SIMRUN:
    {
      sscanf((char*)(commandBuffer+1), "%d", &simRun);
    }
    break;

  case CMD_PRINTSETTINGS:
    {
      Serial.println(F("SETTINGS:"));
      Serial.print(F("VoutMax")); Serial.println(valFloat[VoutMax_ID]);
      Serial.print(F("VinMax")); Serial.println(valFloat[VinMax_ID]);
      Serial.print(F("VoutMin")); Serial.println(valFloat[VoutMin_ID]);
      Serial.print(F("IoutMax")); Serial.println(valFloat[IoutMax_ID]);
      Serial.print(F("TipSpeedRatio")); Serial.println(valFloat[TipSpeedRatio_ID]);
      Serial.print(F("Losses")); Serial.println(valFloat[Losses_ID]);
      Serial.print(F("WattMax")); Serial.println(valFloat[WattMax]);

      if (verboseLevel_old != 0)
        verboseLevel_old = verboseLevel;
      verboseLevel = 0;
    }
    break;

  case CMD_SET_SETTING:
    {
      char tmp1[5]="-1";
      char tmp2[5]="-1";
      sscanf((char*)(commandBuffer+1), "%s %s", tmp1, tmp2);
      int id = atoi(tmp1);
      float val = atof(tmp2);
      if (id==-1 || val==-1)
      {
        Serial.println(F("wrong parameters!"));
        break;
      }
      valFloat[id] = val;
    }
    break;

  case CMD_STORE_SETTINGS:
    eepromUpdateFloat16(valFloat, sizeof(valFloat)/4);
    Serial.println(F("done."));
    break;
    
  case CMD_WIND_PARAMS:
    {
      char val[4][5] = {"-1","-1","-1","-1"};
      sscanf((char*)(commandBuffer+1), "%s %s %s %s", val[0], val[1], val[2], val[3]);
      bool is_ok=true;
      for (byte i=0; i<4; ++i)
      {
        if (atoi(val[i]) == -1)
        {
          Serial.println(F("wrong parameters!"));
          is_ok=false;
          break;
        }
      }
      if (!is_ok)
        break;
      pressureThreshold = atof(val[0]);
      gust_spacing_min = atof(val[1]);
      gust_spacing_max = atof(val[2]);
      required_number_of_gusts = atof(val[3]);
      Serial.print(F("parameters: threshold="));
      Serial.print(pressureThreshold);
      Serial.print(F(" gust_min_spacing="));
      Serial.print(gust_spacing_min);
      Serial.print(F("s gust_max_spacing="));
      Serial.print(gust_spacing_max);
      Serial.print(F("s gust_count="));
      Serial.println(required_number_of_gusts);
    }
    break;
  case CMD_MIN_RUNTIME:
    {
      sscanf((char*)(commandBuffer+1), "%d", &minRunTimeInSeconds);
    }
    break;
  case CMD_SAMPLE_STARTUP_WIND:
    {
      if (state > WAITING)
      {
        Serial.println(F("Command forbidden in charging state!"));
        break;
      }
      float pThres_old = pressureThreshold;
      pressureThreshold = 0.02;
      unsigned long oneSec = millis() + TT*1000;
      unsigned int sampleTimeInSeconds = 60;
      Serial.println(F("Sampling startup wind"));
      while (sampleTimeInSeconds > 0)
      {
        if (millis() > oneSec)
        {
          sampleTimeInSeconds--;
          oneSec += TT*1000;
        }
        if (getWindSpeedOK2())
        {
          Serial.print(F("threshold "));
          Serial.println(pressureThreshold);
          pressureThreshold += 0.02;
          sampleTimeInSeconds = 60;
        }
        if (getCommand())
        {
          pressureThreshold = pThres_old;
          commandBuffer[0] = CMD_IDLE;
          return true;
        }
      }
      pressureThreshold -= 0.02;
    }
    break;
  case CMD_COMMUTATE:
    {
      if (state > WAITING)
      {
        Serial.println(F("Command forbidden in charging state!"));
        break;
      }

      int count=0;
      sscanf((char*)(commandBuffer+1), "%d", &count);
      if (count > 0)
      {
        makeAcknowledge(1, count, 2000);
        shortPhases();
      }
    }
    break;
  case CMD_HELP:
    printHelp();
    verboseLevel_old = verboseLevel;
    verboseLevel = 0;
    break;
  default:
    break;
  }

  commandBuffer[0] = CMD_IDLE;
  return false;
}*/
