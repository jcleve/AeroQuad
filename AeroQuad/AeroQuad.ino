/*
  AeroQuad v3.0.1 - February 2012
  www.AeroQuad.com
  Copyright (c) 2012 Ted Carancho.  All rights reserved.
  An Open Source Arduino based multicopter.

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

/****************************************************************************
   Before flight, select the different user options for your AeroQuad by
   editing UserConfiguration.h.

   If you need additional assistance go to http://www.aeroquad.com/forum.php
   or talk to us live on IRC #aeroquad
*****************************************************************************/

#include "UserConfiguration.h" // Edit this file first before uploading to the AeroQuad

//
// Define Security Checks
//



#if defined (AeroQuadMega_v2) || defined (AeroQuadMega_v21) || defined (MWCProEz30) || defined (Naze32Full) || defined (AeroQuadSTM32)
  #define USE_HORIZON_MODE
  #define HeadingMagHold		
  #define AltitudeHoldBaro	
  #define USE_TPA_ADJUSTMENT	

  #define BattMonitor			  
  
//  #define UseAnalogRSSIReader	
//  #define UseEzUHFRSSIReader	
//  #define UseSBUSRSSIReader		

  
  #define UseGPS		        

//  #define OSD
//  #define ShowRSSI                  // This REQUIRES a RSSI reader
//  #define PAL                       // uncomment this to default to PAL video
//  #define AUTODETECT_VIDEO_STANDARD // detect automatically, signal must be present at Arduino powerup!
//  #define CALLSIGN "AQ"             // Show (optional) callsign
//  #define ShowAttitudeIndicator     // Display the attitude indicator calculated by the AHRS
//  #define USUnits                   // Enable for US units (feet,miles,mph), leave uncommented for metric units (meter,kilometer,km/h)

//  #define OSD_SYSTEM_MENU           // Menu system, currently only usable with OSD or SERIAL_LCD

#endif

#if defined (Naze32) || defined (Naze32Full)
  #define AeroQuadSTM32
  #define BattMonitor	
  #define USE_HORIZON_MODE		
  #define USE_TPA_ADJUSTMENT  
#endif


#if defined(UseGPS) && !defined(AltitudeHoldBaro)
  #error "GpsNavigation NEED AltitudeHoldBaro defined"
#endif

#if defined(UseGPS) && !defined(HeadingMagHold)
  #error "GpsNavigation NEED HeadingMagHold defined"
#endif

#if defined(AutoLanding) && !defined(AltitudeHoldBaro)
  #error "AutoLanding NEED AltitudeHoldBaro defined"
#endif

#if defined(ReceiverSBUS) && defined(SlowTelemetry)
  #error "Receiver SWBUS and SlowTelemetry are in conflict for Seria2, they can't be used together"
#endif

#if defined (CameraTXControl) && !defined (CameraControl)
  #error "CameraTXControl need to have CameraControl defined"
#endif 


#include <EEPROM.h>
#include <Wire.h>
#include <GlobalDefined.h>
#include "AeroQuad.h"
#include "PID.h"
#include <AQMath.h>

#include <FourtOrderFilter.h>
#ifdef BattMonitor
  #include <BatteryMonitorTypes.h>
#endif

#include <vector3.h>

//********************************************************
//********************************************************
//********* HARDWARE GENERALIZATION SECTION **************
//********************************************************
//********************************************************

#ifdef AeroQuadSTM32
  #include "AeroQuad_STM32.h"
#endif

// default to 10bit ADC (AVR)
#ifndef ADC_NUMBER_OF_BITS
#define ADC_NUMBER_OF_BITS 10
#endif

//********************************************************
//****************** KINEMATICS DECLARATION **************
//********************************************************
#include "Kinematics.h"
#if defined(HeadingMagHold)
  #include "Kinematics_MARG.h"
  #include "Kinematics_ARG.h"
#else
  #include "Kinematics_ARG.h"
#endif



//********************************************************
//******************** RECEIVER DECLARATION **************
//********************************************************
#if defined(UseAnalogRSSIReader) 
  #include <AnalogRSSIReader.h>
#elif defined(UseEzUHFRSSIReader)
  #include <EzUHFRSSIReader.h>
#elif defined(UseSBUSRSSIReader)
  #include <SBUSRSSIReader.h>
#endif



//********************************************************
//********************** MOTORS DECLARATION **************
//********************************************************
#if defined(triConfig)
  #if defined (MOTOR_STM32)
    #define MOTORS_STM32_TRI
    #include <Motors_STM32.h>    
  #else
    #include <Motors_Tri.h>
  #endif
#elif defined(MOTOR_PWM)
  #include <Motors_PWM.h>
#elif defined(MOTOR_PWM_Timer)
  #include <Motors_PWM_Timer.h>
#elif defined(MOTOR_APM)
  #include <Motors_APM.h>
#elif defined(MOTOR_I2C)
  #include <Motors_I2C.h>
#elif defined(MOTOR_STM32)
  #include <Motors_STM32.h>    
#endif

//********************************************************
//******* HEADING HOLD MAGNETOMETER DECLARATION **********
//********************************************************
#if defined(HMC5843) 
  #include <Magnetometer_HMC5843.h>
#elif defined(SPARKFUN_9DOF_5883L) || defined(SPARKFUN_5883L_BOB) || defined(HMC5883L) || defined (Naze32Full)
  #include <Magnetometer_HMC5883L.h>
#endif

//********************************************************
//******* ALTITUDE HOLD BAROMETER DECLARATION ************
//********************************************************
#if defined(BMP085)
  #include <BarometricSensor_BMP085.h>
  #include <VelocityProcessor.h>
#elif defined(MS5611)
 #include <BarometricSensor_MS5611.h>
 #include <VelocityProcessor.h>
#endif
#if defined(XLMAXSONAR)
  #include <MaxSonarRangeFinder.h>
#endif 
//********************************************************
//*************** BATTERY MONITOR DECLARATION ************
//********************************************************
#ifdef BattMonitor
  #include <BatteryMonitor.h>
  #ifndef BattCustomConfig
    #define BattCustomConfig BattDefaultConfig
  #endif
  struct BatteryData batteryData[] = {BattCustomConfig};
#endif
//********************************************************
//************** CAMERA CONTROL DECLARATION **************
//********************************************************
// used only on mega for now
#if defined(CameraControl_STM32)
  #include <CameraStabilizer_STM32.h>
#elif defined(CameraControl)
  #include <CameraStabilizer_Aeroquad.h>
#endif

#if defined (CameraTXControl)
  #include <CameraStabilizer_TXControl.h>
#endif

//********************************************************
//****************** GPS DECLARATION *********************
//********************************************************
#if defined(UseGPS)
//  #if !defined(HeadingMagHold)
//    #error We need the magnetometer to use the GPS
//  #endif 
  #include <GpsAdapter.h>
  #include "GpsNavigator.h"
#endif

//********************************************************
//****************** OSD DEVICE DECLARATION **************
//********************************************************
#ifdef MAX7456_OSD     // only OSD supported for now is the MAX7456
  #include <Device_SPI.h>
  #include "OSDDisplayController.h"
  #include "MAX7456.h"
#endif

#if defined(SERIAL_LCD)
  #include "SerialLCD.h"
#endif

#ifdef OSD_SYSTEM_MENU
  #if !defined(MAX7456_OSD) && !defined(SERIAL_LCD)
    #error "Menu cannot be used without OSD or LCD"
  #endif
  #include "OSDMenu.h"
#endif


//********************************************************
//****************** SERIAL PORT DECLARATION *************
//********************************************************
#if defined(AeroQuadSTM32) && defined (SERIAL_USES_USB)
  #define SERIAL_PORT SerialUSB
  #undef BAUD
  #define BAUD
#elif defined (USE_WIRELESS_COMMUNICATION)
  #define SERIAL_PORT Serial1
#else
  #define SERIAL_PORT Serial
#endif

// Include this last as it contains objects from above declarations
#include "AltitudeControlProcessor.h"
#include "FlightControlProcessor.h"
#include "FlightCommandProcessor.h"
#include "HeadingHoldProcessor.h"
#include "DataStorage.h"

#if defined(UseGPS) || defined(BattMonitor)
  #include "LedStatusProcessor.h"
#endif  

#if defined(MavLink)
  #include "MavLink.h"
#else
  #include "SerialCom.h"
#endif



/*******************************************************************
 * Main setup function, called one time at bootup
 * initialize all system and sub system of the
 * Aeroquad
 ******************************************************************/
void setup() {
  SERIAL_BEGIN(BAUD);
  pinMode(LED_Green, OUTPUT);
  digitalWrite(LED_Green, LOW);
  
  initCommunication();
  readEEPROM(); 
  
  boolean firstTimeBoot = false;
  if (readFloat(SOFTWARE_VERSION_ADR) != SOFTWARE_VERSION) { // If we detect the wrong soft version, we init all EEPROM
    initializeEEPROM();
    writeEEPROM();
    firstTimeBoot = true;
  }
  
  initPlatform();
  #ifdef AeroQuadSTM32
    PWM_FREQUENCY = fastLoopSleepingDelay == 2000 ? 500 : 400;
  #endif
  initializeMotors(LASTMOTOR);
  (*initializeReceiver[receiverTypeUsed])();
  
  // Initialize sensors
  // If sensors have a common initialization routine
  // insert it into the gyro class because it executes first
  initializeGyro(); // defined in Gyro.h
  while (!calibrateGyro()); // this make sure the craft is still befor to continue init process
  initializeAccel(); // defined in Accel.h
  if (firstTimeBoot) {
    computeAccelBias();
    writeEEPROM();
  }
  setupFourthOrder();
  initSensorsZeroFromEEPROM();
  
  #ifdef HeadingMagHold
    vehicleState |= HEADINGHOLD_ENABLED;
    initializeMagnetometer();
    measureMagnetometer(0.0, 0.0);
    initializeKinematics(0.0, 0.0, -accelOneG, measuredMag[XAXIS], measuredMag[YAXIS], measuredMag[ZAXIS]);
  #else    
    initializeKinematics();
  #endif
  
  // Optional Sensors
  #ifdef AltitudeHoldBaro
    initializeBaro();
    vehicleState |= ALTITUDEHOLD_ENABLED;
  #endif
  
  #ifdef BattMonitor
    if (isBatteryMonitorEnabled) {
      vehicleState |= BATTMONITOR_ENABLED;
      initializeBatteryMonitor(sizeof(batteryData) / sizeof(struct BatteryData), batteryMonitorAlarmVoltage);
    }
  #endif
  
  #if defined(CameraControl)
    initializeCameraStabilization();
    vehicleState |= CAMERASTABLE_ENABLED;
  #endif

  #if defined(MAX7456_OSD)
    initializeSPI();
    initializeOSD();
  #endif
  
  #if defined(SERIAL_LCD)
    InitSerialLCD();
  #endif

  #if defined(UseGPS)
    if (isGpsEnabled) {
      vehicleState |= GPS_ENABLED;
      initializeGps();
    }
  #endif 

  #ifdef SlowTelemetry
     initSlowTelemetry();
  #endif
  
  #if defined (USE_TPA_ADJUSTMENT)
    userRateRollP = PID[RATE_XAXIS_PID_IDX].P;
    userRateRollI = PID[RATE_XAXIS_PID_IDX].I;
    userRateRollD = PID[RATE_XAXIS_PID_IDX].D;
    userRatePitchP = PID[RATE_YAXIS_PID_IDX].P;
    userRatePitchI = PID[RATE_YAXIS_PID_IDX].I;
    userRatePitchD = PID[RATE_YAXIS_PID_IDX].D;
    userYawP = PID[ZAXIS_PID_IDX].P;
    userYawI = PID[ZAXIS_PID_IDX].I;
    userYawD = PID[ZAXIS_PID_IDX].D;
  #endif

  previousTime = micros();
  digitalWrite(LED_Green, HIGH);
  safetyCheck = 0;
  
}

/*******************************************************************
 * Compute kinematics
 ******************************************************************/
void computerKinematics() {
  
  for (byte axis = XAXIS; axis <= ZAXIS; axis++) {
    filteredAccel[axis] = computeFourthOrder(meterPerSecSec[axis], &fourthOrder[axis]);
  }
  
  const double accelMagnetude = sqrt((filteredAccel[XAXIS]*filteredAccel[XAXIS]) + 
                                     (filteredAccel[YAXIS]*filteredAccel[YAXIS]) + 
                                     (filteredAccel[ZAXIS]*filteredAccel[ZAXIS])) - accelOneG;
                                     
  if (accelMagnetude > 1.15 || accelMagnetude < 0.85) {
    for (byte axis = XAXIS; axis <= ZAXIS; axis++) {
      float filterValue = meterPerSecSec[axis] - filteredAccel[axis];
      if (filteredAccel[axis] > meterPerSecSec[axis]) {
        filterValue = filteredAccel[axis] - meterPerSecSec[axis];
      }
      filteredAccel[axis] = filteredAccel[axis] - filterValue;
    }
  }
  
  #if defined (HeadingMagHold) 
    calculateKinematicsMAGR(gyroRate[XAXIS], gyroRate[YAXIS], gyroRate[ZAXIS], filteredAccel[XAXIS], filteredAccel[YAXIS], filteredAccel[ZAXIS], measuredMag[XAXIS], measuredMag[YAXIS], measuredMag[ZAXIS]);
    magDataUpdate = false;
  #else
    calculateKinematicsAGR(gyroRate[XAXIS], gyroRate[YAXIS], gyroRate[ZAXIS], filteredAccel[XAXIS], filteredAccel[YAXIS], filteredAccel[ZAXIS]);
  #endif

}

/*******************************************************************
 * Fast task 500Hz or 400hz or 100Hz user selectable
 ******************************************************************/
void processFastTask() {
  G_Dt = (currentTime - fastTaskPreviousTime) / 1000000.0;
  fastTaskPreviousTime = currentTime;

  evaluateGyroRate();  
  evaluateMetersPerSec();
  
  computerKinematics();
  
  processFlightControl();
}


/*******************************************************************
 * 100Hz task
 ******************************************************************/
void process100HzTask() {
  
  G_Dt = (currentTime - hundredHZpreviousTime) / 1000000.0;
  hundredHZpreviousTime = currentTime;

  #if defined (AltitudeHoldBaro)
    if (vehicleState & BARO_DETECTED)
    {
      if (altHoldInitCountdown > 0) {
        altHoldInitCountdown--;
      }
      else {
        const float filteredZAccel = -(meterPerSecSec[XAXIS] * kinematicCorrectedAccel[XAXIS] + 
                                       meterPerSecSec[YAXIS] * kinematicCorrectedAccel[YAXIS] + 
                                       meterPerSecSec[ZAXIS] * kinematicCorrectedAccel[ZAXIS]);
  
        computeVelocity(filteredZAccel, G_Dt);
        
        measureBaroSum();
        if (frameCounter % THROTTLE_ADJUST_TASK_SPEED == 0) {  //  50 Hz tasks
          evaluateBaroAltitude();
          estimatedAltitude = getBaroAltitude();
          computeVelocityErrorFromBaroAltitude(estimatedAltitude);
        }
      }
    }
  #endif
        
  #if defined(UseGPS)
    if (isGpsEnabled) {
      updateGps();
    }
  #endif      
  
  #if defined(CameraControl)
    moveCamera(kinematicsAngle[YAXIS],kinematicsAngle[XAXIS],kinematicsAngle[ZAXIS]);
    #if defined CameraTXControl
      processCameraTXControl();
    #endif
  #endif       
}

/*******************************************************************
 * 50Hz task
 ******************************************************************/
void process50HzTask() {
  G_Dt = (currentTime - fiftyHZpreviousTime) / 1000000.0;
  fiftyHZpreviousTime = currentTime;

  // Reads external pilot commands and performs functions based on stick configuration
  readPilotCommands(); 

  // ********************** Process Altitude hold **************************
  #if defined AltitudeHoldBaro
    processAltitudeControl();
  #else
    throttle = receiverCommand[receiverChannelMap[THROTTLE]];
    if (throttle > 1850)
    {
      throttle = 1850;
    }
  #endif
    
  #if defined (USE_TPA_ADJUSTMENT)
    processThrottlePIDAdjustment();
  #endif

  #if defined(UseAnalogRSSIReader) || defined(UseEzUHFRSSIReader) || defined(UseSBUSRSSIReader)
    readRSSI();
  #endif

  #if defined(UseGPS)
    if (isGpsEnabled) {
      processGpsNavigation();
    }
  #endif      
}

/*******************************************************************
 * 10Hz task
 ******************************************************************/
void process10HzTask1() {
  
  #if defined(HeadingMagHold)
  
    G_Dt = (currentTime - tenHZpreviousTime) / 1000000.0;
    tenHZpreviousTime = currentTime;
    if (vehicleState & MAG_DETECTED) {
      measureMagnetometer(kinematicsAngle[XAXIS], kinematicsAngle[YAXIS]);
      magDataUpdate = true;
    }
  #endif
}

/*******************************************************************
 * low priority 10Hz task 2
 ******************************************************************/
void process10HzTask2() {
  G_Dt = (currentTime - lowPriorityTenHZpreviousTime) / 1000000.0;
  lowPriorityTenHZpreviousTime = currentTime;
  
  #if defined(BattMonitor)
    if (vehicleState & BATTMONITOR_ENABLED) {
      measureBatteryVoltage(G_Dt*1000.0);
    }
  #endif

  // Listen for configuration commands and reports telemetry
  readSerialCommand();
  sendSerialTelemetry();
}

/*******************************************************************
 * low priority 10Hz task 3
 ******************************************************************/
void process10HzTask3() {
    G_Dt = (currentTime - lowPriorityTenHZpreviousTime2) / 1000000.0;
    lowPriorityTenHZpreviousTime2 = currentTime;

    #ifdef OSD_SYSTEM_MENU
      updateOSDMenu();
    #endif

    #ifdef MAX7456_OSD
      updateOSD();
    #endif
    
    #if defined(UseGPS) || defined(BattMonitor)
      processLedStatus();
    #endif
    
    #ifdef SlowTelemetry
      updateSlowTelemetry10Hz();
    #endif
}

/*******************************************************************
 * 1Hz task 
 ******************************************************************/
void process1HzTask() {
  #ifdef MavLink
    G_Dt = (currentTime - oneHZpreviousTime) / 1000000.0;
    oneHZpreviousTime = currentTime;
    
    sendSerialHeartbeat();   
  #endif
  static boolean ledHigh = false;
  digitalWrite(LED_Green, ledHigh ? HIGH : LOW);
  ledHigh = !ledHigh;
}

/*******************************************************************
 * Main loop funtions
 ******************************************************************/


void loop () {
  
  currentTime = micros();
  deltaTime = currentTime - previousTime;

  
  measureCriticalSensors();
  
  if (currentTime >= (fastTaskPreviousTime + fastLoopSleepingDelay)) {  // 2500 = 400Hz, 2000 = 500Hz, 10000 = 100Hz
    processFastTask();
  }

  // ================================================================
  // 100Hz task loop
  // ================================================================
  if (deltaTime >= 10000) {
    
    frameCounter++;
    
    process100HzTask();

    // ================================================================
    // 50Hz task loop
    // ================================================================
    if (frameCounter % TASK_50HZ == 0) {  //  50 Hz tasks
      process50HzTask();
    }

    // ================================================================
    // 10Hz task loop
    // ================================================================
    if (frameCounter % TASK_10HZ == 0) {  //   10 Hz tasks
      process10HzTask1();
    }
    else if ((currentTime - lowPriorityTenHZpreviousTime) > 100000) {
      process10HzTask2();
    }
    else if ((currentTime - lowPriorityTenHZpreviousTime2) > 100000) {
      process10HzTask3();
    }
    
    // ================================================================
    // 1Hz task loop
    // ================================================================
    if (frameCounter % TASK_1HZ == 0) {  //   1 Hz tasks
      process1HzTask();
    }
    
    previousTime = currentTime;
  }
  
  if (frameCounter >= 100) {
      frameCounter = 0;
  }
}



