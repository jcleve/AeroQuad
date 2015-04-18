/*
  AeroQuad v3.0 - Febuary 2012
 www.AeroQuad.com
 Copyright (c) 2011 Ted Carancho.  All rights reserved.
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

// FlightControl.pde is responsible for combining sensor measurements and
// transmitter commands into motor commands for the defined flight configuration (X, +, etc.)


#ifndef _AQ_Navigator_H_
#define _AQ_Navigator_H_
#define LOG Serial2
#define TWOPI (2 * PI)
#define RAD2CM (180.0f*60.0f*185200.0f/PI)

// little wait to have a more precise fix of the current position since it's called after the first gps fix
#define MIN_NB_GPS_READ_TO_INIT_HOME 20  
#define MIN_DISTANCE_TO_REACHED 3000
#define GPS_SPEED_SMOOTH_VALUE 0.5
#define GPS_COURSE_SMOOTH_VALUE 0.5
#define POSITION_HOLD_ORIGIN 25.0 // in cm
#define MAX_POSITION_HOLD_CRAFT_ANGLE_CORRECTION 125.0
#define POSITION_HOLD_SPEED 50.0  
#define POSITION_HOLD_PLANE 500.0
#define MAX_YAW_AXIS_CORRECTION 200.0  

byte countToInitHome = 0;
unsigned long previousFixTime = 0;

boolean haveNewGpsPosition() {
  return (haveAGpsLock() && (previousFixTime != getGpsFixTime()));
}

void clearNewGpsPosition() {
  previousFixTime = getGpsFixTime();
}

boolean isHomeBaseInitialized() {
  return homePosition.latitude != GPS_INVALID_ANGLE;
}

float HOLD_CORRECTION_EXP_CURVE_BASE = 0.0;
float POSITION_HOLD_YINTERCEPT = 3.125;
void calculateHoldCorrectionExpCurveBase()
{
  float base = MAX_POSITION_HOLD_CRAFT_ANGLE_CORRECTION/POSITION_HOLD_YINTERCEPT;
  float exponent = 1/(POSITION_HOLD_PLANE-POSITION_HOLD_ORIGIN);
  HOLD_CORRECTION_EXP_CURVE_BASE = pow(base, exponent);
}

float getHoldCorrectionExpCurveWeight(float distance) {
  if(POSITION_HOLD_ORIGIN <= distance <= POSITION_HOLD_PLANE) {
    float base = HOLD_CORRECTION_EXP_CURVE_BASE;
    float exponent = distance-POSITION_HOLD_ORIGIN;
    return POSITION_HOLD_YINTERCEPT * pow(base, exponent) / MAX_POSITION_HOLD_CRAFT_ANGLE_CORRECTION;
  }
  return 0;
}

void initHomeBase() {
  if (countToInitHome < MIN_NB_GPS_READ_TO_INIT_HOME) {
    countToInitHome++;
  }
  else {
    homePosition.latitude = currentPosition.latitude;
    homePosition.longitude = currentPosition.longitude;
    homePosition.altitude = DEFAULT_HOME_ALTITUDE;  

    // Set the magnetometer declination when we get the home position set
    setDeclinationLocation(currentPosition.latitude,currentPosition.longitude);

    // Set reference location for Equirectangular projection used for coordinates
    setProjectionLocation(currentPosition);

    //evaluateMissionPositionToReach();
    missionPositionToReach.latitude = homePosition.latitude;
    missionPositionToReach.longitude = homePosition.longitude;
    missionPositionToReach.altitude = homePosition.altitude;
    
    calculateHoldCorrectionExpCurveBase();
  }  
}

/*
 Because we are using lat and lon to do our distance errors here's a quick chart:
 100 	= 1m
 1000 	= 11m	 = 36 feet
 1800 	= 19.80m = 60 feet
 3000 	= 33m
 10000       = 111m
 */



GeodeticPosition previousPosition = GPS_INVALID_POSITION;

float 	currentLat = 		0.0; 		// current position latitude
float 	currentLon = 		0.0; 		// current position longitude
float gpsLaggedSpeed = 		0.0;
float gpsLaggedCourse = 	0.0;
float currentSpeedRoll = 	0.0; 
float currentSpeedPitch = 	0.0;
float distanceX = 		0.0;
float distanceY = 		0.0;
float distanceToPreviousX =     0.0;
float distanceToPreviousY =     0.0;
float distanceToPrevious = 	0.0;
float angleToWaypoint = 	0.0;
float angleToWaypoint2 =	0.0;
float degreesToWaypoint = 	0.0;
float diffLon = 		0.0;
float diffLat = 		0.0;
float courseRads = 		0.0;

float maxSpeedToDestination = POSITION_HOLD_SPEED;
float maxCraftAngleCorrection = MAX_POSITION_HOLD_CRAFT_ANGLE_CORRECTION;

/** 
 * @return true if there is a mission to execute
 */
boolean haveMission() {
  return missionNbPoint != 0;
}

/**
 * Evaluate the position to reach depending of the state of the mission 
 */
void evaluateMissionPositionToReach() {

  if (waypointIndex == -1) { // if mission have not been started
    waypointIndex++;
  }

  if (waypointIndex < MAX_WAYPOINTS && distanceToDestination < MIN_DISTANCE_TO_REACHED) {
    waypointIndex++;
  }

  if (waypointIndex >= MAX_WAYPOINTS || 
    waypoint[waypointIndex].altitude == GPS_INVALID_ALTITUDE) { // if mission is completed, last step is to go home 2147483647 == invalid altitude

    missionPositionToReach.latitude = homePosition.latitude;
    missionPositionToReach.longitude = homePosition.longitude;
    missionPositionToReach.altitude = homePosition.altitude; 
  }
  else {

    missionPositionToReach.latitude = waypoint[waypointIndex].latitude;
    missionPositionToReach.longitude = waypoint[waypointIndex].longitude;
    missionPositionToReach.altitude = (waypoint[waypointIndex].altitude/100);

    if (missionPositionToReach.altitude > 2000.0) {
      missionPositionToReach.altitude = 2000.0; // fix max altitude to 2 km
    }
  }
}

/** 
 * Compute the distance to the destination, point to reach ***(in cm)***
 * @result is distanceToDestination
 */
// float distanceToDestination2 = 0.0;
//void computeDistanceToDestination(GeodeticPosition destination) {  
//
//  distanceX = (float)(currentPosition.longitude - destination.longitude) * cosLatitude * 1.113195;
//  distanceY = (float)(currentPosition.latitude - destination.latitude) * 1.113195;
//  distanceToDestination2  = sqrt(sq(distanceY) + sq(distanceX));
//
//  // log  
//  currentLat = currentPosition.latitude;
//  currentLon = currentPosition.longitude;  
//}

void getDistanceToHoldPosition(GeodeticPosition destination){

  ////* this is accurate, but slow! *////

  //  lat1 = fabs(GPS2RAD * positionHoldPointToReach.latitude);
  //  lon1 = fabs(GPS2RAD * positionHoldPointToReach.longitude);
  //  lat2 = fabs(GPS2RAD * currentPosition.latitude);
  //  lon2 = fabs(GPS2RAD * currentPosition.longitude);    
  //  distanceToDestination = 2.0f * asin(sqrt(sq(sin((lat1 - lat2) / 2.0f)) + cos(lat1) * cos(lat2) * sq(sin((lon1 - lon2) / 2.0f)))) * RAD2CM; 

  ////** this is fairly accurate and quite fast! **////

  distanceX = (float)(destination.longitude - currentPosition.longitude);
  distanceY = (float)(destination.latitude - currentPosition.latitude);
  distanceToDestination  = sqrt(sq(distanceY) + sq(distanceX));
  angleToWaypoint = RAD2DEG * (atan2(distanceX,distanceY)-trueNorthHeading);
  if(angleToWaypoint < 0){
    angleToWaypoint += 360;
  }

  // back to radians
  angleToWaypoint = radians(angleToWaypoint);

  // log  
  //currentLat = currentPosition.latitude;
  //currentLon = currentPosition.longitude;  
}

/**
 * Compute the current craft speed in cm per sec
 * @result are currentSpeedPitch and currentSpeedRoll
 */
void computeCurrentSpeedInCmPerSec() {

  // we haven't moved
  if(currentPosition.longitude == previousPosition.longitude && currentPosition.latitude == previousPosition.latitude) {
    distanceToPreviousX = 0;
    distanceToPreviousY = 0;
    distanceToPrevious = 0;
    currentSpeedRoll = 0;
    currentSpeedPitch =0;
  }
  else {
    // get the distance between this reading and the last
    distanceToPreviousX = (float)(currentPosition.longitude - previousPosition.longitude);
    distanceToPreviousY = (float)(currentPosition.latitude - previousPosition.latitude);
    distanceToPrevious = sqrt(sq(distanceToPreviousY) + sq(distanceToPreviousX)); 


    gpsLaggedSpeed = gpsLaggedSpeed * (GPS_SPEED_SMOOTH_VALUE) + distanceToPrevious * (1-GPS_SPEED_SMOOTH_VALUE);
    if (distanceToPreviousX != 0 || distanceToPreviousY != 0) {
      float tmp = degrees(atan2(distanceToPreviousX, distanceToPreviousY));
      if (tmp < 0) {
        tmp += 360; 
      }
      gpsLaggedCourse = (int)((float)gpsLaggedCourse*(GPS_COURSE_SMOOTH_VALUE) + tmp*100*(1-GPS_COURSE_SMOOTH_VALUE));
    }

    courseRads = radians(gpsLaggedCourse/100);
    currentSpeedRoll = sin(courseRads-trueNorthHeading)*gpsLaggedSpeed; 
    currentSpeedPitch = cos(courseRads-trueNorthHeading)*gpsLaggedSpeed;
  }
  previousPosition.latitude = currentPosition.latitude;
  previousPosition.longitude = currentPosition.longitude;
}

/*
Function Atn2(Y, X)
 ' Equivalent of spreadsheet Atan2(x,y)
 ' Note argument order!
 ' No value for x=y=0
 ' Returns result in -pi< Atn2 <= pi
 pi2 = Pi / 2
 If (Abs(Y) >= Abs(X)) Then
 Atn2 = Sgn(Y) * pi2 - Atn(X / Y)
 Else
 If (X > 0) Then
 Atn2 = Atn(Y / X)
 Else
 If (Y >= 0) Then
 Atn2 = Pi + Atn(Y / X)
 Else
 Atn2 = -Pi + Atn(Y / X)
 End If
 End If
 End If
 End Function
 */

/*
If number is:
 
 >0 - Sgn returns 1
 =0 - Sgn returns 0
 <0 - Sgn returns -1
 */
float sgn(float x) {
  if(x > 0) return 1.0;
  if(x == 0) return 0.0;
  if(x < 0) return -1.0;
}

float atn2(float y, float x) {
  float pi2 = PI / 2;
  if(abs(y) >= abs(x)) {
    return sgn(y) * pi2 - atan(x/y);
  }
  else {
    if(x > 0) {
      return atan(y/x);
    } 
    else {
      if( y >= 0) {
        return PI + atan(y/x); 
      }
      else {
        return -PI + atan(y/x); 
      }
    }
  }  
}

float mod(float y, float x) {
  float z = y - x * (int)(y / x);
  return z;
}

float ModCrs(float crs) {
  float x = TWOPI - mod(TWOPI - crs, TWOPI);
  return x;
}

float getBearing(float toLatRads, float toLonRads, float fromLatRads, float fromLonRads) {
  float x = sin(toLonRads - fromLonRads) * cos(toLatRads);
  float y = cos(fromLatRads) * sin(toLatRads) - sin(fromLatRads) * cos(toLatRads) * cos(toLonRads - fromLonRads);
  float z = degrees(atan2(x, y)-trueNorthHeading);
  if(z < 0)
    z+=360;
  //return ModCrs(z);
  
  return radians(z);
}

/**
 * compute craft angle in roll/pitch to adopt to navigate to the point to reach
 * @result are gpsRollAxisCorrection and gpsPitchAxisCorrection use in flight control processor
 */
void computeRollPitchCraftAxisCorrection() {

  // alternative method 
  //currentLonRad = radians(currentPosition.longitude);
  //currentLatRad = radians(currentPosition.latitude);
  //diffLon = currentLonRad - holdLonRad;

  // get angle between the orgin (hold point) and our current position
  //angleToWaypoint = getBearing(GPS2RAD * positionHoldPointToReach.latitude, GPS2RAD * positionHoldPointToReach.longitude, 
  //    GPS2RAD * currentPosition.latitude, GPS2RAD * currentPosition.longitude)-trueNorthHeading;  

  float x = sin(angleToWaypoint);
  float y = cos(angleToWaypoint);  

  float maxSpeedRoll = (maxSpeedToDestination*x*((float)distanceToDestination)); 
  float maxSpeedPitch = (maxSpeedToDestination*y*((float)distanceToDestination)); 
  maxSpeedRoll = constrain(maxSpeedRoll, -maxSpeedToDestination, maxSpeedToDestination);
  maxSpeedPitch = constrain(maxSpeedPitch, -maxSpeedToDestination, maxSpeedToDestination);  
  
  gpsRollAxisCorrection = updatePID(maxSpeedRoll, currentSpeedRoll, &PID[GPSROLL_PID_IDX]);
  gpsPitchAxisCorrection = updatePID(maxSpeedPitch, currentSpeedPitch , &PID[GPSPITCH_PID_IDX]);  
    
  // this works pretty well.
  float weight = constrain((distanceToDestination - POSITION_HOLD_ORIGIN)/POSITION_HOLD_PLANE, 0, 1);
  
  //float weight = constrain(getHoldCorrectionExpCurveWeight(distanceToDestination),0,1);
  
  gpsRollAxisCorrection = constrain(gpsRollAxisCorrection * weight, -maxCraftAngleCorrection, maxCraftAngleCorrection);
  gpsPitchAxisCorrection = constrain(gpsPitchAxisCorrection * weight, -maxCraftAngleCorrection, maxCraftAngleCorrection);    
}

/**
 * Process position hold
 */
void processPositionHold() 
{
  // set position hold speed
  maxSpeedToDestination = POSITION_HOLD_SPEED;

  // set position hold maximum correction angle
  maxCraftAngleCorrection = MAX_POSITION_HOLD_CRAFT_ANGLE_CORRECTION;

  // calculate speed
  computeCurrentSpeedInCmPerSec();	

  // calculate distance and azimuth to hold point
  // computeDistanceToDestination(positionHoldPointToReach);  
  getDistanceToHoldPosition(positionHoldPointToReach);

  // only correct our position if we are adequately removed from hold point
  if(distanceToDestination > POSITION_HOLD_ORIGIN){
    computeRollPitchCraftAxisCorrection();
  }
  else {
    // we are adequately close to the hold point, so do not correct position
    gpsRollAxisCorrection = 0;
    gpsPitchAxisCorrection = 0;
    //gpsRollAxisCorrection2 = 0;
    //gpsPitchAxisCorrection2 = 0;
  }

  gpsYawAxisCorrection = 0;    

  // LOG
//  LOG.print(micros());
//  LOG.print(",");
//  LOG.print(gpsData.fixtime);
//  LOG.print(",");
//  LOG.print(positionHoldPointToReach.latitude);
//  LOG.print(",");
//  LOG.print(positionHoldPointToReach.longitude);
//  LOG.print(",");
//  LOG.print(currentLat);
//  LOG.print(",");
//  LOG.print(currentLon);
//  LOG.print(",");  
//  LOG.print(distanceToPreviousX);
//  LOG.print(",");
//  LOG.print(distanceToPreviousY);
//  LOG.print(",");
//  LOG.print(distanceToPrevious);
//  LOG.print(",");
//  LOG.print(currentSpeedPitch);  
//  LOG.print(",");
//  LOG.print(currentSpeedRoll);
//  LOG.print(",");
//  LOG.print(distanceX);
//  LOG.print(",");
//  LOG.print(distanceY);
//  LOG.print(",");
//  LOG.print(distanceToDestination);  
//  LOG.print(",");  
//  LOG.print(trueNorthHeading);
//  LOG.print(",");
//  LOG.print(angleToWaypoint);
//  LOG.print(","); 
//  LOG.print(gpsRollAxisCorrection);
//  LOG.print(",");
//  LOG.print(gpsPitchAxisCorrection);
//  LOG.print(",");
//  LOG.print(gpsRollAxisCorrection2);
//  LOG.print(",");
//  LOG.print(gpsPitchAxisCorrection2);
//  LOG.print(",");      
//  LOG.print(gpsData.idlecount);
//  LOG.print(",");
//  LOG.print(gpsData.speed);
//  LOG.print(",");
//  LOG.print(gpsData.state);
//  LOG.println();
}

/**
 * Compute everything need to make adjustment to the craft attitude to go to the point to reach
 */
void processGpsNavigation() {
  if (haveAGpsLock() && haveNewGpsPosition()) {		
    if(!isHomeBaseInitialized()) {
      initHomeBase();
      clearNewGpsPosition();
      return;
    }

    if (positionHoldState == ON ) {
      processPositionHold();         
    }
    clearNewGpsPosition();   		
  }  
}




#endif









