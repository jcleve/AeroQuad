
#define DEG_TO_RAD  0.017453292519943295769236907684886
#define radians(deg)            ((deg)*DEG_TO_RAD)
#define PI 3.14159265358979323846
#define TWOPI (2 * PI)
#define GPS2RAD (1.0f / 572957795.0f)
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define RAD2CM (180.0f*60.0f*185200.0f/PI)
#define RAD2DEG 57.2957795

// test 2, position 1 -> ~120 degrees
//float rawHoldLat = 355823637;
//float rawHoldLon = -785165370;
//float rawCurrLat = 355823776;
//float rawCurrLon = -785165760;
//float trueNorthHeading = -0.17f;

// test 2, position 2 -> ~240 degrees
//float rawHoldLat = 355823637;
//float rawHoldLon = -785165370;
//float rawCurrLat = 355823776;
//float rawCurrLon = -785165056;
//float trueNorthHeading = -0.08f;

// test 1, position 1 -> ~138 degrees
//float rawHoldLat = 355823626;
//float rawHoldLon = -785165792;
//float rawCurrLat = 355823776;
//float rawCurrLon = -785166144;
//float trueNorthHeading = -0.42f;

// test 1, position 2 -> ~240 degrees
//float rawHoldLat = 355823626;
//float rawHoldLon = -785165792;
//float rawCurrLat = 355823808;
//float rawCurrLon = -785165440;
//float trueNorthHeading = -0.09f;

// test 1, position 3 -> ~45 degrees
//float rawHoldLat = 355823626;
//float rawHoldLon = -785165792;
//float rawCurrLat = 355823456;
//float rawCurrLon = -785165952;
//float trueNorthHeading = -0.12f;

// test 1, position 4 -> ~300 degrees			-
float rawHoldLat = 355823626;
float rawHoldLon = -785165792;
float rawCurrLat = 355823488;
float rawCurrLon = -785165376;
float trueNorthHeading = 0.05f;

float decHoldLat = rawHoldLat/10000000;
float decHoldLon = rawHoldLon/10000000;
float decCurrLat = rawCurrLat/10000000;
float decCurrLon = rawCurrLon/10000000;


float holdLat = GPS2RAD * rawHoldLat;
float holdLon = GPS2RAD * rawHoldLon;
float currLat = GPS2RAD * rawCurrLat;
float currLon = GPS2RAD * rawCurrLon;

float cosLatitude = 0.9f;

void setup()
{
  Serial.begin(115200);
}

float d1, d2, d3, d4, d5, d6, d7, d8, d9;
float t1, t2, t3, t4, t5, t6, t7, t8, t9;
float b1, b2, b3, b4, b5, b6, b7, b8, b9;
void loop()
{
  float d1 = getDistance1(holdLat, holdLon, currLat, currLon);
  float d2 = getDistance2(holdLat, holdLon, currLat, currLon);
  float d3 = getDistance3(currLat, currLon, holdLat, holdLon);
  float d4 = getDistance4(holdLat, holdLon, currLat, currLon);
  float d5 = getDistance5(decHoldLat, decHoldLon, decCurrLat, decCurrLon);
  float d6 = getDistance6(rawHoldLat, rawHoldLon, rawCurrLat, rawCurrLon, cosLatitude);
  float d7 = getDistance7(rawHoldLat, rawHoldLon, rawCurrLat, rawCurrLon);
  float d8 = getDistance8(holdLat, holdLon, currLat, currLon, cosLatitude);
  float d9 = getDistance9(rawHoldLat, rawHoldLon, rawCurrLat, rawCurrLon);
  
  Serial.print("Distance1: ");
  Serial.print(t1);
  Serial.print(" - ");
  Serial.print( d1 , 10);
  Serial.print(" - ");
  Serial.println(b1, 10);
  
  Serial.print("Distance2: ");
  Serial.print(t2);
  Serial.print(" - ");
  Serial.print( d2, 10);
  Serial.print(" - ");
  Serial.println(b2, 10);
  
  Serial.print("Distance3: ");
  Serial.print(t3);
  Serial.print(" - ");
  Serial.print( d3, 10 );
  Serial.print(" - ");
  Serial.println(b3, 10);
  
  Serial.print("Distance4: ");
  Serial.print(t4);
  Serial.print(" - ");
  Serial.print( d4, 10 );
  Serial.print(" - ");
  Serial.println(b4, 10);
  
  Serial.print("Distance5: ");
  Serial.print(t5);
  Serial.print(" - ");
  Serial.print( d5, 10 );
  Serial.print(" - ");
  Serial.println(b5, 10);
  
  Serial.print("Distance6: ");
  Serial.print(t6);
  Serial.print(" - ");
  Serial.print( d6, 10 );
  Serial.print(" - ");
  Serial.println(b6, 10);
  
  Serial.print("Distance7: ");
  Serial.print(t7);
  Serial.print(" - ");
  Serial.print( d7, 10 );
  Serial.print(" - ");
  Serial.println(b7, 10);
  
  Serial.print("Distance8: ");
  Serial.print(t8);
  Serial.print(" - ");
  Serial.print( d8, 10 );
  Serial.print(" - ");
  Serial.println(b8, 10);
  
  Serial.print("Distance9: ");
  Serial.print(t9);
  Serial.print(" - ");
  Serial.print( d9, 10 );
  Serial.print(" - ");
  Serial.println(b9, 10);
  
  Serial.print("Bearing: ");
  Serial.println(getBearing(holdLat, holdLon, currLat, currLon), 10);
  Serial.print("Bearing2: ");
  Serial.println(getBearing2(holdLat, holdLon, currLat, currLon, d1), 10);
  delay(10000);
}


float getDistance1(float lat1, float lon1, float lat2, float lon2)
{  
  float start = micros();
  float x = sq(sin((lat1 - lat2) / 2.0f));
  float y = sq(sin((lon1 - lon2) / 2.0f));
  float dist = 2.0f * asin(sqrt(x + cos(lat1) * cos(lat2) * y)); 
  t1 = micros() - start;
  b1 = (RAD2DEG * atan2(x,y));
  return dist * RAD2CM;
}

float getDistance2(float lat1, float lon1, float lat2, float lon2){
  float start = micros();
  float distanceX = (float)(lon1 - lon2);
  float distanceY = (float)(lat1 - lat2);
  float distance  = sqrt(sq(distanceY) + sq(distanceX));
  t2 = micros() - start;
  b2 = (RAD2DEG * atan2(distanceX,distanceY));
  return   distance  * RAD2CM;
}

float getDistance7(float lat1, float lon1, float lat2, float lon2){
  float start = micros();
  float distanceX = (float)(lon1 - lon2);
  float distanceY = (float)(lat1 - lat2);
  float distance  = sqrt(sq(distanceY) + sq(distanceX));
  t7 = micros() - start;
  b7 = RAD2DEG * (atan2(distanceX,distanceY)-trueNorthHeading);
  if(b7 < 0){
    b7 += 360;
  }
  return   distance;
}

float getDistance6(float lat1, float lon1, float lat2, float lon2, float cosLat) 
{  
  float start = micros();
  float distanceX = (float)(lon2 - lon1) * cosLat * 1.113195;
  float distanceY = (float)(lat2 - lat1) * 1.113195;
  float distance =  sqrt(sq(distanceY) + sq(distanceX));  
  t6 = micros() - start;
  b6 = (RAD2DEG * atan2(distanceX,distanceY));
  return distance;  
}



float getDistance8(float lat1, float lon1, float lat2, float lon2, float cosLat){
  float start = micros();
  float distanceX = (float)(lon2 - lon1) * cosLat * 1.113195;
  float distanceY = (float)(lat2 - lat1)* 1.113195;
  float xSqrd = distanceX * distanceX;
  float ySqrd = distanceY * distanceY;
  float distance  = sqrt(ySqrd + xSqrd);
  t8 = micros() - start;
  b8 = (RAD2DEG * atan2(distanceX,distanceY));
  return   distance  * RAD2CM;
}


/*----------------------------------------------------------------------**
**     Haversine Formula                                                **
** (from R. W. Sinnott, "Virtues of the Haversine," Sky and Telescope,  **
** vol. 68, no. 2, 1984, p. 159):                                       **
**                                                                      **
**   dLon = lon2 - lon1                                                 **
**   dLat = lat2 - lat1                                                 **
**   a = (sin(dlat/2))^2 + cos(lat1) * cos(lat2) * (sin(dlon/2))^2      **
**   c = 2 * atan2(sqrt(a), sqrt(1-a))                                  **
**   d = R * c                                                          **
**                                                                      **
** will give mathematically and computationally exact results. The      **
** intermediate result c is the great circle distance in radians. The   **
** great circle distance d will be in the same units as R.              **
**----------------------------------------------------------------------*/

float getDistance3(float CurrentLatitude, float CurrentLongitude, float SavedLatitude, float SavedLongitude)
{
  float start = micros();
// HaverSine version
    //const float Deg2Rad = 0.01745329252;               // (PI/180)  0.017453293, 0.0174532925
    const float EarthRadius = 637279760;              //6372.7976 In Kilo meters, will scale to other values
    //const float EarthRadius = 20908120.1;              // In feet  20908128.6
    float DeltaLatitude, DeltaLongitude, a, Distance;

    // degrees to radians
//    CurrentLatitude = (CurrentLatitude + 180) * Deg2Rad;     // Remove negative offset (0-360), convert to RADS
//    CurrentLongitude = (CurrentLongitude + 180) * Deg2Rad;
//    SavedLatitude = (SavedLatitude + 180) * Deg2Rad;
//    SavedLongitude = (SavedLongitude + 180) * Deg2Rad;

    DeltaLatitude = (float)(SavedLatitude - CurrentLatitude);
    DeltaLongitude = (float)(SavedLongitude - CurrentLongitude);

    a =(sin(DeltaLatitude/2.0f) * sin(DeltaLatitude/2.0f)) + cos(CurrentLatitude) * cos(SavedLatitude) * (sin(DeltaLongitude/2.0f) * sin(DeltaLongitude/2.0f));
    Distance = EarthRadius * (2.0f * atan2(sqrt(a),sqrt(1-a)));
    t3 = micros() - start;
    b3 = (RAD2DEG * atan2(DeltaLongitude,DeltaLatitude));
    return(Distance);
}

// Following code from <http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1259946578/6>
// "It's pretty much cut'n'paste from the Ardupilot project" which is LGPL
// here: <http://code.google.com/p/ardupilot/>
//
//
/*************************************************************************
 * //Function to calculate the distance between two waypoints
 *************************************************************************/
float getDistance4(float flat1, float flon1, float flat2, float flon2) {
  float start = micros();
  float dist_calc=0;
  float dist_calc2=0;
  float diflat=0;
  float diflon=0;

  //I've to spplit all the calculation in several steps. If i try to do it in a single line the arduino will explode.
    diflat=flat2-flat1;
  flat1=flat1;
  flat2=flat2;
  diflon=flon2-flon1;

  dist_calc = (sin(diflat/2.0f)*sin(diflat/2.0f));
  dist_calc2= cos(flat1);
  dist_calc2*=cos(flat2);
  dist_calc2*=sin(diflon/2.0f);
  dist_calc2*=sin(diflon/2.0f);
  dist_calc +=dist_calc2;

  dist_calc=(2*atan2(sqrt(dist_calc),sqrt(1.0-dist_calc)));

  dist_calc*=637100000.0; //Converting to cm
  t4 = micros() - start;
  b4 = (RAD2DEG * atan2(diflon,diflat));
  return dist_calc;
}

// by David Cary
// public domain
// WARNING: untested code
double haversine( double angle ){
    return( pow( sin(angle/2), 2 ) );
};
double archaversine( double x ){
    return( 2*asin( min(1.0, sqrt(x)) ) );
};
double approx_haversine( double angle ){
    return( angle*angle*(1.0/(2*2)) );
};
double approx_archaversine( double x ){
    return( 2*sqrt(x) );
};
// classic mean radius -- see Wikipedia: Earth radius.
static const double r_earth = 637100.0e3; // in meters
/*
Calculate the distance between two *very* nearby points on Earth.
The points are (latitude1, longitude1) and (latitude2, longitude2) in degrees.
Returns a distance in meters.
*/
double getDistance5( double nLat1, double nLon1, double nLat2, double nLon2 ){
  float start = micros();
    //assert( fabs(nLat1 - nLat2) < 1 );
    //assert( fabs(nLon1 - nLon2) < 1 );
    static double average_latitude = 0;
    static double cos2_avl = 1;
 
    nLat1 = radians(nLat1);
    nLat2 = radians(nLat2);
    const double nDLon = radians( nLon2-nLon1 );
    const double nDLat = radians( nLat2-nLat1);

    // 1 km, in radians
    const double significant_distance = 1000 / r_earth;
    if( abs( average_latitude - nLat2 ) > significant_distance ){
        average_latitude = nLat2;
        double t = cos( average_latitude );
        cos2_avl = t*t;
    }; 
    double nA = approx_haversine(nLat1-nLat2) + cos2_avl * approx_haversine(nDLon);
    double nC = approx_archaversine(nA);
    double nD = r_earth * nC;
    t5 = micros() - start;
    b5 = (RAD2DEG * atan2(nDLon,nDLat));
    return nD; // Return our calculated distance
}

float getDistance6() 
{  
  float start = micros();
  float distanceX = (float)(currLon - holdLon) * cosLatitude * 1.113195;
  float distanceY = (float)(currLat - holdLat) * 1.113195;
  t6 = micros() - start;
  b6 = (RAD2DEG * atan2(distanceX,distanceY));
  return  sqrt(sq(distanceY) + sq(distanceX));  
}

float getDistance9(float lat1, float lon1, float lat2, float lon2) 
{
  float start = micros();
  const float x = (float)(lon2 - lon1) * GPS2RAD * cosLatitude;
  const float y = (float)(lat2 - lat1) * GPS2RAD;
  t9 = micros() - start;
  b9 = (RAD2DEG * atan2(x,y));
  return sqrt(x*x+y*y);
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
  float start = micros();
  float x = sin(toLonRads - fromLonRads) * cos(toLatRads);
  float y = cos(fromLatRads) * sin(toLatRads) - sin(fromLatRads) * cos(toLatRads) * cos(toLonRads - fromLonRads);
  float z = atan2(x, y);

  float bearing = ModCrs(z);
  Serial.println(micros() - start);
  return bearing;
}

float getBearing2(float lat1, float lon1, float lat2, float lon2, float d) {
  float start = micros();
  float bearing = 0.0;
  if(sin(lon2-lon1)<0)       
   bearing = acos((sin(lat2)-sin(lat1)*cos(d))/(sin(d)*cos(lat1))); 
else       
   bearing = TWOPI-acos((sin(lat2)-sin(lat1)*cos(d))/(sin(d)*cos(lat1)));  
   
   Serial.println(micros() - start);
   return bearing;
 
}

