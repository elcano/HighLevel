#include "Globals.h"
#ifndef SIMULATION
#include <Arduino.h>
#endif
#include "Common.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

//comment here for test

//Globals from Common.cpp
//global variables
char          buffer[BUFFSIZ]; // string buffer for the sentence
char          dataString[BUFFSIZ];
volatile bool DataAvailable;

char        GPSfile[BUFFSIZ] = "mmddhhmm.csv";
char        ObstacleString[BUFFSIZ];
char        StartTime[BUFFSIZ] = "yy,mm,dd,hh,mm,ss,xxx";
const char  TimeHeader[]       = "year,month,day,hour,minute,second,msec";
const char *RawKF      = "Raw GPS data,,,,,,Kalman Filtered data";
const char *Header     = "Latitude,Longitude,East_m,North_m,SigmaE_m,SigmaN_m,Time_s,";
const char *ObstHeader = "Left,Front,Right,Busy";

namespace common {
/*---------------------------------------------------------------------------------------*/
bool checksum(char *msg)
{
   int sum = 0;

   if (msg[0] != '$')
   {
      return(false);
   }
   for (int i = 1; i < BUFFSIZ - 4; i++)
   {
      if (msg[i] == '*')
      {
         int msb = (sum >> 4);
         msg[i + 1] = msb > 9 ? 'A' + msb - 10 : '0' + msb;
         int lsb = (sum & 0x0F);
         msg[i + 2] = lsb > 9 ? 'A' + lsb - 10 : '0' + lsb;
         //      msg[i+3] = '\n';  // rely on Serial.println()
         return(true);
      }
      sum ^= msg[i];
   }
   return(false);
}
}
namespace elcano {
// Default constructor for type Position object
Position::Position()
{
   Position(ORIGIN_LAT, ORIGIN_LONG);
}

//Constructor for type Position object
Position::Position(double lat, double log)
{
   latitude  = lat;
   longitude = log;

   cos_lat = cos((latitude) * TO_RADIANS);              // needed to correct for the size of a degree of longitude.
}

/*---------------------------------------------------------------------------------------*/
//---------------------------------------------------------
long int parsedecimal(char *str)
{
   long int d = 0;

   while (str[0] != 0)
   {
      if ((str[0] > '9') || (str[0] < '0'))
      {
         return(d);
      }
      d *= 10;
      d += str[0] - '0';
      str++;
   }
   return(d);
}

//---------------------------------------------------------
//  read a number of form  123.456,
// On entry, str points to the 1 or -
// On return, it points to the next charater beyond the comma.
// whole number can have any number of digits.
// fraction must be three digits.
// If there is no decimal point, returned value is an integer.
// If there is a decimal point, returned value is scaled.

long int ReadDecimal(char *str)
{
   long whole;
   bool negative = false;

   if (str[0] == ',')
   {
      str++;                     // a blank field means no data
      return(INVALID);
   }
   if (str[0] == '-')
   {
      negative = true;
      str++;
   }
   whole = parsedecimal(str);
   if (str[0] == '.')
   {
      str++;                    // skip decimal point
      whole *= 1000;
      whole += parsedecimal(str);
   }
   if (str[0] == ',')
   {
      str++;
   }
   return(negative ? -whole : whole);
}

void DataReady()          // called from an interrupt
{
   DataAvailable = true;
}

//---------------------------------------------------------
/* Function Determines crosspoint on X-axis */
//currently only the X coordinate of the intersection between all points
double CrossPointX(double x1, double y1, double x2, double y2,
                   double x3, double y3, double x4, double y4)
{
   if (x1 == x2 && x3 == x4)
   {
      return((x1 + x2) / 2.0);
   }
   if (x1 == x2)
   {
      return(x1);
   }
   if (x3 == x4)
   {
      return(3);
   }

   /* yc = (y4-y3)/(x4-x3)*(xc-x3) + y3 = (y2-y1)/(x2-x1)*(xc-x1)+y1 */
   double slope1 = (y2 - y1) / (double)(x2 - x1);
   double slope3 = (y4 - y3) / (double)(x4 - x3);

   if (slope3 == slope1)
   {
      return((x2 + x2 + x3) / 4.0);
   }

   double xc = (slope3 * x3 - y3 - slope1 * x1 + y1) / (double)(slope3 - slope1);

   return(xc);
}

/* The function calculates position using Dead Reckoning
 */
void ComputePositionWithDR(Waypoint&oldData, Waypoint&newData)
{
   //Serial.println("millis(): " + String(millis()));
   //Serial.print("ComputePositionWithDR::NewTime:");
   //DEBUGGING				//Serial.print(newData.time_ms);
   //Serial.print(",OldTime:");
   //Serial.println(oldData.time_ms);


   // To check if this is new reading or the same reading
   if (newData.time_ms > oldData.time_ms)
   {
      // Calculate distance using time-difference and speed of newData waypoint
      double distance_mm = (newData.time_ms - oldData.time_ms) * (newData.speed_mmPs);

      //Serial.print("ComputePositionWithDR::newSpeed_mmPs:");
      //DEBUGGING				//Serial.print(newData.speed_mmPs);
      //Serial.print(",distance:");
      //Serial.println(newData.distance_mm);

// Latitude and longitude cannot be used for distance computations since the distance represented
// by one degree is not the same for them.
      //uses cosine to add horizontal distance to east
      newData.east_mm = oldData.east_mm + distance_mm * cos((newData.bearing_deg) * TO_RADIANS);
      // a negative east_mm means west

      //uses sine to add vertical distance to north
      newData.north_mm = oldData.north_mm + distance_mm * sin((newData.bearing_deg) * TO_RADIANS);
      // a negative north_mm means south.

      //Serial.print("ComputePositionWithDR::X_pos:");
      //Serial.print(newData.x_Pos);
      //DEBUGGING				//Serial.print(",Y_pos:");
      //Serial.println(newData.y_Pos);
   }
}

/* This function finds fuzzy crosspoint between GPS & Dead reckoning data
 * Waypoint &gps -> GPS input
 * Waypoint &dr   -> Dead reckoning input
 * Waypoint &estimated_position -> update estimated_position
 */
void FindFuzzyCrossPointXY(Waypoint&gps, Waypoint&dr, Waypoint&estimated_position)
{
   //if GPS and DR position is the same
   if (gps.east_mm == dr.east_mm && gps.north_mm == dr.north_mm)
   {
      estimated_position.north_mm = dr.north_mm;
      estimated_position.east_mm  = dr.east_mm;
      return;
   }

   //if the change between reckoning east and GPS-east is greater than the change between reckoning north and GPS-north
   if (abs(gps.east_mm - dr.east_mm) > abs(gps.north_mm - dr.north_mm))
   {
      //DEBUGGING									//Serial.println("more east");

      //if the gps-east is further east than reckoning-east
      if (gps.east_mm >= dr.east_mm)
      {
         // cross point is intersection of line down from DR and line up to GPS
         // DR down line is from (dr->east_mm, 1) to (dr->east_mm + DR_ERROR_mm, 0)
         // line up to GPS is from (gps->east_mm - gps->sigma_mm, 0) to (gps->east_mm, 1)
         estimated_position.east_mm = CrossPointX(dr.east_mm, 1, dr.east_mm + DR_ERROR_mm, 0,
                                                  gps.east_mm - gps.sigma_mm, 0., gps.east_mm, 1);
      }

      //otherwise the reckoning-east is further east than the GPS
      else
      {
         // cross point is intersection of line down from GPS and line up to DR
         estimated_position.east_mm = CrossPointX(dr.east_mm - DR_ERROR_mm, 0., dr.east_mm, 1,
                                                  gps.east_mm, 1, gps.east_mm + gps.sigma_mm, 0.);
      }

      // north position is proportional to east position												WHY IS IT PROPORTIONAL??
      if (gps.north_mm >= dr.north_mm)
      {
         //WHERE DO THESE EQUATIONS COME FROM?
         estimated_position.north_mm = dr.north_mm + (gps.north_mm - dr.north_mm) *
                                       abs((gps.east_mm - estimated_position.east_mm) / (double)(gps.east_mm - dr.east_mm));
      }
      else
      {
         estimated_position.north_mm = gps.north_mm + (dr.north_mm - gps.north_mm) *
                                       abs((gps.east_mm - estimated_position.east_mm) / (double)(gps.east_mm - dr.east_mm));
      }
   }

   //then the difference between north's is greater than the differences between south's
   else
   {
      // TO DO: similar to above, but swap east and north
      //Serial.println("more north");
      if (gps.north_mm >= dr.north_mm)
      {
         // cross point is intersection of line down from DR and line up to GPS
         // DR down line is from (1, dr->north_mm) to (0, dr->north_mm + DR_ERROR_mm)
         // line up to GPS is from (0, gps->north_mm - gps->sigma_mm) to (1, gps->north_mm)
         estimated_position.north_mm = CrossPointX(dr.north_mm, 1, dr.north_mm + DR_ERROR_mm, 0,
                                                   gps.north_mm - gps.sigma_mm, 0., gps.north_mm, 1);
      }

      //then the reckoning-north is bigger than GPS-north
      else
      {                         // cross point is intersection of line down from GPS and line up to DR
         estimated_position.north_mm = CrossPointX(dr.north_mm - DR_ERROR_mm, 0., dr.north_mm, 1,
                                                   gps.north_mm, 1, gps.north_mm + gps.sigma_mm, 0.);
      }


      // east position is proportional to north position
      if (gps.east_mm >= dr.east_mm)
      {                                                                                                                                 //WHERE DO THESE EQUATIONS COME FROM
         estimated_position.east_mm = dr.east_mm + (gps.east_mm - dr.east_mm) *
                                      abs((gps.north_mm - estimated_position.north_mm) / (double)(gps.north_mm - dr.north_mm));
      }
      else
      {
         estimated_position.east_mm = gps.east_mm + (dr.east_mm - gps.east_mm) *
                                      abs((gps.north_mm - estimated_position.north_mm) / (double)(gps.north_mm - dr.north_mm));
      }
   }
}

void Position::operator=(Position& right)
{
   latitude  = right.latitude;
   longitude = right.longitude;
   cos_lat   = right.cos_lat;
}
Waypoint::Waypoint()  // Constructor
{
    Waypoint(MapOrigin);
}
Waypoint::Waypoint(Position Lat_Lon)  
{
    CreateFrom( Lat_Lon); 
}

void elcano::Waypoint::CreateFrom(elcano::Position Lat_Lon)
{
   latitude    = Lat_Lon.latitude;
   longitude   = Lat_Lon.longitude;
   Compute_mm(Lat_Lon);
   bearing_deg = 0;
   Compute_EandN_Vectors(0);
   speed_mmPs  = 0;
   time_ms = millis();
}

void Waypoint::operator=(Waypoint& right)
{
   latitude      = right.latitude;
   longitude     = right.longitude;
   east_mm       = right.east_mm;
   north_mm      = right.north_mm;
   bearing_deg   = right.bearing_deg;
   Evector_x1000 = right.Evector_x1000;
   Nvector_x1000 = right.Nvector_x1000;
   sigma_mm      = right.sigma_mm;
   speed_mmPs    = right.speed_mmPs;
   time_ms       = right.time_ms;
   index         = right.index;
}
void Waypoint::operator=(Waypoint* right)
{
    latitude      = right->latitude;
    longitude     = right->longitude;
    east_mm       = right->east_mm;
    north_mm      = right->north_mm;
    bearing_deg   = right->bearing_deg;
    Evector_x1000 = right->Evector_x1000;
    Nvector_x1000 = right->Nvector_x1000;
    sigma_mm      = right->sigma_mm;
    speed_mmPs    = right->speed_mmPs;
    sigma_mm      = DEFAULT_GPS_ERROR_MM;
    time_ms       = right->time_ms;
    index         = right->index;
}

void Waypoint::SetTime(char *pTime, char *pDate)
{
   //GPSfile = "mmddhhmm.CSV";
   strncpy(GPSfile, pDate + 2, 2);               // month
   strncpy(GPSfile + 2, pDate, 2);               // day
   strncpy(GPSfile + 4, pTime, 2);               // GMT hour
   strncpy(GPSfile + 6, pTime + 2, 2);           // minute
   Serial.println(GPSfile);

   strncpy(StartTime, pDate + 4, 2);                 // year
   strncpy(StartTime + 3, pDate + 2, 2);             // month
   strncpy(StartTime + 6, pDate, 2);                 // day
   strncpy(StartTime + 9, pTime, 2);                 // GMT hour
   strncpy(StartTime + 12, pTime + 2, 2);            // minute
   strncpy(StartTime + 15, pTime + 4, 2);            // second
   strncpy(StartTime + 18, pTime + 7, 3);            // millisecond
}

/* There are more accurate ways to compute distance between two latitude and longitude points.
 * We use a simple approximation, since we are interesed in a flat projection over a small area.
 * Curvature of the earth is not significant.
 */
/*----------------------------------------------------------------------------------------
*  Given a latitude and longitude, compute its coordinates on the current map as
*  (east_mm, north_mm).
*  This position depends on the latitude and longitude of the map origin.
*  Thus the argument to the function is the origin of the current map.
*
*  double and float on Arduino Uno, Mega and Micro are both 32 bits, with only
*  24 bits for the mantissa. Thus an Arduino doouble is not capable of enough resolution.
*  The 7th digit after the decimal point in latitude or longitude corresponds to about 1 cm.
*  To get 1 cm resolution in longitude would require 3 digits for the whole number and 7
*  digits after the decimal point or 10^-10.
*  But 24 binary digits correspond to 6 * 10^-8 which is not precise resolution.
*  Double on Arduino Due is 64 bits, so it would have adequate resolution
*  Thus distances are kept as 32-bit integers in mm.
*---------------------------------------------------------------------------------------*/

void Waypoint::Compute_mm(Position&origin)
{
   // compute relative to origin, since Arduino double is limited to 6 digits.
   double diffWhole;
   double dist;

/*
 * One degree of latitude is 111 km anywhere on the globe.
 */
   diffWhole = latitude - origin.latitude;               // In degrees
   dist      = diffWhole * TO_RADIANS * EARTH_RADIUS_MM; // In mm as double
   north_mm  = dist;                                     // In mm as 32-bit int

/*
 * One degree of longitude is 111 km at the equator and zero at the poles.
 * The distance represented by a degree of longitude depends on the latitude.
 */
   diffWhole = (longitude - origin.longitude);                   // In degrees
   dist      = diffWhole * TO_RADIANS * EARTH_RADIUS_MM;         // In mm as double
   east_mm   = (dist * origin.cos_lat);                          // account for variation in longitude distance
}

void Waypoint::Compute_LatLon(Position&origin)
{
   double theta;

   theta    = asin((double)north_mm / EARTH_RADIUS_MM);
   latitude = origin.latitude + (theta / TO_RADIANS);

   theta     = asin((double)(east_mm / origin.cos_lat) / EARTH_RADIUS_MM);
   longitude = origin.longitude + (theta / TO_RADIANS);
}

/*
 * Estimated state (index=-1)  or Waypoint, or list of next Waypoints on route (index > 0);									WHAT IS THIS
 *        $POINT,<east_m>,<north_m>,<sigma_m>,<time_s>,<speed_mPs>,<Evector_x1000>,<Nvector_x1000>,<index>*CKSUM
 *        // at leat 18 characters
 */

char *Waypoint::formPointString()
// function uses a global dataString buffer; thus a second call to formDataString
// may overwrite results from the previous call.
{
   // now log the information
   // make a string for assembling the data to log:
   long eastFraction, northFraction, speedFraction;

   eastFraction  = east_mm >= 0 ? east_mm % 1000 : (-east_mm) % 1000;
   northFraction = north_mm >= 0 ? north_mm % 1000 : (-north_mm) % 1000;
   speedFraction = speed_mmPs >= 0 ? speed_mmPs % 1000 : (-speed_mmPs) % 1000;
   sprintf(dataString,
           "$POINT,%ld.%.3ld,%ld.%.3ld,%ld.%.3ld,%ld.%.3ld,%ld.%.3ld,%ld,%ld,%d*\r\n",
           east_mm / 1000, eastFraction, north_mm / 1000, northFraction,
           sigma_mm / 1000, sigma_mm % 1000, time_ms / 1000, time_ms % 1000,
           speed_mmPs / 1000, speed_mmPs % 1000, Evector_x1000, Nvector_x1000, index);

   return(dataString);
}

/*
 * bool Waypoint::readPointString(unsigned long max_wait_ms, int channel) {
 *      char *parsePtr;
 *      unsigned long end_time = millis() + max_wait_ms;
 *      while (millis() < end_time) {
 *              if (readline(channel) && strncmp(buffer, "$POINT", 6) == 0) {
 *                      //  $POINT,<east_m>,<north_m>,<sigma_m>,<time_s>,<speed_mPs>,<Evector_x1000>,<Nvector_x1000>,<index>*CKSUM
 *                      parsePtr = buffer + 7;
 *                      east_mm = ReadDecimal(parsePtr);
 *                      north_mm = ReadDecimal(parsePtr);
 *                      Serial.println("East_mm " + String(east_mm) + " \t  North_mm" + String(north_mm));
 *                      sigma_mm = ReadDecimal(parsePtr);
 *                      time_ms = ReadDecimal(parsePtr);
 *                      speed_mmPs = ReadDecimal(parsePtr);
 *                      Evector_x1000 = ReadDecimal(parsePtr);
 *                      Nvector_x1000 = ReadDecimal(parsePtr);
 *                      index = ReadDecimal(parsePtr);
 *                      east_mm += 100;
 *                      north_mm += 100;
 *                      return true;
 *              }
 *      }
 *      return false;
 * }
 */
long Waypoint::distance_mm(Waypoint *other)
{
   long deltaX, deltaY;

   deltaX = east_mm - other->east_mm;
   deltaY = north_mm - other->north_mm;
   return(sqrt(deltaX * deltaX + deltaY * deltaY));
}

void Waypoint::vectors(Waypoint *other)
{
   long deltaX, deltaY, dist;

   deltaX        = -east_mm + other->east_mm;
   deltaY        = -north_mm + other->north_mm;
   dist          = sqrt(deltaX * deltaX + deltaY * deltaY);
   Evector_x1000 = (deltaX * 1000.) / dist;
   Nvector_x1000 = (deltaY * 1000.) / dist;
}

long Waypoint::distance_mm(long East_mm, long North_mm)
{
   long deltaX, deltaY;

   deltaX = East_mm - east_mm;
   deltaY = North_mm - north_mm;
   return(sqrt(deltaX * deltaX + deltaY * deltaY));
}

//========================= Items  for C6 Navigator =================================

#define REAL    float
// unsigned long millis() is time since program started running
// offset_ms is value of millis() at start_time
unsigned long offset_ms = 0;

void Filter(REAL *x, REAL *P, REAL *measure, REAL deltaT, REAL *variance)
{
}                                                                                    // needs to be implemented

/*---------------------------------------------------------------------------------------*/
//Method need futher implementation. Currently not used
void Waypoint::fuse(Waypoint GPS_reading, int deltaT_ms, Position&origin)
{
   // Assume uncertainty standard deviation is 10 meters.
   // The numbers below are variances in m.
   // speed standard deviation is in m/sec;
   // assuming no time error it is same as position standard deviation
   static REAL uncertainty[] = { 100.,    0,    0, 0,
                                 0,    100.,    0, 0,
                                 0,       0, 100., 0,
                                 0,       0,    0, 100. };
   static REAL State[4]   = { 5000000, 0, 0, 0 };
   REAL        variance[] = { 100., 0,
                              0, 100. };

   REAL deltaT_s = ((REAL)deltaT_ms) / 1000.0;
   REAL measurements[2];
   REAL speedX, speedY;

   if (State[0] > 2500000)                 // first time
   {
      State[0] = GPS_reading.east_mm / 1000.;
      State[1] = GPS_reading.north_mm / 1000.;
   }
   speedX          = ((REAL)(speed_mmPs) * Evector_x1000) / MEG;       // m/sec
   speedY          = ((REAL)(speed_mmPs) * Nvector_x1000) / MEG;
   measurements[0] = GPS_reading.east_mm / 1000. + speedX * deltaT_s;
   measurements[1] = GPS_reading.north_mm / 1000. + speedY * deltaT_s;

   REAL GPS_sigma = ((REAL)GPS_reading.sigma_mm) / 1000.;

   variance[0] = variance[3] = GPS_sigma * GPS_sigma;

   Filter(State, uncertainty, measurements, deltaT_s, variance);

   east_mm  = State[0] * 1000;
   north_mm = State[1] * 1000;
   speedX   = State[2] * 1000;
   speedY   = State[3] * 1000;
   sigma_mm = sqrt(uncertainty[0]) * 1000;

   Compute_LatLon(origin);
   speed_mmPs = 1000 * sqrt(speedX * speedX + speedY * speedY);
   if (speed_mmPs > 100 || speed_mmPs < -100)
   {
      Evector_x1000 = (MEG * speedX / speed_mmPs);
      Nvector_x1000 = (MEG * speedY / speed_mmPs);
   }
}

// Calculate and compute the E and N vector
void Waypoint::Compute_EandN_Vectors(long heading_deg)
{
   Evector_x1000 = cos((heading_deg) * TO_RADIANS) * 1000;
   Nvector_x1000 = sin((heading_deg) * TO_RADIANS) * 1000;
}

//----------------------------------------------------------
char *Waypoint::GetLatLon(char *parseptr)
{
   long int latitd, longitd;
   char     latdir, longdir;
   int      latDegree, longDegree;
   float    latFraction, longFraction;

   latitd = parsedecimal(parseptr);
   if (latitd != 0)
   {
      latitd  *= 10000;
      parseptr = strchr(parseptr, '.') + 1;
      latitd  += parsedecimal(parseptr);
   }
   parseptr = strchr(parseptr, ',') + 1;
   // read latitude N/S data
   if (parseptr[0] != ',')
   {
      latdir = parseptr[0];
   }
   // longitude
   parseptr = strchr(parseptr, ',') + 1;
   longitd  = parsedecimal(parseptr);
   if (longitd != 0)
   {
      longitd *= 10000;
      parseptr = strchr(parseptr, '.') + 1;
      longitd += parsedecimal(parseptr);
   }
   parseptr = strchr(parseptr, ',') + 1;
   // read longitude E/W data
   if (parseptr[0] != ',')
   {
      longdir = parseptr[0];
   }
   // latitude in latDegree = dd, latMinutes = .mmmmmm
   latDegree   = latitd / 1000000;
   latFraction = ((latitd % 1000000) / 10000) / 60;
   latitude    = latDegree + latFraction;
   if (latdir == 'S')
   {
      latitude = -latitude;
   }
   //longitude in longDegree = ddd, latMinutes = .mmmmmm
   longDegree   = longitd / 1000000;
   longFraction = ((longitd % 1000000) / 10000) / 60;
   longitude    = longDegree + longFraction;
   if (longdir == 'W')
   {
      longitude = -longitude;
   }

   return(parseptr);
}

double Position::distance_points_mm(double lat, double lon)
{
   //Haversine distance formula
   float R       = 6373; //earth radius in kilometers
   float latRad1 = PIf * lat / 180.0;
   float latRad2 = PIf * latitude / 180.0;
   float lonRad1 = PIf * lon / 180.0;
   float lonRad2 = PIf * longitude / 180.0;
   float dlat    = latRad2 - latRad1;
   float dlon    = lonRad2 - lonRad1;
   float comp    = asin(sqrt(sin(dlat / 2) * sin(dlat / 2) + cos(latRad1) * cos(latRad2) * sin(dlon / 2) * sin(dlon / 2)));

   return(2 * R * comp * 1000000);//in mm
}
}// namespace elcano
