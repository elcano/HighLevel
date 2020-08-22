
#pragma once
#ifdef USING_ARDUINO
#include <due_can.h>
#include <Adafruit_GPS.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20.h>
#endif
#include <math.h>
namespace elcano {
class Location {
private:
   // Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to DEBUG and listen to the raw GPS sentences.
#define GPSECHO    false
#define GPSRATE    9600

   long actualSpeed = 0; // Speed will come from Drive-by-wire.

   bool got_GPS = false;
#ifdef USING_ARDUINO
   /* Assign a unique ID to this sensor at the same time */
   Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);

   /* Assign a unique ID to this sensor at the same time */
   Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);

   CAN_FRAME incoming;
#endif

   void setup_GPS();       // called once by constuctor
   bool AcquireGPS();
   void Read_CAN_bus();
   long getHeading();
   double getAccelX();
   double getAccelY();
   double getAccelZ();
   double getGyroRoll();
   double getGyroPitch();
   double getGyroYaw();
   void findPosition(Waypoint&estimPos, Waypoint&op);
   void initial_position();      // called once by constuctor
   // Defaullt position set to center of UWB soccer field
   elcano::Position GPS_reading;
   elcano::Waypoint newPos;

public:
   Location();
   ~Location()
   {
   }                  //destructor
   Position GetPos()
   {
	   return GPS_reading;
   }
   Waypoint GetWaypoint()
   {
	   return newPos;
   }
   void update(Position&ogn, Waypoint&ep, Waypoint&newPos);
};
} // namespace elcano
