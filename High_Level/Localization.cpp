#include "Globals.h"
#include "Common.h"
#include "Localization.h"
#include "Can_Protocol.h"
#define gpsSerial    Serial3

Adafruit_GPS GPS(&gpsSerial);

//Adafruit_L3GD20 gyro; GYRO still not working
//#ifdef USE_I2C
// The default constructor uses I2C
//Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(1337);
//#else
// To use SPI, you have to define the pins
/* These pin numbers look wrong. Need the pins on the Due, not the ones on the device. TCF 8/6/20 */
// #define GYRO_CS 4 // labeled CS
// #define GYRO_DO 5 // labeled SA0
// #define GYRO_DI 6  // labeled SDA
// #define GYRO_CLK 7 // labeled SCL
// Adafruit_L3GD20 gyro(GYRO_CS, GYRO_DO, GYRO_DI, GYRO_CLK);
//#endif

namespace elcano {
/******************************************************************************************************
 * constructor for Location
 *  activates the GPS, Compass, Gyroscope and finds initial positition
 *****************************************************************************************************/
Location::Location()
{
   //setting up the GPS rate
   setup_GPS();

   //Enable auto-gain
   mag.enableAutoRange(true);
   accel.enableAutoRange(true);

   if (!accel.begin())
   {
      /* There was a problem detecting the ADXL345 ... check your connections */
#ifdef DEBUG
      Serial.println("Ooops, no accelerometer detected ... Check your wiring!");
#endif
   }
   if (!mag.begin())
   {
#ifdef DEBUG
      Serial.println("no magnetometer detected ... Check your wiring!");
#endif
   }

   initial_position(); //getting the initial position from GPS
   //set starting speed to 0
   newPos.speed_mmPs = 0;

   /*
    * // Try to initialise and warn if we couldn't detect the chip
    * if (!gyro.begin(GYRO_RANGE_250DPS))
    * //if (!gyro.begin(gyro.L3DS20_RANGE_500DPS))
    * //if (!gyro.begin(gyro.L3DS20_RANGE_2000DPS))
    * {
    #ifdef DEBUG
    * Serial.println("Oops ... unable to initialize the gyro. Check your wiring!");
    #endif
    * }
    */
}

/********************************************************************************************************
 * setup_GPS()
 * initializes the GPSRATE chooses option of GGA
 *******************************************************************************************************/
void Location::setup_GPS()
{
   //Serial 3 (gpsSerial) is used for GPS
   gpsSerial.begin(GPSRATE);
   // Serial3.begin(GPSRATE);
   if (!GPS.begin(GPSRATE))
   {
#ifdef DEBUG
      Serial.println("Ooops, no GPS detected ... Check your wiring!");
#endif
   }

   // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
   //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

   // uncomment this line to turn on only the "minimum recommended" data
   GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
   // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
   // the parser doesn't care about other sentences at this time

   // Set the update rate
   GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);  // 5 Hz update rate
   // For the parsing code to work nicely and have time to sort thru the data, and
   // print it out we don't suggest using anything higher than 1 Hz

   // Request updates on antenna status, comment out to keep quiet
   GPS.sendCommand(PGCMD_ANTENNA);
}

/********************************************************************************************************
 * AcquireGPS()
 * Searches for a GPS signal.
 * param: gps_position to record gps position if found
 * return: true if gps is found
 *******************************************************************************************************/
bool Location::AcquireGPS()
{
#ifdef DEBUG
   Serial.println("Acquire GPS");
#endif
   char c;
   //read atleast 25 characters every loop speed up update time for GPS
   // GPS.read() does not appear to do anything.  TCF 8/17/20
   for (int i = 0; i < 25; i++)
   {
      c = GPS.read();
   }

   // if a sentence is received, we can check the checksum, parse it...
   if (GPS.newNMEAreceived())
   {
      //  Serial.println("newNMEArecieved");
      // a tricky thing here is if we print the NMEA sentence, or data
      // we end up not listening and catching other sentences!
      // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
      //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

      if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      {
         return(false);               // we can fail to parse a sentence in which case we should just wait for another
      }
      if (GPS.fix)
      {
         GPS_reading.latitude  = GPS.latitudeDegrees;
         GPS_reading.longitude = GPS.longitudeDegrees;
#ifdef DEBUG
         Serial.println("Latitude: " + String(GPS_reading.latitude, 6));
         Serial.println("Longitude: " + String(GPS_reading.longitude, 6));
#endif
         return(true);
      }
      return(false);
   }
   return(false);
}

/*******************************************************************************************************
 * Local_communication_with_LowLevel()
 * Receive actual speed and actual turn angle from the C2_Lowlevel
 * board via CAN BUS message to help calculate where the trike's estimated position in
 ******************************************************************************************************/
void Location::Read_CAN_bus()
{
   CAN.watchForRange(Actual_CANID, LowStatus_CANID); //filter for low level communication

   while (CAN.available() > 0)                       // check if CAN message available
   {
      CAN.read(incoming);

#ifdef DEBUG2
      Serial.println("Get data from (low level) ID: " + String(incoming.id, HEX));
      Serial.print("Low: " + String((int)incoming.data.low, DEC));
      Serial.println(", High: " + String((int)incoming.data.high, DEC));
#endif

      if (incoming.id == Actual_CANID)
      {
         actualSpeed = incoming.data.low;
         if (actualSpeed >= 0)
         {
            newPos.speed_mmPs = actualSpeed;     //upadte acutal speed from DBW
         }

#ifdef DEBUG
         else
         {
            Serial.println("Got a negative speed from DBW");
         }
         else
         {
            Serial.println("Did not receive actual speed, angle from DBW");
         }
#endif
      } // incoming == actual
   }    // while
}

/*******************************************************************************************************
 * getHeading()
 * checks the compass to get the heading in degrees
 ******************************************************************************************************/
long Location::getHeading()
{
   //Get a new sensor event from the magnetometer
   sensors_event_t event;

   mag.getEvent(&event);

   //Calculate the current heading (angle of the vector y,x)
   //Normalize the heading
   float heading = (atan2(event.magnetic.y, event.magnetic.x) * 180) / PIf;

#ifdef DEBUG
   Serial.println("mag y = " + String(event.magnetic.y));
   Serial.println("mag x = " + String(event.magnetic.x));
   Serial.println("mag z = " + String(event.magnetic.z));

   Serial.println("raw y = " + String(mag.raw.y));
   Serial.println("raw x = " + String(mag.raw.x));
   Serial.println("raw z = " + String(mag.raw.z));

   Serial.println("acquired heading: " + String(heading));
#endif
   if (heading < 0)
   {
      heading = 360 + heading;
   }
#ifdef DEBUG3
   Serial.println("Heading is: " + String(heading));
#endif
   return(heading);
}

//

/*******************************************************************************************************
 * getGyroRoll()
 * uses gyroscope to get Gyroscope roll
 ******************************************************************************************************
 * double Location::getGyroRoll(){
 *  sensors_event_t event;
 *  gyro.getEvent(&event);
 #ifdef DEBUG
 *  Serial.println("Gyroscope Roll X: " + String(event.gyro.x));
 #endif
 * return event.gyro.x;
 * }
 */
/*******************************************************************************************************
 * getGyroPitch()
 * uses gyroscope to get GyroPitch
 **************************************************************************************************
 * double Location::getGyroPitch(){
 *  sensors_event_t event;
 *  gyro.getEvent(&event);
 #ifdef DEBUG
 * Serial.println("Gyroscope Pitch Y: " + String(event.gyro.y));
 #endif
 * return event.gyro.y;
 */
/*******************************************************************************************************
 * getGyroYaw()
 * uses gyroscope to get GyroYaw
 ******************************************************************************************************
 * double Location::getGyroYaw(){
 * sensors_event_t event;
 * gyro.getEvent(&event);
 #ifdef DEBUG
 * Serial.println("Gyroscope Yaw Z: " + String(event.gyro.z));
 #endif
 * return event.gyro.z;
 * }
 */
/*******************************************************************************************************
 * getAccelX()
 * uses acceleromenter to get Acceleration in X axis
 ******************************************************************************************************/
double Location::getAccelX()
{
   //Get a new sensor event from the magnetometer
   sensors_event_t event;

   accel.getEvent(&event);

   double accelerationX = event.acceleration.x;

   /* Display the results (acceleration is measured in m/s^2) */
#ifdef DEBUG3
   Serial.println("Acceleration X: " + String(event.acceleration.x) + "  m/s^2 ");
#endif
   return(accelerationX);
}

/*******************************************************************************************************
 * getAccelY()
 * uses acceleromenter to get Acceleration in Y axis
 ******************************************************************************************************/
double Location::getAccelY()
{
   //Get a new sensor event from the magnetometer
   sensors_event_t event;

   accel.getEvent(&event);

   double accelerationY = event.acceleration.y;

   /* Display the results (acceleration is measured in m/s^2) */
#ifdef DEBUG3
   Serial.println("Acceleration Y: " + String(event.acceleration.y) + "  m/s^2 ");
#endif
   return(accelerationY);
}

/*******************************************************************************************************
 * getAccelZ()
 * uses acceleromenter to get Acceleration in Z axis
 ******************************************************************************************************/
double Location::getAccelZ()
{
   //Get a new sensor event from the magnetometer
   sensors_event_t event;

   accel.getEvent(&event);

   double accelerationZ = event.acceleration.z;

   /* Display the results (acceleration is measured in m/s^2) */
#ifdef DEBUG3
   Serial.println("Acceleration Z: " + String(event.acceleration.z) + "  m/s^2 ");
#endif
   return(accelerationZ);
}

/*******************************************************************************************************
 * findPosition()
 * On entry, estimated position is from gps.
 * Combine dead reckoning and GPS to calculate a new estimated position'
 * sets the estimPos vectors values to the newly calculated position
 * Future:  use accelerometer with the Kalman filter for better results
 ******************************************************************************************************/
void Location::findPosition(Waypoint&estimPos, Waypoint&oldPos)
{
   newPos.time_ms = millis(); //mark current time

   //get heading coordinates from the compass
   newPos.bearing_deg   = getHeading();
   estimPos.bearing_deg = newPos.bearing_deg; //used to find first path in Pilot.cpp
#ifdef DEBUG
   Serial.println("loop6 newPos.bearing_deg = " + String(newPos.bearing_deg));
#endif
   //for use in Kalman filter - may want to move this somewhere else
   double accelX = getAccelX();
   double accelY = getAccelY();
   double accelZ = getAccelZ();
//  double gyroRoll = getGyroRoll();
//  double gyroPitch = getGyroPitch();
//  double gyroYaw = getGyroYaw();
//
   if (got_GPS)
   {
#ifdef DEBUG
      Serial.println("got gps and deadreckoning");
#endif
      //to get an esitimation position average between the GPS and Dead Rekoning
      //estimPos is updated to the current position inside this method

      FindFuzzyCrossPointXY(estimPos, newPos, estimPos);

      //calculating the E and N vector by constantly updating everything you move
      oldPos.vectors(&estimPos);
   }
   else
   {
#ifdef DEBUG
      Serial.println("Only got dead reckoning");
#endif
      // calculate position using Dead Reckoning
      ComputePositionWithDR(oldPos, newPos);

      //calculating the E and N unit vector
      oldPos.vectors(&newPos);

      //update new current positon
      estimPos.east_mm  = newPos.east_mm;
      estimPos.north_mm = newPos.north_mm;
   }

   //update E and N vector of current position
   estimPos.Evector_x1000 = oldPos.Evector_x1000;
   estimPos.Nvector_x1000 = oldPos.Nvector_x1000;
#ifdef DEBUG3
   Serial.println("--- North: " + String(estimPos.north_mm) + ", East: " + String(estimPos.east_mm) + "\n");
#endif
   //update old position to current
   oldPos = estimPos;
}

/*******************************************************************************************************
 * initial_position()
 * Get your first initial position from the GPS
 * On entry, new_pos and GPS_reading are unknown
 ******************************************************************************************************/
void Location::initial_position()
{
   bool          GPS_available   = false;
   unsigned long elapsed_time_ms = 0;

   newPos.time_ms = millis();

   // Try to get GPS; time-out after a minute.
   while (!GPS_available && elapsed_time_ms < 60000)
   {
      GPS_available   = AcquireGPS();
      elapsed_time_ms = millis() - newPos.time_ms;
   }
   if (GPS_available)
   {
#ifdef DEBUG
      Serial.println("Acquired GPS position");
#endif
      newPos.time_ms = millis();
   }
   else   // No GPS
   // With no GPS a default GPS_reading is (47.760850, -122.190044); //center of UW soccer field
   {
      GPS_reading.latitude  = ORIGIN_LAT;
      GPS_reading.longitude = ORIGIN_LONG;
      GPS_reading.cos_lat   = cos(GPS_reading.latitude * TO_RADIANS);
   }
   newPos.latitude  = GPS_reading.latitude;
   newPos.longitude = GPS_reading.longitude;
   newPos.Compute_mm(GPS_reading);
//    newPos.Compute_EandN_Vectors(getHeading()); //get position E and N vector

#ifdef DEBUG
   Serial.println("Computed Vectors in initial position");
#endif
}

/******************************************************************************************************
 * update loop for Location
 *  gets a new GPS signal, Deadreckoning data from the Lowlevel
 *  and uses this data to estimate the current position of the trike
 *****************************************************************************************************/
void Location::update(Position&ogn, Waypoint&estimated_position, Waypoint&oldPos)
{
   oldPos.time_ms = millis();
   delay(1);
   got_GPS = AcquireGPS(); //try to get a new GPS position as GPS_reading
   int test = 0;

   while (got_GPS == false && test < 5)
   {
      got_GPS = AcquireGPS();
      test++;
   }

   Read_CAN_bus(); // Receiving speed data from DBW using CAN Bus

   if (got_GPS)
   {
      estimated_position.latitude  = GPS_reading.latitude;
      estimated_position.longitude = GPS_reading.longitude;
      estimated_position.Compute_mm(ogn); // get north and east coordinates from originStl
#ifdef DEBUG
      Serial.println("Got and computed GPS");
#endif
   }
   else
   {
#ifdef DEBUG
      Serial.println("Failed to get got_GPS");
#endif
   }
   findPosition(estimated_position, oldPos); //determine location based on dead reckoning and GPS
}
} // namespace elcano
