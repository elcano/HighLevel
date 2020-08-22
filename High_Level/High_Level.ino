//Elcano Libraries
#include "Globals.h"
#include "Common.h"
#include "Pilot.h"
#include "Localization.h"
//Outside libraries
#ifdef USING_ARDUINO
#include <Adafruit_L3GD20.h>
#endif
using namespace elcano;
Location *      vehiclePosition;
Pilot *         myPilot;

Position   MapOrigin(ORIGIN_LAT, ORIGIN_LONG); //origin on the selected map
Waypoint estimated_position(MapOrigin), old_position(MapOrigin);

/******************************************************************************************************
 * main setup method
 *****************************************************************************************************/
void setup()
{
   Serial.begin(9600);          // for the serial monitor

   if (CAN.begin(CAN_BPS_500K)) // initalize CAN with 500kbps baud rate
   {
      Serial.println("CAN init success");
   }

/* Set up the vehicle location and acquire GPS
 *   The Location constuctor calls setupGPS() and initialPosition().
 *   initialPosition() calls acquireGPS, which sets up GPS_reading.
 *   If GPS cannot be acquired in a minute, a default position is used.
 *   The Waypoint newPos is partly initialized.
 */
   vehiclePosition = new Location();

   estimated_position.CreateFrom(vehiclePosition->GetPos());
   old_position.CreateFrom(vehiclePosition->GetPos());

 #ifdef DEBUG
   Serial.println("estimated_position = " + String(myLocal->estimated_position.latitude));
   Serial.println("Starting Pilot");
#endif

   /* Pilot is a mash-up of several objects and should be refactored.
    *  The Pilot constructor starts off by creating a Planner class.
    *  The Planner constuctor calls initializeMap().
    *  initializeMap uses the present vehicle position to find the most
    *  appropriate map and loads it.
    *  It then reads a list of destinations.
    *  Using the map, current location and destination'
    *  Planner calls A* to compute the route.
    *
    * At this point, the map is no longer needed, and the planner is done.
    * It can be retained in case the vehicle gets off-course and
    * needs to compute a new route.
    *
    * This is the point that the Pilot should begin, with input of the route,
    * vehicle position, and destination.
    *
    * Once the vehicle starts to move, it become possible to determine its
    * direction vectors and bearing.
    *
    * TCF 8/15/20
    */
   myPilot = new Pilot(vehiclePosition->GetPos());

#ifdef DEBUG
   Serial.print("Map origin after planner is now set to: ");
   Serial.print(MapOrigin.latitude, 6);
   Serial.print(" ");
   Serial.println(MapOrigin.longitude, 6);
#endif
}

/******************************************************************************************************
 * main loop method
 *****************************************************************************************************/
void loop()
{
   vehiclePosition->update(MapOrigin, estimated_position, old_position);
#ifdef DEBUG
   Serial.println("estimated_position.eastmm = " + String(estimated_position.east_mm));
#endif
   //RE-compute path if too far off track (future development)

   myPilot->update(estimated_position, old_position);

   //path has been set to first mission cone, once reach it use planner in pilot
   //update myPlanner->mission_index++
   //update myPlanner->Start to current Location
   //make a new plan
   //in pilot call myPlanner->last_index_of_path = myPlanner->PlanPath(origin(fix to origin name), myPlanner->Start, myPlanner->mission[mission_index]);
   //go to next mission index until myPlanner->mission_index > CONES
   //reached end so stop
}
