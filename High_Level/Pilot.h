#pragma once
#include "Common.h"
#include "Planner.h"
#ifdef USING_ARDUINO
#include <due_can.h>
#endif

//using namespace elcano;
namespace elcano {
class Pilot {
private:
#define DESIRED_SPEED2    1200
   enum States { STARTING, STOP, STRAIGHT, ENTER_TURN, LEAVE_TURN, APPROACH_GOAL, LEAVE_GOAL };
   States state;
   Planner *myPlanner;
   int next = 1; //index to pathz in a list

   long speed_mmPs;
   long turn_direction;
   long pre_desired_speed;
   long pre_turn_angle;
#ifdef USING_ARDUINO
   CAN_FRAME CANoutput; //CAN frame to carry message to DBW
#endif

   /*
    * The path is the output of reading the map and applying A*
    * The Pilot has no need of the  map object. It just needs the path.
    * Waypoint pathz[MAX_WAYPOINTS];  // course route to goal/mission
    * The vehicle is meant to follow a path from Waypint to Waypoint.
    *
    *
    * As it gets to one waypoint and transitions to the next, it goes thru some of the states below.
    * String wrdState[7] = { "STARTING", "STOP", "STRAIGHT", "ENTER_TURN",
    * "LEAVE_TURN", "APPROACH_GOAL", "LEAVE_GOAL"}; //used for clearer debugging
    */
   //last is the the last index of the Path/goal
   int last_index_of_pathz = 2; //hardcode path of the last index/dest to 3 [cur,loc1,goal]
   int q      = 0;              //for testing to vary speed in C3 find_state
   bool first = true;
   //******************** hard coded alternating speeds for testing ******************
   int speeds[6]     = { 2000, 2500, 1500, 2500, 1000, 2000 };
   int angg[6]       = { 100, 125, 100, 120, 110, 122 };
   int speedIndex    = 0;
   int insaneCounter = 0;
   //*********************************************************************************

   long turning_radius_mm(long speed_mmPs);
   bool test_past_destination(int n, Waypoint&estPos);
   bool test_approach_intersection(long turn_radius_mm, int n, Waypoint&estPos);
   bool test_leave_intersection(long turning_radius_mm, int n, Waypoint&estPos);
   int get_turn_direction_angle(int n, Waypoint&estPos);
   void find_state(long turn_radius_mm, int n, Waypoint& estimated_pos);
   void hardCoded_Pilot_Test();
   void Pilot_communicate_LowLevel();
   void initializePosition(Waypoint&estPos, Waypoint&oldPos);
   void populate_path();
   bool proper_heading(Waypoint&estimated_pos, int n);

public:
   Pilot(Position vehiclePosition);
   ~Pilot()
   {
   }               //destructor

   void update(Waypoint&estimated_pos, Waypoint&old_pos);
};
} // namespace elcano
