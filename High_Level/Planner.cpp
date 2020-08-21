#include "Globals.h"
#include "Common.h"
#ifndef SIMULATION
#include <Arduino.h>
#include <SD.h>
#endif
#include "Planner.h"
#include <math.h>


namespace elcano {
Planner::Planner(Position&vehiclePosition) //default constructor
{
   map_points         = 0;               //will be filled in during map loading
   mission_index      = 0;               //index number of mission/cone currently attempting to reach
   last_index_of_path = 0;
   last_index_of_path = CONES - 1;       //hardcode path of the last index/dest to 3 [cur,loc1,goal]
   //Store the initial GPS latitude and longtitude to select the correct map

   initialize_Map(vehiclePosition);  //Start selecting/load map and start planning path

 #ifdef DEBUG
   Serial.println("Start planning path");
 #endif
   Waypoint start(vehiclePosition);

   last_index_of_path = PlanPath(start, mission[mission_index]);  //start is current location - destination is first cone
}

/*---------------------------------------------------------------------------------------*/
// Fill the distances of the Junctions in the MAP //Uses mapNodes array
void Planner::FindDistances(Junction *Map)
{
   const int ShiftA[17] = { 12, 9, 7, 6, 5, 5, 4, 4, 4, 3, 3, 3, 2, 2, 2, 2, 2 };
   const int ShiftB[17] = { 12, 9, 9, 8, 12, 6, 7, 5, 4, 5, 4, 4, 9, 5, 4, 3, 3 };

   long int deltaX, deltaY;
   int      destination;

   for (int i = 0; i < map_points; i++)
   {
      if (Map[i].east_mm == INVALID)
      {
         continue;
      }
      for (int j = 0; j < 4; j++)
      {
         destination = Map[i].destination[j];
         if (destination == END)
         {
            continue;
         }
         deltaX = Map[i].east_mm - Map[destination].east_mm;
         if (deltaX < 0)
         {
            deltaX = -deltaX;                                      // absolute value
         }
         deltaY = Map[i].north_mm - Map[destination].north_mm;
         if (deltaY < 0)
         {
            deltaY = -deltaY;                                      // absolute value
         }
         if (deltaX < deltaY)
         {                        // make deltaX the  bigger
            long int temp = deltaX;
            deltaX = deltaY;
            deltaY = temp;
         }
         // fast computation by not using double or sqrt
         Map[i].Distance[j] = deltaX;                         // Base distance is the maxmum change in X or Y
         if (deltaY <= 0)
         {
            continue;                                         // Nothing more to do if no change in 2nd coordinate
         }
         int ratio = (deltaY << 4) / deltaX;                  // ratio is 0 for linear to 16 for square
         Map[i].Distance[j] += deltaX >> ShiftA[ratio];
         Map[i].Distance[j] += deltaX >> ShiftB[ratio];       // Appoximation to Euclidean distance.
      }
   }
}

/*---------------------------------------------------------------------------------------*/
// Set up mission structure from goal_lat and goal_lat arrays (the goal cones to hit)
void Planner::GetGoals(Position&ori, Junction *mapNodes)
{
   /* #ifdef DEBUG
    *      Serial.print("Printing origin latitide ");
    *  Serial.print(ori.latitude, 6);
    *  Serial.print(" and longitude ");
    *  Serial.println(ori.longitude, 6);
    #endif
    */
   double deltaX, deltaY, Distance;

   for (int i = 0; i < CONES; i++)
   {
      mission[i].latitude  = goal_lat[i];
      mission[i].longitude = goal_lon[i];
      mission[i].Compute_mm(ori);
      mission[i].speed_mmPs = DESIRED_SPEED_mmPs;                   //defined in Settings_HighLevel.h
      mission[i].index      = 1 | GOAL;
      mission[i].sigma_mm   = 1000;
      mission[i].time_ms    = 0;

      /*   #ifdef DEBUG
       *                    Serial.print("Printing mission[" + String(i) + "] latitide ");
       *        Serial.print(mission[i].latitude, 6);
       *        Serial.print(" and longitude ");
       *        Serial.println(mission[i].longitude, 6);
       *        Serial.println("Printing mission[" + String(i) + "] east_mm " + String(mission[i].east_mm) + " " + "north_mm " + String(mission[i].north_mm));
       #endif
       */
      if (i == 0)                     //If CONE == 1
      // Do not have a direstion for the starting position, so set it
      // to an arbitrary value.
      {
         mission[i].Evector_x1000 = 1000;
         mission[i].Nvector_x1000 = 0;
      }
      else
      {
         deltaX   = mission[i].east_mm - mission[i - 1].east_mm;
         deltaY   = mission[i].north_mm - mission[i - 1].north_mm;
         Distance = sqrt(deltaX * deltaX + deltaY * deltaY);
         mission[i - 1].Evector_x1000 = (deltaX * 1000.) / Distance;
         mission[i - 1].Nvector_x1000 = (deltaY * 1000.) / Distance;
      }
      if (i == CONES - 1)
      {
         mission[i].Evector_x1000 = mission[i - 1].Evector_x1000;
         mission[i].Nvector_x1000 = mission[i - 1].Nvector_x1000;
         mission[i].index        |= END;
      }
   }
}

/*---------------------------------------------------------------------------------------*/

// Find the distance from (east_mm, north_mm) to a road segment mapNodes[i].distance[j]
// return distance in mm, and per cent of completion from i to j.
// distance is negative if (east_mm, north_mm) lies to the left of the road
// when road direction is from i to j

// Compare this routine to distance() in C3 Pilot
//k =  index into mapNodes[]
//east_mm : current
long Planner::distance(int&cur_node, int&k, long&cur_east_mm, long&cur_north_mm, int&perCent)
{
   // Original intent was to used scaled long integer instead of double. TCF 8/10/20
   // Long integer can be significantly faster on a microprocessor.
   // These computations can be done in integer arithmetic instead of doouble.
   double deltaX, deltaY;
   int    cur, destination;
   long   closest_mm = MAX_DISTANCE;
   double Road_distance;
   long   RoadDX_mm, RoadDY_mm;

   long pc;              //per cent of completion from i to j.

   perCent = 0;
   k       = 0;

   for (cur = 0; cur < 4; cur++)                // Don't make computations twice.
   {
      destination = mapNodes[cur_node].destination[cur];
      //if 0 or less already checked or is a dead end so don't check
      if (destination == 0 || destination < cur_node)
      {
         continue;                                                                 //replace Destination with END
      }
      // compute road unit vectors from i to cur
      RoadDX_mm = mapNodes[destination].east_mm - mapNodes[cur_node].east_mm;
      int Eunit_x1000 = RoadDX_mm * 1000 / mapNodes[cur_node].Distance[cur];
      RoadDY_mm = mapNodes[destination].north_mm - mapNodes[cur_node].north_mm;
      int Nunit_x1000 = RoadDY_mm * 1000 / mapNodes[cur_node].Distance[cur];
      //      // normal vector is (Nunit, -Eunit)
      //      //Answers: What would be the change in X/Y from my current Node.
      deltaX = cur_east_mm - mapNodes[cur_node].east_mm;
      deltaY = cur_north_mm - mapNodes[cur_node].north_mm;
      //      // sign of return value gives which side of road it is on.
      //      Road_distance = (-deltaY * Eunit_x1000 + deltaX * Nunit_x1000) / 1000;  // more computationally efficient
      Road_distance = sqrt((RoadDX_mm * RoadDX_mm) + (RoadDY_mm * RoadDY_mm));                   // less computationally efficient
      //Why do percentage computation like this?
      pc = (deltaX * Eunit_x1000 + deltaY * Nunit_x1000) / (mapNodes[cur_node].Distance[cur] * 10);
#ifdef DEBUG
      Serial.println("Destination " + String(destination));
      Serial.println("RoadDX_mm " + String(RoadDX_mm));
      Serial.println("mapNodes[destination].east_mm " + String(mapNodes[destination].east_mm));
      Serial.println("-mapNodes[cur_loc].east_mm " + String(-mapNodes[cur_node].east_mm));
      Serial.println("Eunit_x1000: " + String(Eunit_x1000));
      Serial.println("RoadDY_mm " + String(RoadDY_mm));
      Serial.println("mapNodes[destination].north_mm " + String(mapNodes[destination].north_mm));
      Serial.println("-mapNodes[cur_loc].north_mm " + String(-mapNodes[cur_node].north_mm));
      Serial.println("Nunit_x1000: " + String(Nunit_x1000));
      Serial.println("DX x DX: " + String(RoadDX_mm * RoadDX_mm));
      Serial.println("DY x DY: " + String(RoadDY_mm * RoadDY_mm));
      Serial.println("Added together: " + String((RoadDX_mm * RoadDX_mm) + (RoadDY_mm * RoadDY_mm)));
      Serial.println("Closest_mm " + String(closest_mm) + "\t Road_distance " + String(Road_distance));
#endif
      if (abs(Road_distance) < abs(closest_mm) && pc >= 0 && pc <= 100)
      {
         closest_mm = Road_distance;
         k          = destination;
         perCent    = pc;
      }
   }
    #ifdef DEBUG
   Serial.println("In distance method returning closest_mm: " + String(closest_mm));
   Serial.println(" ");
#endif
   return(long(closest_mm));
}

/*---------------------------------------------------------------------------------------*/
//Figuring out a path to get the road network
void Planner::FindClosestRoad(Waypoint&start, Waypoint&road)              //populate road with best road from start
{
   long closest_mm = MAX_DISTANCE;
   long dist;
   int  close_index;
   int  perCent;
   long done = 1;             //2000;
   int  i, node_successor;

   for (i = 0; i < 5 /*map_points*/; i++)                                         // find closest road.
   {
      dist = distance(i, node_successor, start.east_mm, start.north_mm, perCent); //next node to visit
#ifdef DEBUG
      Serial.println("Start : Latitude " + String(start.latitude) + "\t Longitude " + String(start.longitude)
                     + "\t Dist " + String(dist));
#endif

      if (abs(dist) < abs(closest_mm))
      {
         close_index   = node_successor;
         closest_mm    = dist;
         done          = 1;              // perCent; //Not really true amount of mapNodes done?
         road.index    = i;
         road.sigma_mm = node_successor;
      }
   }
   if (closest_mm < MAX_DISTANCE)
   {
      i = road.index;                   //0
      node_successor = close_index;     //0
      road.east_mm   = mapNodes[i].east_mm + done * (mapNodes[node_successor].east_mm - mapNodes[i].east_mm) / 100;
      road.north_mm  = mapNodes[i].north_mm + done * (mapNodes[node_successor].north_mm - mapNodes[i].north_mm) / 100;
   }
   else
   {
      for (i = 0; i < 5 /*map_points*/; i++)                    // find closest node
      //Serial.println("I got here");
      {
         dist = start.distance_mm(mapNodes[i].east_mm, mapNodes[i].north_mm);

         if (dist < closest_mm)
         {
            close_index = i;
            closest_mm  = dist;
         }
      }
      road.index    = road.sigma_mm = close_index;
      road.east_mm  = mapNodes[close_index].east_mm;
      road.north_mm = mapNodes[close_index].north_mm;
   }

   road.Evector_x1000 = 1000;
   road.Nvector_x1000 = 0;
   road.time_ms       = 0;
   road.speed_mmPs    = DESIRED_SPEED_mmPs;

   //Test FindClosest Road:
#ifdef DEBUG
   Serial.println("Distance " + String(dist));
   Serial.println("Road :  East_mm " + String(road.east_mm) + "\t North_mm " + String(road.north_mm));
#endif
}

/*---------------------------------------------------------------------------------------*/
// start and destination are on the road network given in mapNodes.
// start is in Path[1].
// Place other Junction Waypoints into Path.
// Returned value is next index into Path.
// start->index identifies the closest node.
// sigma_mm holds the index to the other node.
// A* is traditionally done with pushing and popping node from an Open and Closed list.
// Since we have a small number of mapNodes, we instead reserve a slot on Open and Closed
// for each node.

int Planner::BuildPath(long&j, Waypoint&start, Waypoint&destination)              // Construct path backward to start.
{
#ifdef DEBUG
   Serial.println("To break");
#endif
   int last = 1;              //already did 0 in planPath
   int route[MAX_Waypoints];
   int k, node;

   k        = map_points - 1;
   route[k] = j;

   while (Open[j].ParentID != currentlocation)
   {
      j = route[--k] = Open[j].ParentID;
   }

   path[last] = start;
   for (; k < map_points; k++)
   {
      node = route[k];
      path[++last].east_mm = mapNodes[node].east_mm;
      path[last].north_mm  = mapNodes[node].north_mm;
   }
   path[++last] = destination;
   for (k = 0; k <= last; k++)
   {
      if (k > 0)
      {
         path[k].sigma_mm = 10;                           // map should be good to a cm.
      }
      path[k].index      = k;
      path[k].speed_mmPs = DESIRED_SPEED_mmPs;
   }
   last++;
   for (j = 0; j < last - 1; j++)
   {
      path[j].vectors(&path[j + 1]);
   }

   return(last);             //The next index to use in planPath
}

/*---------------------------------------------------------------------------------------*/
//Use A-star
int Planner::FindPath(Waypoint&start, Waypoint&destination)              //While OpenSet is not empty
{
#ifdef DEBUG
   Serial.println("Start East_mm " + String(start.east_mm) + "\t North " + String(start.north_mm));
   Serial.println("Start East_mm " + String(destination.east_mm) + "\t North " + String(destination.north_mm));
#endif
   long ClosedCost[MAX_Waypoints];
   int  i, neighbor;
   long NewCost, NewStartCost, NewCostToGoal;
   long NewIndex;
   long BestCost = MAX_DISTANCE, BestID;

   for (i = 0; i < MAX_Waypoints; i++)                // mark all mapNodes as empty
   {
      Open[i].TotalCost = MAX_DISTANCE;
      ClosedCost[i]     = MAX_DISTANCE;
   }

   i = start.index;              // get successor mapNodes of start
   Open[i].CostFromStart = start.distance_mm(mapNodes[i].east_mm, mapNodes[i].north_mm);
   Open[i].CostToGoal    = destination.distance_mm(mapNodes[i].east_mm, mapNodes[i].north_mm);

   Open[i].TotalCost = Open[i].CostFromStart + Open[i].CostToGoal;
   Open[i].ParentID  = currentlocation;

   i = start.sigma_mm;
   Open[i].CostFromStart = start.distance_mm(mapNodes[i].east_mm, mapNodes[i].north_mm);
   Open[i].CostToGoal    = destination.distance_mm(mapNodes[i].east_mm, mapNodes[i].north_mm);
   Open[i].TotalCost     = Open[i].CostFromStart + Open[i].CostToGoal;

   Open[i].ParentID = currentlocation;
   while (BestCost < MAX_DISTANCE)                //While OpenSet is not empty
   {
      BestCost = MAX_DISTANCE;
      BestID   = -1;
      // pop lowest cost node from Open; i.e. find index of lowest cost item
      for (i = 0; i < 6; i++)
      {
         if (Open[i].TotalCost < BestCost)
         {
            BestID = i;
            //            Serial.println("BESTID " + String(BestID));
            BestCost = Open[i].TotalCost;
            //            Serial.println("BestCost " + String(BestCost));
         }
      }
      if (BestID < 0)
      {
         return(INVALID);
      }
#ifdef DEBUG
      Serial.println("BESTID " + String(BestID));
      Serial.println("Destination Index " + String(destination.index));
#endif
      Open[BestID].TotalCost = MAX_DISTANCE;                             // Remove node from "stack".
      if (BestID == destination.index || BestID == destination.sigma_mm) // Done:: reached the goal!!

      {
         return(BuildPath(BestID, start, destination));                          // Construct path backward to start.
      }

      i = BestID;                    // get successor mapNodes from map

      for (neighbor = 0; neighbor < 4; neighbor++)
      {
         NewIndex = mapNodes[i].destination[neighbor];

         if (NewIndex == END)
         {
            continue;                                         // No success in this slot
         }
         NewStartCost  = Open[i].CostFromStart + mapNodes[i].Distance[neighbor];
         NewCostToGoal = destination.distance_mm(mapNodes[NewIndex].east_mm, mapNodes[NewIndex].north_mm);
         NewCost       = NewStartCost + NewCostToGoal;

         if (NewCost >= ClosedCost[NewIndex])           // check if this node is already on Open or Closed.
         {
            continue;                                   // Have already looked at this node
         }
         else if (ClosedCost[NewIndex] != MAX_DISTANCE) // looked at this node before, but at a higher cost
         {
            ClosedCost[NewIndex] = MAX_DISTANCE;        // remove node from Closed
         }
         if (NewCost >= Open[NewIndex].TotalCost)
         {
            continue;                               // This node is a less efficient way of getting to a node on the list
         }
         // Push successor node onto stack.

         Open[NewIndex].CostFromStart = NewStartCost;
         Open[NewIndex].CostToGoal    = NewCostToGoal;
         Open[NewIndex].TotalCost     = NewCost;
         Open[NewIndex].ParentID      = i;
      }                    // end of successor mapNodes


      ClosedCost[BestID] = BestCost;                   // Push node onto Closed
   }

#ifdef DEBUG
   Serial.println("Destination East_mm " + String(destination.east_mm) + "\t North "
                  + String(destination.north_mm));
#endif
   return(0);              // failure
}

/*---------------------------------------------------------------------------------------*/
// Low level path is a straight line from start to detination.
// PathPlan makes an intermediate level path that uses as many roads as possible.
//start = currentlocation: destination = heading to;
//Find the cloeset road and call Findpath to do the A star
int Planner::PlanPath(Waypoint&start, Waypoint&destination)
{
   //Serial.println("Start : East_mm = " + String(start->east_mm) + "\t North_mm =  " + String(start->north_mm));
   Waypoint roadOrigin, roadDestination;

   int last = 0;

   path[0]       = start;
   path[0].index = 0;

   FindClosestRoad(start, roadOrigin);
   FindClosestRoad(destination, roadDestination);

   /*
    #ifdef DEBUG
    *  Serial.println("In plan path - Selected road origin is: " + String(roadorigin.latitude));
    * Serial.println("In plan path - Selected road destination is: " + String(roadDestination.latitude));
    * Serial.println(" ");
    #endif
    */
   int w = abs(start.east_mm - roadOrigin.east_mm) + abs(start.north_mm - roadOrigin.north_mm);
   int x = abs(destination.east_mm - roadDestination.east_mm) + abs(destination.north_mm - roadDestination.north_mm);

   int straight_dist = 190 * abs(start.east_mm - destination.east_mm) + abs(start.north_mm - destination.north_mm);

   if (w + x >= straight_dist)                // don't use roads; go direct
   {
      last = 1;
#ifdef DEBUG
      Serial.println("In Straight");
#endif
   }
   else                 // use A* with the road network
   {
#ifdef DEBUG
      Serial.println("In Else");
#endif
      path[1]       = roadOrigin;
      path[1].index = 1;
      //why index = 7?
      destination.index = 7;
      last = FindPath(roadOrigin, roadDestination);
   }

   path[last] = destination;
   path[last - 1].vectors(&path[last]);
   path[last].Evector_x1000 = path[last - 1].Evector_x1000;
   path[last].Nvector_x1000 = path[last - 1].Nvector_x1000;
   path[last].index         = last | END;

#ifdef DEBUG
   Serial.println("Destination : East_mm = " + String(destination.east_mm) + "\t North_mm =  " + String(destination.north_mm));
   Serial.println(" ");
#endif
   return(last);
}
} // namespace elcano
