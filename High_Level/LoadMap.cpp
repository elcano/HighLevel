#include "Globals.h"
#include "Common.h"
#ifndef SIMULATION
#include <Arduino.h>
#include <SD.h>
#else
#include <stdio.h>
#endif
#include "Planner.h"

namespace elcano {
/*---------------------------------------------------------------------------------------*/
void Planner::initialize_Map(Position&vehPos)
{
   //chipSelect = 35 located in Global.h if change needed
//      pinMode(chipSelect_SD, OUTPUT);
#ifdef DEBUG
   Serial.println("Initializing SD card...");
   Serial.println("chipSelect = " + String(chipSelect_SD));

   if (!SD.begin(chipSelect_SD))
   {
      Serial.println("initialization failed!");
   }
   else
   {
      Serial.println("initialization done.");
#endif
   //13 to include null terminate - SD cannot have more than 12 char & \0 in filename caused error
   char nearestMap[13] = "";
   Position MapOrigin(vehPos.latitude,vehPos.longitude);
   //Finds the map whose origin is closest to vehicle, and returns name in nearestMap;
   SelectMap(MapOrigin, vehPos, "MAP_DEFS.TXT", nearestMap);

#ifdef DEBUG
   Serial.print("after map selection origin is now set to : ");
   Serial.print(orign.latitude, 6);
   Serial.print(", ");
   Serial.println(orign.longitude, 6);
   Serial.print("current estimated_pos is: ");
   Serial.print(vehiclePosition.latitude, 6);
   Serial.print(", ");
   Serial.println(vehiclePositions.longitude, 6);
#endif
   //   estimPos.Compute_mm(orign);  //initialize north and east coordinates for position
#ifdef DEBUG
   Serial.print("Estimate E: ");
   Serial.println(estimPos.east_mm);
   Serial.print("Estimate N: ");
   Serial.println(estimPos.north_mm);
   Serial.println("Map selected was: " + String(nearestMap));
#endif
   //populate nearest map in Junction mapNodes structure
   LoadMap(nearestMap);

   //takes in the mapNodes that contains all of the map
   FindDistances(mapNodes);  //To fill out the rest of the mapNodes info

   GetGoals(MapOrigin, mapNodes);

   /*
    * After loading the map, the next action should be to use A* to find the path
    * The desired path should be found in startup.
    * The pilot should follow the path in the loop.
    * The map is a separate object from the pilot.
    */
}

/*---------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------*/
// SelectMap:  Return the file name of the closest origin
// Determines which map to load.
// Takes in the current location as a Waypoint and a string with the name of the file that
//   contains the origins and file names of the maps.
// Determines which origin is closest to the Waypoint and returns it as a Junction.
// Assumes the file is in the correct format according to the description above.
void Planner::SelectMap(Position&MapOrgin, Position&vehiclePosition, const char *fileName, char *nearestMap)
{
   // open the file. note that only one file can be open at a time,
   // so you have to close this one before opening another.
   File myFile = SD.open(fileName, FILE_READ);

   // if the file opened okay, read from it:
   if (myFile)
   {
      // Initialize a string buffer to read lines from the file into
      // Allocate an extra char at the end to add null terminator
      char *buffer = (char *)malloc(myFile.size() + 1);

      //Serial.println("Printing file info");
      // index for the next character to read into buffer
      char *ptr = buffer;

      // read from the file until there's nothing else in it:
      while (myFile.available())
      {
         *ptr = myFile.read();
         ++ptr;
      }

      // Null terminate the buffer string
      *ptr = '\0';

      //MAX_MAPS is set to 10 in Common.h
      // Set up storage for coordinates and file names
      // Note: we malloc here because the stack is too small
      float *map_latitudes  = (float *)malloc(MAX_MAPS * 4);
      float *map_longitudes = (float *)malloc(MAX_MAPS * 4);
      char **map_file_names = (char **)malloc(MAX_MAPS * sizeof(char *));

      for (int i = 0; i < MAX_MAPS; i++)
      {
         // initialize using invalid values so that we can ensure valid data in allocated memory
         map_latitudes[i]  = 91;
         map_longitudes[i] = 181;
         map_file_names[i] = 0;
      }
#ifdef DEBUG
      Serial.println("");
      Serial.println("Loaded " + String(fileName) + " now loading and printing maps array");
#endif
      // Set up tokenizer for the file buffer string
      const char *delimiter = " ,\n";
      char *      token;
      int         col = 0;
      int         row = 0;
      //    char* t;

      // get the first token
      token = strtok(buffer, delimiter);

      //The first token has junk at the front and won't atof() need to use below work around
      //for first number before switch
      //need to make into a string, manipulate and turn back to null terminated char array.
      String temps = String(token);
      temps.trim();
      const int len      = temps.length();
      char *    tempchar = new char[len + 1];
      int       numGood  = 0;
      for (unsigned int i = 0; i < temps.length(); i++)
      {
         if (temps[i] == '-')
         {
            tempchar[numGood] = temps[i];
            numGood++;
         }
         if (temps[i] == '.')
         {
            tempchar[numGood] = temps[i];
            numGood++;
         }
         if (temps[i] >= '0' && temps[i] <= '9')
         {
            tempchar[numGood] = temps[i];
            numGood++;
         }
      }
      // temps.toCharArray(tempchar, temps.length()+1);
      //need to null terminate string or can't convert
      tempchar[numGood + 1] = '\0';
      float temp;
      temp = atof(tempchar);
      map_latitudes[row] = temp;
#ifdef DEBUG
      Serial.println("index: " + String(row));
      Serial.println(map_latitudes[row], 6);
#endif
      col++;
      token = strtok(NULL, delimiter);

      // fill the map_latitude, map_longitude, & map_file with tokens
      while (token != NULL)
      {
         switch (col % 3)
         {
         case 0: // latitude //skipped first time as handled above due to error handling
            map_latitudes[row] = atof(token);
#ifdef DEBUG
            Serial.println("index: " + String(row));
            Serial.println(map_latitudes[row], 6);
#endif
            col++;
            break;

         case 1: // longitude
            map_longitudes[row] = atof(token);
            col++;
#ifdef DEBUG
            { Serial.println(map_longitudes[row], 6); }
#endif
            break;

         case 2: // filename
            map_file_names[row] = token;
            col++;
#ifdef DEBUG
            { Serial.println(map_file_names[row]); }
#endif
            row++;
            break;

         default: // unexpected condition; print error
#ifdef DEBUG
            Serial.println("Unexpected error happened while reading map description file. Please verify the file is in the correct format. Planner may not work correctly if this message appears.");
#endif
            break;
         }
         token = strtok(NULL, delimiter);
      }
      int    numMaps         = row; //number of map origins loaded
      int    closestIndex    = -1;
      double closestDistance = MAX_DISTANCE;

#ifdef DEBUG
      Serial.print("vehiclePosition = ");
      Serial.print(vehiclePosition.latitude, 6);
      Serial.print(", ");
      Serial.println(vehiclePosition.longitude, 6);
#endif
      for (int i = 0; i < numMaps; i++)
      {
         double dist = vehiclePosition.distance_points_mm(map_latitudes[i], map_longitudes[i]);
#ifdef DEBUG
         Serial.println("Distance for index " + String(i) + " is " + String(dist / 1000000) + " km");
#endif
         if (dist < closestDistance)
         {
            closestIndex    = i;
            closestDistance = dist;
         }
      }
      if (closestIndex >= 0)            // Determine closest map to current location
      {
#ifdef DEBUG
         Serial.println("origin is now set to :" + String(orgin.latitude) + " " + String(orgin.longitude));
         Serial.println("map chosen index = " + String(closestIndex));
#endif
         //have to set origin with constructor so orgin.cos_lat will be set

         // Update origin global variable
         Position newOrg(map_latitudes[closestIndex], map_longitudes[closestIndex]);
         MapOrgin = newOrg;
         //orgin.latitude = map_latitudes[closestIndex];
         //orgin.longitude = map_longitudes[closestIndex];
#ifdef DEBUG
         Serial.println("closest index = " + String(closestIndex));
         Serial.print("origin is now set to : ");
         Serial.print(orgin.latitude, 6);
         Serial.print(", ");
         Serial.println(orgin.longitude, 6);
#endif
         //make all file names 12 in length so this works 12 char plus \0
         for (int i = 0; i < 12; i++)
         {
            nearestMap[i] = map_file_names[closestIndex][i];
         }
         //null terminate the string as it is C
         //map names should be 13 char and the 14th will by the \0
         nearestMap[12] = '\0';
#ifdef DEBUG
         Serial.println("nearestmap = " + String(nearestMap));
#endif
      }
      else    // closest index negative
      {
#ifdef DEBUG
         Serial.println("error determining closest map.");
         Serial.println("");
#endif
      }
      // Free the memory allocated for the buffer
      free(buffer);
      free(map_latitudes);
      free(map_longitudes);
      free(map_file_names);

      // close the file:
      myFile.close();
#ifdef DEBUG
      Serial.println("Map definitions loaded.");
      Serial.println("");
#endif
   }
   else    //the file didn't open, print an error:
   {
      myFile.close();
#ifdef DEBUG
      Serial.println("error opening MAP_DEFS.TXT");
      Serial.println("");
#endif
   }
}

/*---------------------------------------------------------------------------------------*/
// nearestMap
// Loads the map mapNodes from a file.
// Takes in the name of the file to load and loads the appropriate map.
// Returns true if the map was loaded.
// Returns false if the load failed.
bool Planner::LoadMap(char *fileName)
{
   // open the file. note that only one file can be open at a time,
   // so you have to close this one before opening another.

   //Try opening root then check each file name til find right one then do then close then rewindDirectory so can open new
   File root    = SD.open("/");
   bool found   = false;
   File useFile = root;

   while (!found)
   {
      useFile = root.openNextFile();
#ifdef DEBUG
      { Serial.println(String(useFile.name())); }
#endif
      if (!String(useFile.name()).equals(String(fileName)))
      {
         useFile.close();
#ifdef DEBUG
         { Serial.println("closed file: " + String(useFile.name())); }
#endif
      }
      else
      {
         found = true;
#ifdef DEBUG
         { Serial.println("file's match: " + String(useFile.name())); }
#endif
      }
   }
   // if the file opened okay, read from it:
   if (useFile)
   {
      // Initialize a string buffer to read lines from the file into
      // Allocate an extra char at the end to add null terminator
      char *buffer = (char *)malloc(useFile.size() + 1);

      // index for the next character to read into buffer
      char *ptr = buffer;

      // read from the file until there's nothing else in it:
      while (useFile.available())
      {
         *ptr = useFile.read();
         ++ptr;
      }
      // Null terminate the buffer string
      *ptr = '\0';

      // Set up tokenizer for the file buffer string
      const char *delimiter = " ,\n";
      char *      token;
      int         col = 0;
      int         row = 0;

      // get the first token //it is junk so move on
      token = strtok(buffer, delimiter);
      //get the first good token
      token = strtok(NULL, delimiter);
      //Serial.println("The first token is: " + String(token));
      // walk through other tokens
      while (token != NULL)
      {
         switch (col % 10)
         {
         case 0: // latitude
            mapNodes[row].east_mm = atol(token);
            col++;
            break;

         case 1: // longitude
            mapNodes[row].north_mm = atol(token);
            col++;
            break;

         case 2:                               // filename
            if (0 == strncmp(token, "END", 4)) //(token == "END")
            {
               mapNodes[row].destination[0] = END;
            }
            else
            {
               mapNodes[row].destination[0] = atoi(token);
            }
            col++;
            break;

         case 3: // filename
            if (0 == strncmp(token, "END", 4))
            {
               mapNodes[row].destination[1] = END;
            }
            else
            {
               mapNodes[row].destination[1] = atoi(token);
            }
            col++;
            break;

         case 4: // filename
            if (0 == strncmp(token, "END", 4))
            {
               mapNodes[row].destination[2] = END;
            }
            else
            {
               mapNodes[row].destination[2] = atoi(token);
            }
            col++;
            break;

         case 5: // filename
            if (0 == strncmp(token, "END", 4))
            {
               mapNodes[row].destination[3] = END;
            }
            else
            {
               mapNodes[row].destination[3] = atoi(token);
            }
            col++;
            break;

         //The distance array is the cost for taking that road. Temp holders is 1
         //may be used later to help choose path. distance normally 1 if .5 may be half the speed if 2 may be twice the speed
         case 6: // filename
            mapNodes[row].Distance[0] = atol(token);
            col++;
            break;

         case 7: // filename
            mapNodes[row].Distance[1] = atol(token);
            col++;
            break;

         case 8: // filename
            mapNodes[row].Distance[2] = atol(token);
            col++;
            break;

         case 9: // filename
            mapNodes[row].Distance[3] = atol(token);

            //this method below needs to be looked at in Common.cpp
            //convertLatLonToMM(mapNodes[row].east_mm, mapNodes[row].north_mm);
            col++;
            row++;
            break;

         default: // unexpected condition; print error
#ifdef DEBUG
            Serial.print("Unexpected error happened while reading map description file.");
            Serial.print("Please verify the file is in the correct format.");
            Serial.println("Planner may not work correctly if this message appears.");
#endif
            break;
         }

         token = strtok(NULL, delimiter);
      }
      map_points = row;

#ifdef DEBUG
      Serial.println("Test map");
      Serial.println(map_points);
      //To test LoadMap:
      for (int i = 0; i < map_points; i++)
      {
         Serial.println("inside the loop: " + String(i));
         Serial.print(mapNodes[i].east_mm);
         Serial.print(",");
         Serial.print(mapNodes[i].north_mm);
         Serial.print(",");
         Serial.print(mapNodes[i].destination[0]);
         Serial.print(",");
         Serial.print(mapNodes[i].destination[1]);
         Serial.print(",");
         Serial.print(mapNodes[i].destination[2]);
         Serial.print(",");
         Serial.print(mapNodes[i].destination[3]);
         Serial.print(",");
         Serial.print(mapNodes[i].Distance[0]);
         Serial.print(",");
         Serial.print(mapNodes[i].Distance[1]);
         Serial.print(",");
         Serial.print(mapNodes[i].Distance[2]);
         Serial.print(",");
         Serial.println(mapNodes[i].Distance[3]);
      }
   }
#endif
      // If file loaded, read data into mapNodes[]
      free(buffer);
      useFile.close();
#ifdef DEBUG
      { Serial.println("Closed the file: " + String(useFile.name())); }
#endif
   }
   else
   {
      // if the file didn't open, print an error:
      useFile.close();
#ifdef DEBUG
      { Serial.println("error opening: " + String(fileName)); }
#endif
      return(false);
   }
   return(true);
}
}
