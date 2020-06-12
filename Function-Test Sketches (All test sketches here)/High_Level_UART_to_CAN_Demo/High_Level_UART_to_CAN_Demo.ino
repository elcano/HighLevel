/*'''
  Sends CAN messages based on incoming UART commands. Works best using putty for UART: https://putty.org/
  Heavily borrows from https://copperhilltech.com/blog/app-note-arduino-due-2channel-can-bus-driver-software/
  '''
*/

#include "DueCANLayer.h"

// CAN Layer functions
extern byte canInit(byte cPort, long lBaudRate);
extern byte canTx(byte cPort, long lMsgID, bool bExtendedFormat, byte* cData, byte cDataLen);
extern byte canRx(byte cPort, long* lMsgID, bool* bExtendedFormat, byte* cData, byte* cDataLen);

void setup()
{
  // Set the serial interface baud rate
  Serial.begin(115200);

  // Initialize CAN controller
  if (canInit(0, CAN_BPS_500K) == CAN_OK)
    Serial.print("CAN0 Initialized Successfully.\n\r");
  else
    Serial.print("CAN0 Initialization Failed.\n\r");

}// end setup

int16_t steer = 0; //steering setting
int16_t throttle = 0; // throttle setting
int16_t brake = 0; //brake setting

bool sendCan = true; // flag to signal that CAN message is ready to send start by sending one to prime the pump


void loop()
{

  /////////////////////////////////////////////////////////////////
  byte cTxData[] = {(byte)(throttle >> 8), (byte)throttle, (byte)(brake >> 8), (byte)brake, (byte)(steer >> 8), (byte)steer, 0x00, 0x00}; // define message
  /////////////////////////////////////////////////////////////////

  while (1) // Endless loop
  {

    // Send the message when available
    if (sendCan)
    {
      if (canTx(0, 0x350, true, cTxData, 8) == CAN_OK) {
        Serial.print("Data sent successfully.\n\r");
        Serial.print("Throttle: ");
        Serial.println(throttle);
        Serial.print("Brake: ");
        Serial.println(brake);
        Serial.print("Steer: ");
        Serial.println(steer);
      }
      else
        Serial.print("Error during data transmission.\n\r");
      sendCan = false; // clear flag
    }

//depending on serial message that comes in, set the appropriate byte
    if (Serial.available()) {
      int command = Serial.read();
      switch (command) {
        case 'w':
          throttle ^= 0x0010;
          break;

        case 's':
          brake ^= 0x5000;
          break;

        case 'a':
          steer -= 10;
          break;

        case 'A':
          steer -= 50;
          break;

        case 'd':
          steer += 10;
          break;

        case 'D':
          steer += 50;
          break;

        default:
          throttle = 0;
          brake = 0;
          steer = 0;
      } // end switch
      sendCan = true; // set flag to send CAN message
    }


//set the data bytes for the CAN message
    cTxData[0] = (byte)(throttle >> 8);
    cTxData[1] = (byte)throttle;
    cTxData[2] = (byte)(brake >> 8);
    cTxData[3] = (byte)brake;
    cTxData[4] = (byte)(steer >> 8);
    cTxData[5] = (byte)steer;
    cTxData[6] = 0x00;
    cTxData[7] = 0x00;

        // Check for received message
        long lMsgID;
        bool bExtendedFormat;
        byte cRxData[8];
        byte cDataLen;
        if(canRx(0, &lMsgID, &bExtendedFormat, &cRxData[0], &cDataLen) == CAN_OK)
        {
          Serial.print("Rx - MsgID:");
          Serial.print(lMsgID, HEX);
          Serial.print(" Ext:");
          Serial.print(bExtendedFormat);
          Serial.print(" Len:");
          Serial.print(cDataLen);
          Serial.print(" Data:");
    
          for(byte cIndex = 0; cIndex < cDataLen; cIndex++)
          {
            Serial.print(cRxData[cIndex], HEX);
            Serial.print(" ");
          }// end for
    
          Serial.print("\n\r");
    
        }// end if

  }// end while

}// end loop
