#include <DAC_MCP49xx.h>
#include <SPI.h>
#include "boards.h"
#include "RBL_nRF8001.h"
#include "services.h"
#include <Timer.h>

// Ultrasonic Sensor Configuration could be 
#define TRIGGER_PIN1 2   //sensor trig 1
#define ECHO_PIN1 3      //sensor echo 1
#define TRIGGER_PIN2 1   //sensor trig 2
#define ECHO_PIN2 9      //sensor echo 2
#define MAX_DISTANCE 50 
int maxRange  = 50; // max distance before wheelchair turns or stops 

// BLE Configuration
// Buffer large enough to hold the max size of a joystick update message:
// xx\0x
// offset\0direction

#define BUFFER_LEN 7
char buffer[BUFFER_LEN] = {0};
short bufferLen = 0;
char* offsetPtr;
char* anglePtr;

// Constants for motor controller
// 4095
DAC_MCP49xx MOTOR_BF_POWER(DAC_MCP49xx::MCP4921, 5);
DAC_MCP49xx MOTOR_LR_POWER(DAC_MCP49xx::MCP4921, 6);
DAC_MCP49xx MIDDLE(DAC_MCP49xx::MCP4921, 7);
#define MOTOR_POWER_CENTER 1500 

void setup()
{
    // serial moniter setting
  Serial.begin(9600);
  
   pinMode(TRIGGER_PIN1, OUTPUT); //
   pinMode(ECHO_PIN1, INPUT);     //
   pinMode(TRIGGER_PIN2, OUTPUT); //
   pinMode(ECHO_PIN2, INPUT);     //
  
  // Initialize BLE
  ble_begin();

  // Set offset pointer to the beginning of the buffer
  offsetPtr = &buffer[0];

  // Configure motor controller pins
  MOTOR_LR_POWER.output(MOTOR_POWER_CENTER);
  MOTOR_BF_POWER.output(MOTOR_POWER_CENTER);
  MIDDLE.output(MOTOR_POWER_CENTER);
  Serial.println("Setup Complete");
}
  void loop()
  {
    //low high to find distance from objects sensor 1/2
   digitalWrite(TRIGGER_PIN1, LOW); 
   digitalWrite(TRIGGER_PIN2, LOW); 
   delayMicroseconds(2); 
   digitalWrite(TRIGGER_PIN1, HIGH);
   digitalWrite(TRIGGER_PIN2, HIGH);
   delayMicroseconds(10); 
   digitalWrite(TRIGGER_PIN1, LOW);
   digitalWrite(TRIGGER_PIN2, LOW);

     // putting distance found into duration 
   int duration1 = pulseIn(ECHO_PIN1, HIGH);
   int duration2 = pulseIn(ECHO_PIN2, HIGH);

    //turning distance into inchs 
   int distance1 = duration1/58.2;
   int distance2 = duration2/58.2;
   
    // While BLE data is available
    while (ble_available())
    {
      // Read a single character from BLE
      char bleChar = ble_read();

      // If this isn't a line feed character
      if(bleChar != 0x0A) {

        // And the buffer isn't full
        if(bufferLen < BUFFER_LEN) {

          // Add it to the buffer
          buffer[bufferLen++] = bleChar;

          // If this is a null terminator, direction starts at the next character
          if(bleChar == '\0') {
            anglePtr = &buffer[bufferLen];
          }
        }
      }
      else {
        // If this is a line feed character, this is the end of the message

          // Null-terminate the buffer
        buffer[bufferLen++] = '\0';

        // Use these values to drive the motor
        int offset = atoi(offsetPtr);
        float angle = atof(anglePtr);

        //wait for SPI bus to be free
        while (ble_busy()) {}
        bufferLen = 0;

        //compensate for start
        int fb_offset =  sin(angle) * 2 * offset * -1;
        int lr_offset = cos(angle) * 2 * offset;

        int i = maxRange; //maxRange = distance before change
        //if something in way of left sensor move right
        if (distance1 < maxRange) {
           lr_offset = 200;
           fb_offset = 0;
           Serial.println("RIGHT");
         }   
        //if something in way of right sensor move left
        if (distance2 < maxRange) {
           lr_offset = -200;
           fb_offset = 0;
           Serial.println("LEFT");
        }
        //if something in way of both sensors stop
        if (distance1 < maxRange && distance2<maxRange){
           fb_offset = 0; 
           lr_offset = 0; 
        Serial.println("STOPED, somthing blocking wheelchair");
         }
        //output to go left or right 
        MOTOR_LR_POWER.output(lr_offset + MOTOR_POWER_CENTER);

        //output to go left or right 
        MOTOR_BF_POWER.output(fb_offset + MOTOR_POWER_CENTER);
 
       } //end null terminated message
    } //end ble_available()
  ble_do_events();
  }
    
 
