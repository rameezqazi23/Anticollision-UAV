/*
*****************************************************************************************************************************************************************************
<===========================================================================================================================================================================>

                                                                 """CRASH AVOIDANCE CODE FOR QUADCOPTER"""
                                                               
               ^^^^^^ This code was written/modified by students of "Electrical Engineering" from "Usman Institute of technology". September 2020 ^^^^^^
                     
   The purpose of this code is to sense objects in four directions of the quadcopter(front, back, left and right), in order to avoide any contact between quadcopter and 
      objects such as, walls, trees, humans, etc and calculate the movement to avoide that object and sends that data to pixhawk. The sensors used in this code are 
    TFmini LIDar x1 in the front of quad and x3 Ultrasonic sensors(HC SR04) on other three sides. The pin connections are commented below, adjacent to the code line. 
      The sensors are connected to "Arduino UNO" and UNO is connected to "PIXHAWK" flight controller via TELEM2 port, which is used for serial communication (UART). 

<===========================================================================================================================================================================>
*****************************************************************************************************************************************************************************
*/

//#include "TFMini.h"                                                                      // Lidar Library
#include <NewPing.h>                                                                       // Ultrasonic Library for easy trigring and echos
#include <SoftwareSerial.h>                                                                // Serial communication: Universal Asynchronous Receiver - Transmitter (UART)
#include "C:\Users\mohid\OneDrive\Documents\Arduino\libraries\mavlink\common\mavlink.h"    // MAVlink library for communication b/w Pixhawk and Arduino
                          //  --->   // For the Above include, download MAVlink librarhy for arduino and add in arduino libraries folder

#define RANGE  100


SoftwareSerial mySerial(10, 11);                                                           // RX, TX (For MAVlink communication)
//SoftwareSerial mytf(7, 8);                                                               // RX(TX of TFmini), TX(RX of TFmini) For TFmini
//TFMini TFMINI;
  
NewPing SONARFRONT(3, 3, 100);                                                             // Front Sonar (if not using TFmini)----> D3
NewPing SONARRIGHT(4, 4, 100);                                                             // Right Sonar--------------------------> D4
NewPing SONARBACK(5, 5, 100);                                                              // Back Sonar---------------------------> D5
NewPing SONARLEFT(6, 6, 100);                                                              // Left Sonar---------------------------> D6


unsigned long HeartbeatTime = 0;
int PITCH = 0, ROLL = 0, Pt = 0, Rt = 0;

struct Sensors {
  int RAW_VAL[5]  = {0}, DISTANCE  = 0;
  bool DETECT  = false;
}Sensor[4];

void setup() {
  Serial.begin(9600);
  mySerial.begin(57600);
  
//  mytf.begin(TFMINI_BAUDRATE);                                                           // Initialize the data rate for the SoftwareSerial port
//  TFMINI.begin(&mytf);                                                                   // Initialize the TF Mini sensor.
  
}

void loop() {
  if ( (millis() - HeartbeatTime) > 1000 ) {
    HeartbeatTime = millis();
    SEND_HEART_BEAT();
  }
  READ_SENSOR();                                                                           // Function for sensor data reading
  BUILD_MAVLINK_DATA();                                                                    // Function for Sending data to Pixhawk via MAVLink

}


void READ_SENSOR() {
  for (uint8_t i = 0; i < 4; i++)
      Sensor[i].RAW_VAL[4] = Sensor[i].RAW_VAL[3], Sensor[i].RAW_VAL[3] = Sensor[i].RAW_VAL[2], Sensor[i].RAW_VAL[2] = Sensor[i].RAW_VAL[1], Sensor[i].RAW_VAL[1] = Sensor[i].RAW_VAL[0];
      
  Sensor[0].RAW_VAL[0] = SONARFRONT.ping_cm();                 //     ---->     // Change: "SONARFRONT.ping_cm()" with: "TFMINI.getDistance(" if using front Ultrasonic
  Sensor[1].RAW_VAL[0] = SONARRIGHT.ping_cm();
  Sensor[2].RAW_VAL[0] = SONARBACK.ping_cm(); 
  Sensor[3].RAW_VAL[0] = SONARLEFT.ping_cm();

  for (uint8_t i = 0; i < 4; i++) {
    int SUM = 0, COUNT = 0;
    for (uint8_t j = 0; j < 5; j++) 
       if (Sensor[i].RAW_VAL[j] != 0  && Sensor[i].RAW_VAL[j] < 70) 
         SUM += Sensor[i].RAW_VAL[j], COUNT++;
      
    if (COUNT > 3)
      Sensor[i].DISTANCE = SUM / COUNT;
    else
      Sensor[i].DISTANCE = 0;

    if (Sensor[i].DISTANCE != 0 && Sensor[i].DISTANCE < RANGE)
      Sensor[i].DETECT = true;
    else
      Sensor[i].DETECT = false;
  }

 Serial.print("\nRAW_VAL: "); 
 Serial.print(Sensor[0].DISTANCE); 
 Serial.print(", "); 
 Serial.print(Sensor[1].DISTANCE); 
 Serial.print(", "); 
 Serial.print(Sensor[2].DISTANCE); 
 Serial.print(", "); 
 Serial.print(Sensor[3].DISTANCE); 
 Serial.print("cm\n\r");

}

uint8_t N = 0;
void BUILD_MAVLINK_DATA() {

  CALCULATE_PITCH();
  CALCULATE_ROLL();

  if ( PITCH != Pt || ROLL != Rt )  
    Pt = PITCH, Rt  = ROLL, SEND_DATA(PITCH, ROLL);

}


void CALCULATE_PITCH() {
  
  if(Sensor[0].DETECT && Sensor[2].DETECT && Sensor[0].DISTANCE < Sensor[2].DISTANCE )       //  FRONT + BACK, FRONT < BACK
      PITCH = STICK( Sensor[0].DISTANCE, 1 );                
  else if(Sensor[0].DETECT && Sensor[2].DETECT && Sensor[0].DISTANCE > Sensor[2].DISTANCE )  //  FRONT + BACK, FRONT > BACK
      PITCH = STICK( Sensor[2].DISTANCE, 0 );  
  else if(Sensor[0].DETECT && !Sensor[2].DETECT)                                             //  FRONT ONLY                               
      PITCH = STICK( Sensor[0].DISTANCE, 1 );  
  else if(!Sensor[0].DETECT && Sensor[2].DETECT)                                             //  BACK ONLY
      PITCH = STICK( Sensor[2].DISTANCE, 0 ); 
  else
      PITCH = 0;                                                                             //  NOTHING
}

void CALCULATE_ROLL() {
  if(Sensor[1].DETECT && Sensor[3].DETECT && Sensor[1].DISTANCE < Sensor[3].DISTANCE )       //  RIGHT + LEFT, RIGHT < LEFT
      ROLL = STICK( Sensor[1].DISTANCE, 0 );                
  else if(Sensor[1].DETECT && Sensor[3].DETECT && Sensor[1].DISTANCE > Sensor[3].DISTANCE )  //  RIGHT + LEFT, RIGHT > LEFT
      ROLL = STICK( Sensor[3].DISTANCE, 1 );  
  else if(Sensor[1].DETECT && !Sensor[3].DETECT)                                             //  RIGHT ONLY                               
      ROLL = STICK( Sensor[1].DISTANCE, 0 );  
  else if(!Sensor[1].DETECT && Sensor[3].DETECT)                                             //  LEFT ONLY
      ROLL = STICK( Sensor[3].DISTANCE, 1 ); 
  else
      ROLL = 0;                                                                              ////  NOTHING  
}


int STICK( int VAL, bool DIR) {
  
   VAL=constrain(VAL, 50, 100);
   if(DIR)
      return map(VAL, 100, 50, 1550, 1750);
   else
      return map(VAL, 100, 50, 1450, 1250);
}


//============================MAVLINK==========================//
mavlink_message_t msg;
uint8_t buf[MAVLINK_MAX_PACKET_LEN];
uint16_t len;
 
void SEND_HEART_BEAT() {
  mavlink_msg_heartbeat_pack(255, 0, &msg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, 0, 1, 0);    // System ID = 255 = GCS
  len = mavlink_msg_to_send_buffer(buf, &msg);

  mySerial.write(buf, len);
}

void SEND_DATA(int P, int R) {
  mavlink_msg_rc_channels_override_pack(255, 0 , &msg, 1, 0, R, P, 0, 0, 0, 0, 0, 0);              // CHANNEL = 1(ROLL), 2(PITCH), 3(Throttle), 4(Yaw) , 5, 6, 7, 8
  len = mavlink_msg_to_send_buffer(buf, &msg);
  
  mySerial.write(buf, len);
  Serial.print("\n\rPITCH: ");  
  Serial.print(P);  
  Serial.print(", ");  
  Serial.print(" ROLL: ");  
  Serial.print(R);
}
