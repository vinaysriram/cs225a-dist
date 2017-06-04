/*************************************************************
  File:      Servo_Redis.ino
  Contents:  This program contains implementation of control of a Hitec HS-645MG Servo using Redis via python (Pyserial)
             Program also uses Timer interrupts to implement event driven programming
             serves as an introduction to event-driven programming
  Notes:     Target: Arduino Uno
  Created:   Obinna Onyemepu (05/17/17)
 ************************************************************/

/*---------------Includes-----------------------------------*/
// #include <Timers.h>
#include <Servo.h>


/*---------------Module Defines-----------------------------*/

#define TIMER_2 2                 //Timer for interrupts
#define TIME_INTERVAL 1000  //Time interval to run servo

#define SERVO_PIN 9

#define ANGLE1 30
#define ANGLE2 75

#define RESET_COMPLETED 'R'  //Outgoing Serial Message to Redis which indicates that Arduino setup is completed
#define TRIGGER_CYCLE_COMPLETED 'T' //Outgoing Serial Message to Redis which indicates that trigger cycle is completed


///*---------------Module Function Prototypes-----------------*/
void globalEvents(void);
void triggerOff(void);
void triggerOn(void);


/*---------------State Definitions--------------------------*/
typedef enum {
   STATE_ON, STATE_OFF
} States_t;

/*---------------Module Variables---------------------------*/
static char incomingFromRedis;   //Incoming Serial Message from Redis
static Servo my_servo;
States_t state;

void setup() {
  Serial.begin(115200);
  my_servo.attach(SERVO_PIN);
  my_servo.write(ANGLE1);
  state = STATE_OFF;
  pinMode(13,OUTPUT);
//  digitalWrite(13,LOW);
  Serial.write(RESET_COMPLETED); // Send to Serial port that the board is done resetting
  Serial.flush();
}


void loop() {
  if (Serial.available() > 0) {
      incomingFromRedis = Serial.read();  // read the incoming byte:
  }

  if (incomingFromRedis == 49){
    my_servo.write(ANGLE2);
    delay(2000);
    my_servo.write(ANGLE1);
//    delay(2000);
    Serial.write(TRIGGER_CYCLE_COMPLETED);
    Serial.flush();
  }
  else{
    my_servo.write(ANGLE1);
  }
}
