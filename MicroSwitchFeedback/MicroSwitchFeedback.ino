/*************************************************************
  File:      MicroSwitch.ino
  Contents:  This program contains implementation of microswitch to activate/trigger RGB LED's
  Notes:     Target: Arduino Nano
  Created:   Obinna Onyemepu (05/31/17)
 ************************************************************/

#define leftTop 2
#define rightTop 3
#define leftBottom 4
#define rightBottom 5

#define redLED  9
#define greenLED  10
#define blueLED  11

#define TIME_INTERVAL 200


void setLEDColor(int red, int green, int blue);
void turn_RGB_On(void);
void turn_RGB_Off(void);

void setup() {
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  
  pinMode(leftTop, INPUT);
  pinMode(rightTop, INPUT);
  pinMode(leftBottom, INPUT);
  pinMode(rightBottom, INPUT);

  pinMode(redLED,OUTPUT);
  pinMode(greenLED,OUTPUT);
  pinMode(blueLED,OUTPUT);
  Serial.write('R');
  Serial.flush();
  turn_RGB_Off();
}

void loop() {

  if (digitalRead(leftTop) == HIGH || digitalRead(rightTop) == HIGH || digitalRead(leftBottom) == HIGH || digitalRead(rightBottom) == HIGH) {
    Serial.write('T');
    Serial.flush();
    Serial.write('T');
    Serial.flush();
    turn_RGB_On();
  } 
  
  else {
    Serial.write('S');
    Serial.flush();
    turn_RGB_Off();
  }

}

//___________To set the required color of the LED_____
void setLEDColor(int red, int green, int blue){
  analogWrite(redLED, red);
  analogWrite(greenLED, green);
  analogWrite(blueLED, blue);
}


void turn_RGB_On(void){
  setLEDColor(255, 0, 0);  // red
  delay(TIME_INTERVAL);
  setLEDColor(0, 255, 0);  // green
  delay(TIME_INTERVAL);
  setLEDColor(0, 0, 255);  // blue
  delay(TIME_INTERVAL);
  setLEDColor(255, 255, 0);  // yellow
  delay(TIME_INTERVAL);  
  setLEDColor(80, 0, 80);  // purple
  delay(TIME_INTERVAL);
  setLEDColor(0, 255, 255);  // aqua
  delay(TIME_INTERVAL);
}

void turn_RGB_Off(void){
      setLEDColor(0, 0, 0);
}

