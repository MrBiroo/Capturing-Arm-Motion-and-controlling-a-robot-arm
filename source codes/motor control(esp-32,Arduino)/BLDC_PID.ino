#include<Servo.h>
#include <Wire.h>
//Arduino: SDA: A4 SCL: A5
Servo ESC ;
#define relay_pin 5 
  
//**********Magnetic sensor things****************
int magnetStatus = 0; //value of the status register (MD, ML, MH)

int lowbyte; //raw angle 7:0
word highbyte; //raw angle 7:0 and 11:8
int rawAngle; //final raw angle 
float degAngle; //raw angle in degrees (360/4096 * [value between 0-4095])

int quadrantNumber, previousquadrantNumber; //quadrant IDs
float numberofTurns = 0; //number of turns
float correctedAngle = 0; //tared angle - based on the startup value
float startAngle = 0; //starting angle
float totalAngle = 0; //total absolute angular displacement
float previoustotalAngle = 0; //for the display printing
/**************************************************/

//********for PID******//
double dt, last_time;
double integral, previous, output = 0;
double kp, ki, kd; // PID gains
double setpoint = 0; //the target for PID
/*************************************************/

String message;

void setup() {

  ESC.attach (10,1000,2000);
  ESC.write(0); // initialising the BLDC
  delay(5000); // Wait for intit to finish

  pinMode(relay_pin,OUTPUT); // For controlling the direction of the motor

  CLKwise(); // init the direction of the motor
  delay(500);
  countCLKwise();
  delay(500);
  CLKwise();

  Serial.begin(115200);
  Serial.println("setup began!");

  Wire.begin(); //start i2C  
  Serial.println("Wire began!");
	//Wire.setClock(800000L); //fast clock
  Serial.println("checking for magnet");
  checkMagnetPresence(); //check the magnet (blocks until magnet is found)

  ReadRawAngle(); //make a reading so the degAngle gets updated
  startAngle = degAngle; //update startAngle with degAngle - for taring
  Serial.println(startAngle);
  delay(500);
  kp = 0.5; //p=0.5
  ki = 0.1; //i=0.2
  kd = 0;
  last_time = 0;

}

 double error = 0;
  float angle ;
void loop() {
  double now = millis();
  dt = (now - last_time)/1000.00; // 1000 to convert time from msec to sec 
  last_time = now; // updating the last time


  if (Serial.available() > 0 ) {
    message = Serial.readStringUntil('\n');
    Serial.println(message);
if (message.startsWith("A")) {
    String AngleString = message.substring(1);
    angle = AngleString.toFloat();
    setpoint = angle; // Update setpoint directly here
}
  }


  ReadRawAngle(); //ask the value from the sensor
  correctAngle(); //tare the value , the value of it is stored in : correctedAngle
  checkQuadrant(); //check quadrant, check rotations, check absolute angular position
   error = setpoint - totalAngle;

if (error > 0) {
    CLKwise(); // Move forward
} else if (error < 0) {
    countCLKwise(); // Move backward
} 
    

  double output = pid(error);
  
  // Make sure the value is within the expected range (0 to setpoint)
  output = constrain(output, 0, setpoint); // Constrain the speed to be between 0 and 100
    
  // Map the speed (0-setpoint) to the PWM range (1000-2000 microseconds)
  double pwmValue = map(output, 0, setpoint, 1400, 2000); // Map speed to PWM range

  if (abs(error) < 7) {
    Serial.println("motor reached its target...");
    ESC.writeMicroseconds(1000); // Stop the motor
    delay(500);
    return; // Skip rest of loop
}

  ESC.writeMicroseconds(pwmValue);
 
  Serial.print(setpoint);
  Serial.print(",");
  Serial.print(totalAngle);
  Serial.print(",");
  Serial.println(pwmValue);
  delay(50);
  

    



}
void countCLKwise()
{
  digitalWrite(relay_pin,LOW);

}
void CLKwise()
{
  digitalWrite(relay_pin,HIGH);

} 
double pid(double error)
{
  double proportional = error;
  integral += error * dt;
  double derivative = (error - previous) / dt;
  previous = error;
  double output = (kp * proportional) + (ki * integral) + (kd * derivative);
  return output;
}
void ReadRawAngle()
{ 
  ///Serial.println("Reading raw angle...");
  //7:0 - bits
  Wire.beginTransmission(0x36); //connect to the sensor
  Wire.write(0x0D); //figure 21 - register map: Raw angle (7:0)
  Wire.endTransmission(); //end transmission
  Wire.requestFrom(0x36, 1); //request from the sensor
  
  while(Wire.available() == 0); //wait until it becomes available 
  lowbyte = Wire.read(); //Reading the data after the request
 
  //11:8 - 4 bits
  Wire.beginTransmission(0x36);
  Wire.write(0x0C); //figure 21 - register map: Raw angle (11:8)
  Wire.endTransmission();
  Wire.requestFrom(0x36, 1);
  
  while(Wire.available() == 0);  
  highbyte = Wire.read();
  
  //4 bits have to be shifted to its proper place as we want to build a 12-bit number
  highbyte = highbyte << 8; //shifting to left
  //What is happening here is the following: The variable is being shifted by 8 bits to the left:
  //Initial value: 00000000|00001111 (word = 16 bits or 2 bytes)
  //Left shifting by eight bits: 00001111|00000000 so, the high byte is filled in
  
  //Finally, we combine (bitwise OR) the two numbers:
  //High: 00001111|00000000
  //Low:  00000000|00001111
  //      -----------------
  //H|L:  00001111|00001111
  rawAngle = highbyte | lowbyte; //int is 16 bits (as well as the word)

  //We need to calculate the angle:
  //12 bit -> 4096 different levels: 360° is divided into 4096 equal parts:
  //360/4096 = 0.087890625
  //Multiply the output of the encoder with 0.087890625
  degAngle = rawAngle * 0.087890625; 
  
  /*Serial.print("Deg angle: ");
  Serial.println(degAngle, 2); //absolute position of the encoder within the 0-360 circle*/
  
}
void correctAngle()
{
  //recalculate angle
  correctedAngle = degAngle - startAngle; //this tares the position

  if(correctedAngle < 0) //if the calculated angle is negative, we need to "normalize" it
  {
  correctedAngle = correctedAngle + 360; //correction for negative numbers (i.e. -15 becomes +345)
  }
  else
  {
    //do nothing
  }
  ///Serial.print("Corrected angle: ");
  ///Serial.println(correctedAngle, 2); //print the corrected/tared angle  
}

void checkQuadrant()
{
  /*
  //Quadrants:
  4  |  1
  ---|---
  3  |  2
  */

  //Quadrant 1
  if(correctedAngle >= 0 && correctedAngle <=90)
  {
    quadrantNumber = 1;
  }

  //Quadrant 2
  if(correctedAngle > 90 && correctedAngle <=180)
  {
    quadrantNumber = 2;
  }

  //Quadrant 3
  if(correctedAngle > 180 && correctedAngle <=270)
  {
    quadrantNumber = 3;
  }

  //Quadrant 4
  if(correctedAngle > 270 && correctedAngle <360)
  {
    quadrantNumber = 4;
  }
  //Serial.print("Quadrant: ");
  //Serial.println(quadrantNumber); //print our position "quadrant-wise"

  if(quadrantNumber != previousquadrantNumber) //if we changed quadrant
  {
    if(quadrantNumber == 1 && previousquadrantNumber == 4)
    {
      numberofTurns++; // 4 --> 1 transition: CW rotation
    }

    if(quadrantNumber == 4 && previousquadrantNumber == 1)
    {
      numberofTurns--; // 1 --> 4 transition: CCW rotation
    }
    //this could be done between every quadrants so one can count every 1/4th of transition

    previousquadrantNumber = quadrantNumber;  //update to the current quadrant
  
  }  
  //Serial.print("Turns: ");
  //Serial.println(numberofTurns,0); //number of turns in absolute terms (can be negative which indicates CCW turns)  

  //after we have the corrected angle and the turns, we can calculate the total absolute position
  totalAngle = (numberofTurns*360) + correctedAngle; //number of turns (+/-) plus the actual angle within the 0-360 range
  /*Serial.print("Total angle: ");
  Serial.println(totalAngle, 2); //absolute position of the motor expressed in degree angles, 2 digits*/
}

void checkMagnetPresence()
{  
  //This function runs in the setup() and it locks the MCU until the magnet is not positioned properly

  while((magnetStatus & 32) != 32) //while the magnet is not adjusted to the proper distance - 32: MD = 1
  {
    magnetStatus = 0; //reset reading

    Wire.beginTransmission(0x36); //connect to the sensor
    Wire.write(0x0B); //figure 21 - register map: Status: MD ML MH
    Wire.endTransmission(); //end transmission
    Wire.requestFrom(0x36, 1); //request from the sensor

    while(Wire.available() == 0); //wait until it becomes available 
    magnetStatus = Wire.read(); //Reading the data after the request

    Serial.print("Magnet status: ");
    Serial.println(magnetStatus, BIN); //print it in binary so you can compare it to the table (fig 21) 
    delay(1000);     
  }      
  
  //Status register output: 0 0 MD ML MH 0 0 0  
  //MH: Too strong magnet - 100111 - DEC: 39 
  //ML: Too weak magnet - 10111 - DEC: 23     
  //MD: OK magnet - 110111 - DEC: 55

  Serial.println("Magnet found!");
  //delay(1000);  
}
