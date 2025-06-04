//--------------------------------------------------------------------------
// Code to test basic Hapkit functionality (sensing and force output)
// Code updated by Cara Nunez 4.17.19
//--------------------------------------------------------------------------
// Parameters that define what environment to render
#define TESTING_1D_BALL


// Includes
#include <math.h>
#include "helpers.h"
#include "me327_AS5048A.h"
//#include <SoftwareSerial.h>

// Pin declares
int pwmXPin = 5;         // PWM output pin for motor 1
int dirXPin = 8;         // direction output pin for motor 1
int pwmYPin = 6;         // PWM output pin for motor 2
int dirYPin = 7;         // PWM output pin for motor 2
int sensorPosPin = A2;   // input pin for MR sensor
int sensorPosPinF = A4;  // input pin for MR sensor
int fsrPin = A3;         // input pin for FSR sensor


int vibMotorPin1 = 9;
int vibMotorPin2 = 10;

// Position tracking variables
int updatedPos = 0;      // keeps track of the latest updated value of the MR sensor reading
int rawPos = 0;          // current raw reading from MR sensor
int lastRawPos = 0;      // last raw reading from MR sensor
int lastLastRawPos = 0;  // last last raw reading from MR sensor
int flipNumber = 0;      // keeps track of the number of flips over the 180deg mark
int tempOffset = 0;
int rawDiff = 0;
int lastRawDiff = 0;
int rawOffset = 0;
int lastRawOffset = 0;
const int flipThresh = 700;  // threshold to determine whether or not a flip over the 180 degree mark occurred
//boolean flipped = false;
float handle_position;


// Position tracking variables - FOLLOWER
int updatedPosF = 0;      // keeps track of the latest updated value of the MR sensor reading
int rawPosF = 0;          // current raw reading from MR sensor
int lastRawPosF = 0;      // last raw reading from MR sensor
int lastLastRawPosF = 0;  // last last raw reading from MR sensor
int flipNumberF = 0;      // keeps track of the number of flips over the 180deg mark
int tempOffsetF = 0;
int rawDiffF = 0;
int lastRawDiffF = 0;
int rawOffsetF = 0;
int lastRawOffsetF = 0;
const int flipThreshF = 700;  // threshold to determine whether or not a flip over the 180 degree mark occurred

float handle_positionF;

// Calibration
double slopePos = -0.0217;
double slopePosF = 0.0201;
double offsetPos = 0;
double offsetPosF = 0;

// Kinematics variables
double xh = 0;       // position of the handle [m]
double theta_s = 0;  // Angle of the sector pulley in deg
double xh_prev;      // Distance of the handle at previous time step
double xh_prev2;
double dxh;  // Velocity of the handle
double dxh_prev;
double dxh_prev2;
double dxh_filt;  // Filtered velocity of the handle
double dxh_filt_prev;
double dxh_filt_prev2;

double xhF = 0;       // position of the handle [m]
double theta_sF = 0;  // Angle of the sector pulley in deg
double xh_prevF;      // Distance of the handle at previous time step
double xh_prev2F;
double dxhF;  // Velocity of the handle
double dxh_prevF;
double dxh_prev2F;
double dxh_filtF;  // Filtered velocity of the handle
double dxh_filt_prevF;
double dxh_filt_prev2F;


//*****************************************************************
//****************** Initialize Variables (START) *****************
//*****************************************************************
// Parameters & variables for wall and mass-spring-damper system

// STUDENT CODE HERE

//*****************************************************************
//******************* Initialize Variables (END) ******************
//*****************************************************************

// Force output variables
double force = 0;         // force at the handle
double Tp = 0;            // torque of the motor pulley
double duty = 0;          // duty cylce (between 0 and 255)
unsigned int output = 0;  // output command to the motor

int lf = 10;  //

String myString;
float dispWallPos;

int counter = 0;
unsigned long collisionXStartTime = 0;
bool vibrationXActive = false;

float processing_width = 600;
float current_pos_x = processing_width * 0.5, current_pos_y = processing_width * 0.5;
float prev_pos_x = processing_width * 0.5, prev_pos_y = processing_width * 0.5;

double dPosx;  // Velocity of the handle
double dPosx_prev;
double dPosx_prev2;
double dPosx_filt;  // Filtered velocity of the handle
double dPosx_filt_prev;
double dPosx_filt_prev2;

double dPosy;  // Velocity of the handle
double dPosy_prev;
double dPosy_prev2;
double dPosy_filt;  // Filtered velocity of the handle
double dPosy_filt_prev;
double dPosy_filt_prev2;

float dPosMag = 0;

float m = 0.02;
float fscalefactor = 0.4;
float g = -9.81;

float forcelim = 30.0;  // max force limit in Nm
float tslim = 40.0;     // max angle in degrees
float klim = 20.0;

float rp = 0.5 / 100.0;  // m
float rs = 5.08 / 100;   // m
float rh = 8.0 / 100;    // m
float j = rh * rp / rs;

float force_x = 0;
float pulley_torque_x = 0;
float duty_x = 0;

float force_y = 0;
float pulley_torque_y = 0;
float duty_y = 0;

//Vibration
unsigned long currentTime = 0;
float timeStep = 0.02;   // Simulation step (seconds)
float frequency = 50.0;  // Base surface frequency (Hz)
unsigned long lastTime = 0;
float phase = 0.0;

// collision
float x_coll;
float y_coll;

const unsigned long impactDuration = 120;      // Vibration length in ms
const unsigned long collisionCooldown = 600;   // Minimum time between vibrations in ms

// ==== Runtime Variables ====
bool prevCollisionX = false;
bool motorOnX = false;
unsigned long motorStartTimeX = 0;
unsigned long lastTriggerTimeX = 0;

bool prevCollisionY = false;
bool motorOnY = false;
unsigned long motorStartTimeY = 0;
unsigned long lastTriggerTimeY = 0;

// --------------------------------------------------------------
// Setup function -- NO NEED TO EDIT
// --------------------------------------------------------------


void setup() {
  // Set up serial communication
  Serial.begin(115200);
  //fromRemote.begin(9600);

  // Set PWM frequency
  setPwmFrequency(pwmXPin, 1);

  // Input pins
  pinMode(sensorPosPin, INPUT);  // set MR sensor pin to be an input
  pinMode(fsrPin, INPUT);        // set FSR sensor pin to be an input
  pinMode(sensorPosPinF, INPUT);

  // Output pins

  pinMode(pwmXPin, OUTPUT);  // PWM pin for motor A
  pinMode(dirXPin, OUTPUT);  // dir pin for motor A
  pinMode(pwmYPin, OUTPUT);  // PWM pin for motor B
  pinMode(dirYPin, OUTPUT);  // dir pin for motor B

  pinMode(vibMotorPin1, OUTPUT);

  // Initialize motor
  analogWrite(pwmXPin, 0);     // set to not be spinning (0/255)
  digitalWrite(dirXPin, LOW);  // set direction

  analogWrite(pwmYPin, 0);     // set to not be spinning (0/255)
  digitalWrite(dirYPin, LOW);  // set direction

  // Initialize position valiables
  lastLastRawPos = analogRead(sensorPosPin);
  lastRawPos = analogRead(sensorPosPin);

  lastLastRawPosF = analogRead(sensorPosPinF);
  lastRawPosF = analogRead(sensorPosPinF);


  initialize_loop_checker();
}


// --------------------------------------------------------------
// Main Loop
// --------------------------------------------------------------
void loop() {
  currentTime = accurate_micros() * 0.001;
  //*************************************************************
  //*** Section 1. Compute position in counts (do not change) ***
  //*************************************************************

  // ***************************** FOR LEADER HANDLE
  // Get voltage output by MR sensor
  rawPos = analogRead(sensorPosPin);  //current raw position from MR sensor
  //float remote_angle = send_receive_remote_arduino(handle_position);
  // Calculate differences between subsequent MR sensor readings
  rawDiff = rawPos - lastRawPos;          //difference btwn current raw position and last raw position
  lastRawDiff = rawPos - lastLastRawPos;  //difference btwn current raw position and last last raw position
  rawOffset = abs(rawDiff);
  lastRawOffset = abs(lastRawDiff);

  // Update position record-keeping vairables
  lastLastRawPos = lastRawPos;
  lastRawPos = rawPos;

  // Keep track of flips over 180 degrees
  if ((lastRawOffset > flipThresh) && (!flipped)) {  // enter this anytime the last offset is greater than the flip threshold AND it has not just flipped
    if (lastRawDiff > 0) {                           // check to see which direction the drive wheel was turning
      flipNumber--;                                  // cw rotation
    } else {                                         // if(rawDiff < 0)
      flipNumber++;                                  // ccw rotation
    }
    if (rawOffset > flipThresh) {                    // check to see if the data was good and the most current offset is above the threshold
      updatedPos = rawPos + flipNumber * rawOffset;  // update the pos value to account for flips over 180deg using the most current offset
      tempOffset = rawOffset;
    } else {                                             // in this case there was a blip in the data and we want to use lastactualOffset instead
      updatedPos = rawPos + flipNumber * lastRawOffset;  // update the pos value to account for any flips over 180deg using the LAST offset
      tempOffset = lastRawOffset;
    }
    flipped = true;                                 // set boolean so that the next time through the loop won't trigger a flip
  } else {                                          // anytime no flip has occurred
    updatedPos = rawPos + flipNumber * tempOffset;  // need to update pos based on what most recent offset is
    flipped = false;
  }

  //****************** FOR FOLLOWER HANDLE

  // Get voltage output by MR sensor
  rawPosF = analogRead(sensorPosPinF);  //current raw position from MR sensor
  //Serial.println(sensorPosPinF);
  // Calculate differences between subsequent MR sensor readings
  rawDiffF = rawPosF - lastRawPosF;          //difference btwn current raw position and last raw position
  lastRawDiffF = rawPosF - lastLastRawPosF;  //difference btwn current raw position and last last raw position
  rawOffsetF = abs(rawDiffF);
  lastRawOffsetF = abs(lastRawDiffF);

  // Update position record-keeping vairables
  lastLastRawPosF = lastRawPosF;
  lastRawPosF = rawPosF;

  // Keep track of flips over 180 degrees
  if ((lastRawOffsetF > flipThreshF) && (!flippedF)) {  // enter this anytime the last offset is greater than the flip threshold AND it has not just flipped
    if (lastRawDiffF > 0) {                             // check to see which direction the drive wheel was turning
      flipNumberF--;                                    // cw rotation
    } else {                                            // if(rawDiff < 0)
      flipNumberF++;                                    // ccw rotation
    }
    if (rawOffsetF > flipThreshF) {                      // check to see if the data was good and the most current offset is above the threshold
      updatedPosF = rawPosF + flipNumberF * rawOffsetF;  // update the pos value to account for flips over 180deg using the most current offset
      tempOffsetF = rawOffsetF;
    } else {                                                // in this case there was a blip in the data and we want to use lastactualOffset instead
      updatedPosF = rawPosF + flipNumber * lastRawOffsetF;  // update the pos value to account for any flips over 180deg using the LAST offset
      tempOffsetF = lastRawOffsetF;
    }
    flippedF = true;                                    // set boolean so that the next time through the loop won't trigger a flip
  } else {                                              // anytime no flip has occurred
    updatedPosF = rawPosF + flipNumberF * tempOffsetF;  // need to update pos based on what most recent offset is
    flippedF = false;
  }

  //*****************************************************************
  //************ Compute Position in Meters (START) *****************
  //*****************************************************************

  if (counter == 0) {
    offsetPos = updatedPos;
    offsetPosF = updatedPosF;
  }

  // MAG to DEGREE CALCS
  double rp = 1.25 / 100.0;  // m
  double rs = 5.08 / 100;    // m
  double rh = 8.0 / 100;     // m
  double j = rh * rp / rs;

  double ts = -slopePos * (updatedPos - offsetPos);
  double tsF = -slopePosF * (updatedPosF - offsetPosF);

  /* Old Depreciated Code
  //xh = rh * ts * M_PI / 180;
  //xhF = rh * tsF * M_PI / 180;
  //*****************************************************************
  //**************** Compute Position in Meters (END) ***************
  //*****************************************************************

  // Calculate velocity with loop time estimation
  dxh = (double)(xh - xh_prev) / 0.001;

  // Calculate the filtered velocity of the handle using an infinite impulse response filter
  dxh_filt = .9 * dxh + 0.1 * dxh_prev;

  // Record the position and velocity
  xh_prev2 = xh_prev;
  xh_prev = xh;

  dxh_prev2 = dxh_prev;
  dxh_prev = dxh;

  dxh_filt_prev2 = dxh_filt_prev;
  dxh_filt_prev = dxh_filt;*/

  if (Serial.available() > 0) {
    String string_recieved = Serial.readStringUntil('\n');

    String strArray[5];
    int index = 0;
    int start = 0;
    int end = 0;

    while ((end = string_recieved.indexOf(',', start)) > -1) {
      strArray[index] = string_recieved.substring(start, end);
      start = end + 1;
      index++;
    }

    strArray[index] = string_recieved.substring(start);  // Get the last part



    // int commaIndex = string_recieved.indexOf(',');

    // if (commaIndex != -1) {
    //   String xStr = string_recieved.substring(0, commaIndex);
    //   String yStr = string_recieved.substring(commaIndex + 1);
    //   x_coll = string_recieved.substring(commaIndex + 2);
    //   y_coll = string_recieved.substring(commaIndex + 3);


    float temp_x = strArray[0].toFloat();
    float temp_y = strArray[1].toFloat();
    x_coll = strArray[2].toFloat();
    y_coll = strArray[3].toFloat();
    dPosMag = strArray[4].toFloat();


    if (!isnan(temp_x) && !isnan(temp_y)) {
      prev_pos_x = current_pos_x;
      prev_pos_y = current_pos_y;
      current_pos_x = temp_x;
      current_pos_y = temp_y;

    } else {
      current_pos_x = prev_pos_x;
      current_pos_y = prev_pos_y;
    }
  }




  if (counter % 100 == 0) {
    Serial.print(ts, 2);
    Serial.print(",");
    Serial.print(tsF, 2);  // sending values to processing

    // for debugging arduino stuff by sending to processing - rm/add!!! "ln" from line above
    Serial.print(",");
    Serial.println(force_x);
    // Serial.print(",");
    // Serial.println(y_coll);
  }

  counter++;


bool currentCollisionX = (x_coll == 1);  // Ball touching wall on X axis
bool risingEdgeX = currentCollisionX && !prevCollisionX;

if (risingEdgeX && currentTime - lastTriggerTimeX > collisionCooldown) {
  analogWrite(vibMotorPin1, 200);  // Turn on motor for X collision
  motorOnX = true;
  motorStartTimeX = currentTime;
  lastTriggerTimeX = currentTime;
}

if (motorOnX && currentTime - motorStartTimeX > impactDuration) {
  analogWrite(vibMotorPin1, 0);  // Turn off motor for X collision
  motorOnX = false;
}

prevCollisionX = currentCollisionX;  // Update for next loop


// For Y collision
bool currentCollisionY = (y_coll == 1);  // Ball touching wall on Y axis
bool risingEdgeY = currentCollisionY && !prevCollisionY;

if (risingEdgeY && currentTime - lastTriggerTimeY > collisionCooldown) {
  analogWrite(vibMotorPin2, 200);  // Turn on motor for Y collision (using second motor pin)
  motorOnY = true;
  motorStartTimeY = currentTime;
  lastTriggerTimeY = currentTime;
}

if (motorOnY && currentTime - motorStartTimeY > impactDuration) {
  analogWrite(vibMotorPin2, 0);  // Turn off motor for Y collision
  motorOnY = false;
}

prevCollisionY = currentCollisionY;  

  // motor commands based on position of ball and relative moment
  if (current_pos_x >= 290 && current_pos_x <= 320) {
    force_x = 0.1 * (current_pos_x - processing_width / 2) * m * g * cos(ts * M_PI / 180.0);
  } else {
    force_x = (current_pos_x - processing_width / 2) * m * g * cos(ts * M_PI / 180.0);
  }

  pulley_torque_x = j * force_x;




  if (force_x > forcelim) {
    force_x = forcelim;
  }
  if (force_x < -1 * forcelim) {
    force_x = -1 * forcelim;
  }

  // if (ts > tslim) {
  //   force_x += (-klim)*(ts-tslim);
  // }
  // if (ts < -1 * tslim) {
  //   force_x += (-klim)*(ts+tslim);
  // }


  if (force_x > 0) {
    digitalWrite(dirXPin, LOW);
  } else {
    digitalWrite(dirXPin, HIGH);
  }

  // Motor B
  if (current_pos_y >= 290 && current_pos_y <= 320) {
    force_y = 0.1 * (current_pos_y - processing_width / 2) * m * g * cos(tsF * M_PI / 180.0);  // + y_coll*dPosx_filt*2;
  } else {
    force_y = (current_pos_y - processing_width / 2) * m * g * cos(tsF * M_PI / 180.0);
  }
  pulley_torque_y = j * force_y;


  if (force_y > forcelim) {
    force_y = forcelim;
  }
  if (force_y < -1 * forcelim) {
    force_y = -1 * forcelim;
  }

  // if (tsF > tslim) {
  //   force_x += (-klim)*(ts-tslim);
  // }
  // if (tsF < -1 * tslim) {
  //   force_x += (-klim)*(ts+tslim);
  // }


  if (force_y > 0) {
    digitalWrite(dirYPin, LOW);
  } else {
    digitalWrite(dirYPin, HIGH);
  }

  int pwmValue = (int)(0.4 * 255);

  //analogWrite(vibMotorPin1, x_coll*pwmValue);
  // analogWrite(vibMotorPin2, y_coll*pwmValue);


  dPosx = (double)(current_pos_x - prev_pos_x) / 0.001;

  dPosx_filt = .9 * dPosx + 0.1 * dPosx_prev;

  dPosx_prev2 = dPosx_prev;
  dPosx_prev = dPosx;

  dPosx_filt_prev2 = dPosx_filt_prev;
  dPosx_filt_prev = dPosx_filt;

  dPosy = (double)(current_pos_y - prev_pos_y) / 0.001;

  dPosy_filt = .9 * dPosy + 0.1 * dPosy_prev;

  dPosy_prev2 = dPosy_prev;
  dPosy_prev = dPosy;

  dPosy_filt_prev2 = dPosy_filt_prev;
  dPosy_filt_prev = dPosy_filt;

  // Compute the duty cycle required to generate Tp (torque at the motor pulley)
  duty_x = sqrt(abs(pulley_torque_x) / 0.03);
  duty_y = sqrt(abs(pulley_torque_y) / 0.03);

  // Make sure the duty cycle is between 0 and 100%
  if (duty_x > 1) {
    duty_x = 1;
  } else if (duty_x < 0) {
    duty_x = 0;
  }
  unsigned int output_x = (int)(fscalefactor * duty_x * 255);  // convert duty cycle to output signal
  analogWrite(pwmXPin, output_x);                              // output the signal

  if (duty_y > 1) {
    duty_y = 1;
  } else if (duty_y < 0) {
    duty_y = 0;
  }
  unsigned int output_y = (int)(fscalefactor * duty_y * 255);  // convert duty cycle to output signal
  analogWrite(pwmYPin, output_y);                              // output the signal
}
//*************************************************************
//******* Assign a Motor Output Force in Newtons (END) ********
//*************************************************************
//*************************************************************



//*************************************************************
//************ Force output (do not change) *******************
//*************************************************************

// Determine correct direction for motor torque
//   if(force > 0) {
//     digitalWrite(dirPin, HIGH);
//   } else {
//     digitalWrite(dirPin, LOW);
//   }

//   // Compute the duty cycle required to generate Tp (torque at the motor pulley)
//   duty = sqrt(abs(Tp)/0.03);

//   // Make sure the duty cycle is between 0 and 100%
//   if (duty > 1) {
//     duty = 1;
//   } else if (duty < 0) {
//     duty = 0;
//   }
//   output = (int)(duty* 255);   // convert duty cycle to output signal
//   analogWrite(pwmPin,output);  // output the signal


// --------------------------------------------------------------
// Function to set PWM Freq -- DO NOT EDIT
// --------------------------------------------------------------
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if (pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch (divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if (pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if (pin == 3 || pin == 11) {
    switch (divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}
