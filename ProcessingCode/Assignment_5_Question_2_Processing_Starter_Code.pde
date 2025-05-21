// Pro_Graph2.pde
/*
 Based on the Arduining example which is based on the Tom Igoe example.
 Mofified by Cara Nunez 5/1/2019:
 -A wider line was used. strokeWeight(4);
 -Continuous line instead of vertical lines.
 -Bigger Window size 600x400.
 -------------------------------------------------------------------------------
 This program takes ASCII-encoded strings
 from the serial port at 9600 baud and graphs them. It expects values in the
 range 0 to 1023, followed by a newline, or newline and carriage return
 
 
 */

import processing.serial.*;

Serial myPort;        // The serial port

//*****************************************************************
//****************** Initialize Variables (START) *****************
//*****************************************************************

// STUDENT CODE HERE
float currentCursorVal;
float currentMassVal; 
float prevCursorVal;
float prevMassVal;
float mappedCursorVal;
int lf = 10; // 
String myString = null;
float dispWallPos; 
float wallPos = 0.5/10;
float minValueIn = -0.06;
float maxValueIn = 0.06;
float minValueOut = 0;
float maxValueOut = 600;

// variables for mass
float massEqPos = 0.5/100;
float mapped_massPos;
float current_massPos = massEqPos;

float last_massVel;
float mappedMassEqPos = map(massEqPos, minValueIn, maxValueIn, minValueOut, maxValueOut);
float mapped_cursorPos;
float last_massAcc;
float current_massVel = 0;
float current_massAcc = 0;
float massF;
float m = 2.0; //[kg]

// parameters of spring and damper
float k = 300.0; // stiffness
float b = 1.0; // damping
float k_user = 1000.0; // stiffness btwn user and mass

float f_user;
float f_spring;
float f_damper;

//*****************************************************************
//******************* Initialize Variables (END) ******************
//*****************************************************************


void setup () {
  // set the window size:
  size(600, 400);

  // List all the available serial ports
  println(Serial.list());
  // Check the listed serial ports in your machine
  // and use the correct index number in Serial.list()[].

  myPort = new Serial(this, Serial.list()[0], 9600);  //make sure baud rate matches Arduino

  // A serialEvent() is generated when a newline character is received :
  myPort.bufferUntil('\n');
  background(0);      // set inital background:
}
void draw () {
  // everything happens in the serialEvent()
  background(0); //uncomment if you want to control a ball
  stroke(127, 34, 255);     //stroke color
  strokeWeight(4);        //stroke wider
  
  //*****************************************************************
  //***************** Draw Objects in Scene (START) *****************
  //*****************************************************************

  // STUDENT CODE HERE
   float radius = 10;
   
  // (1) draw an ellipse (or other shape) to represent user position
    // HINT: the ellipse (or other shape) should not penetrate the mass object
    
    ellipse(mappedCursorVal, 200, radius*2, radius*2);
  // (2) draw an ellipse (or other shape) to represent the mass position
    ellipse(mapped_massPos+ 20, 200, radius*2, radius*2);
  // (3) draw the wall where the spring & damper are fixed (i.e., at 0.5 cm)
    dispWallPos = map(wallPos, minValueIn, maxValueIn, minValueOut, maxValueOut);
    
    line(dispWallPos, 0, dispWallPos, 400);
    line(dispWallPos, 200, mapped_massPos+3*radius, 200);
  
  
  // (4) draw a line between the fixed wall in (3) and the moving mass

  //*****************************************************************
  //****************** Draw Objects in Scene (END) ******************
  //*****************************************************************
}

void serialEvent (Serial myPort) {
  //*****************************************************************
  //** Read in Handle and Mass Positions from Serial Port (START) ***
  //*****************************************************************
  
  // STUDENT CODE HERE
  while (myPort.available() > 0)
    {
      myString = myPort.readStringUntil(lf);
      //print(myString);
      if (myString != null)
      {
        myString.trim();
        String[] values = split(myString, ",");
        currentCursorVal = float(values[0]);

        currentMassVal = float(values[1]);

      }

      if (Float.isNaN(currentCursorVal))
      {
        currentCursorVal = prevCursorVal;
        currentMassVal = prevMassVal;
      }
      
      else
      {
        mappedCursorVal = map(currentCursorVal, minValueIn, maxValueIn, minValueOut, maxValueOut);
        mapped_massPos = map(currentMassVal, minValueIn, maxValueIn, minValueOut, maxValueOut);
        prevCursorVal = mappedCursorVal;
      }

    }
  // (1) read the input string
    // HINT: use myPort.readStringUntil() with the appropriate argument
    
  // (2) if the input is null, don't do anything else
  
  // (3) else, trim and convert string to a number
  
  // (4) if the number is NaN, set current value to previous value
  //     otherwise: map the new value to the screen width
  //     & update previous value variable
  
  //*****************************************************************
  //**** Read in Handle and Mass Positions from Serial Port (END) ***
  //*****************************************************************
}
