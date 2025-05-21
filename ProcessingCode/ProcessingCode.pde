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

// current ball pos, vel
float ballpos_current = width/2;
float ballvel_current = 0;


// prev ball pos, vel
float ballpos_prev = width/2;
float ballvel_prev = 0;

// getting values from serial
int lf = 10; 
String myString = null;

// mapping shit
float minValueIn = -45;
float maxValueIn = 45;
float minValueOut = 0;
float maxValueOut = 800;
float current_angle;
float prev_angle;

// physics shit
float wall_slope;
float ball_mass = 0.25; // kg?? idk help
float ballacc;
float g = 9.81; // m/s2




//*****************************************************************
//******************* Initialize Variables (END) ******************
//*****************************************************************


void setup () {
  // set the window size:
  size(600, 400);
  
  // List all the available serial ports
  //println(Serial.list());
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
   float dt = 1.0/frameRate  * 5;
   wall_slope = tan(current_angle);
   
   
   
   float len_platform = 600;
   
   float x1 = width/2 - len_platform/2;
   float x2 = width/2 + len_platform/2;
   
   float y1 = -wall_slope * len_platform/2 + height/2;
   float y2 = wall_slope * len_platform/2 + height/2;
    
    
  
    ballacc = g*sin(current_angle);
    
    ballvel_current = ballvel_prev + ballacc * dt;
    ballpos_current = ballpos_prev + (ballvel_prev + ballvel_current)/2 * dt;
    
    
     
    line(x1, y1, x2, y2);
   ellipse(ballpos_current, height/2, radius*2, radius*2);
   
   ballvel_prev = ballvel_current;
    ballpos_prev = ballpos_current;

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
        myString = myString.trim();
        String[] values = split(myString, ",");
        
        current_angle = float(values[0]);
        current_angle = radians(current_angle);
        //float random = float(values[1]);
        
      }

      if (Float.isNaN(current_angle))
      {
        
        current_angle = prev_angle;
      }
      
      else
      {
        prev_angle = current_angle;
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
