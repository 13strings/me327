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
// BALL STUFF FOR X
// current ball pos, vel
float ballpos_x_current;
float ballvel_x_current = 0;
// prev ball pos, vel
float ballpos_x_prev = width/2;
float ballvel_x_prev = 0;

// BALL STUFF FOR Y
// current ball pos, vel
float ballpos_y_current;
float ballvel_y_current = 0;
// prev ball pos, vel
float ballpos_y_prev = width/2;
float ballvel_y_prev = 0;

// getting values from serial
int lf = 10; 
String myString = null;

// mapping shit
float minValueIn = -45;
float maxValueIn = 45;
float minValueOut = 0;
float maxValueOut = 800;
float current_angle_x;
float current_angle_y;
float prev_angle_x;
float prev_angle_y;
float radius = 10;
float wall_thick = 2;

// physics shit
float wall_slope_x;
float wall_slope_y;
float ball_mass = 0.02; // kg?? idk help
float ballacc_x;
float ballacc_y;
float g = 9.81; // m/s2

class Wall {
  float x,y,w,h;
  Wall (float x, float y, float w, float h) {
    this.x = x;
    this.y = y;
    this.w = w;
    this.h = h;
  }
  
  void display() {
    rect(x,y,w, h);
  }
    boolean isColliding(float ballX, float ballY, float radius) {
    return ballX + radius >= x && ballX - radius <= x + w &&
           ballY + radius >= y && ballY - radius <= y + h;
  }
}

// maze shit
ArrayList<Wall> wallsList = new ArrayList<Wall>();

// coms with collisions
 boolean collisionXSent = false;
 boolean collisionYSent = false;

//*****************************************************************
//******************* Initialize Variables (END) ******************
//*****************************************************************


void setup () {
  // set the window size:
  size(600, 600);
  ballpos_x_current = 300;
  ballpos_y_current = 300;
  ballpos_x_prev = 300;
  ballpos_y_prev = 300;
  // List all the available serial ports
  //println(Serial.list());
  // Check the listed serial ports in your machine
  // and use the correct index number in Serial.list()[].

  myPort = new Serial(this, Serial.list()[0], 115200);  //make sure baud rate matches Arduino

  // A serialEvent() is generated when a newline character is received :
  myPort.bufferUntil('\n');
  background(0);      // set inital background:
  
  
  // maze shit 
  //// creating the maze
  //wallsList.add(new Wall(150,450,300,0)); // bot
  //wallsList.add(new Wall(150, 150, 300, 0)); // top
  //wallsList.add(new Wall(150, 150, 0, 300)); // left
  //wallsList.add(new Wall(450, 150, 0, 300)); // right
  
  //// inside maze walls (simple maze)
  //wallsList.add(new Wall(210, 210, 0, 240)); // 1
  //wallsList.add(new Wall(270, 390, 0, 60));  // 2
  //wallsList.add(new Wall(270, 390, 120, 0)); // 3
  //wallsList.add(new Wall(270, 330, 120, 0)); // 4
  //wallsList.add(new Wall(270, 210, 0, 120)); // 5
  //wallsList.add(new Wall(270, 210, 120, 0)); // 6
  //wallsList.add(new Wall(330, 270, 120, 0)); // 7
  
    // outer walls
  wallsList.add(new Wall(150, 450, 300, wall_thick));   // bottom
  wallsList.add(new Wall(150, 150 - wall_thick, 300, wall_thick)); // top
  wallsList.add(new Wall(150 - wall_thick, 150, wall_thick, 300)); // left
  wallsList.add(new Wall(450 - wall_thick, 150, wall_thick, 300)); // right
  
  // inner walls 
  wallsList.add(new Wall(210 - wall_thick/2, 210, wall_thick, 240)); // wall 1
  wallsList.add(new Wall(270 - wall_thick/2, 390, wall_thick, 60));  // wall 2
  wallsList.add(new Wall(270, 390 - wall_thick/2, 120, wall_thick)); // wall 3
  wallsList.add(new Wall(270, 330 - wall_thick/2, 120, wall_thick)); // wall 4
  wallsList.add(new Wall(270 - wall_thick/2, 210, wall_thick, 120)); // wall 5
  wallsList.add(new Wall(270, 210 - wall_thick/2, 120, wall_thick)); // wall 6
  wallsList.add(new Wall(330, 270 - wall_thick/2, 120, wall_thick)); // wall 7
    
 
  
}

void draw () {
  // everything happens in the serialEvent()
  background(0); //uncomment if you want to control a ball
  stroke(127, 34, 255);     //stroke color
  strokeWeight(4);        //stroke wider
  
  //*****************************************************************
  //***************** Draw Objects in Scene (START) *****************
  //*****************************************************************

  //ellipse(300, 300, radius*2, radius*2);
 
   float dt = 1.0/frameRate *5;
   
   // solving for the horizontal tilt-direction
   wall_slope_x = tan(current_angle_x);
   //float len_platform = 600;
   //float x1_x = width/2 - len_platform/2;
   //float x2_x = width/2 + len_platform/2;
   //float y1_x = -wall_slope_x * len_platform/2 + height/2;
   //float y2_x = wall_slope_x * len_platform/2 + height/2;
    
    ballacc_x = g*sin(current_angle_x);
    
    ballvel_x_current = ballvel_x_prev + ballacc_x * dt;
    ballpos_x_current = ballpos_x_prev + (ballvel_x_prev + ballvel_x_current)/2 * dt;
     

    
    // solving for vertical tilt-direction
    wall_slope_y = 1/tan(current_angle_y);
    
   //float x1_y = width/2 - len_platform/2;
   //float x2_y = width/2 + len_platform/2;
   //float y1_y = -wall_slope_y * len_platform/2 + height/2;
   //float y2_y = wall_slope_y * len_platform/2 + height/2;
    
    ballacc_y = g*sin(current_angle_y);
    
    ballvel_y_current = ballvel_y_prev + ballacc_y * dt;
    ballpos_y_current = ballpos_y_prev + (ballvel_y_prev + ballvel_y_current)/2 * dt;
    
    
    for (Wall wall : wallsList) {
      if (wall.isColliding(ballpos_x_current, ballpos_y_prev, radius)) {
        ballvel_x_current = 0;
        ballpos_x_current = ballpos_x_prev; // stay in place
                     
      } 
      
      
      if (wall.isColliding(ballpos_x_current, ballpos_y_current, radius)) {
        ballvel_y_current = 0;
        ballpos_y_current = ballpos_y_prev; // stay in place  
        
      } 
      
    }
    
    ballvel_x_prev = ballvel_x_current;
    ballpos_x_prev = ballpos_x_current;
    
    ballvel_y_prev = ballvel_y_current;
    ballpos_y_prev = ballpos_y_current;

     //line(x1_x, y1_x, x2_x, y2_x); // horizontal tilt
    //line(x1_y, y1_y, x2_y, y2_y); // horizontal tilt
    
   
    ellipse(ballpos_x_current, ballpos_y_current, radius*2, radius*2);
   
   for (Wall wall : wallsList) {
      wall.display();
    }
    
   String ballData = nf(ballpos_x_current, 0, 2) + "," + 
                  nf(ballpos_y_current, 0, 2) + "\n";

   myPort.write(ballData);
   //print(ballData);
    
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
        
        current_angle_x = float(values[0]);
        current_angle_x = radians(current_angle_x);
        
        current_angle_y = float(values[1]);
        current_angle_y = radians(current_angle_y);
        

        // for debugging arduino commands!
          float ard_1 = float(values[2]);
          //float ard_2 = float(values[3]);
          
          String ardData = nf(ard_1, 0, 2) + "\n";// + "," + 
          //          nf(ard_2, 0, 2) + "\n";
          print(ardData);
        
        
      }

      if (Float.isNaN(current_angle_x) ||Float.isNaN(current_angle_y) )
      {
        
        current_angle_x = prev_angle_x;
        current_angle_y = prev_angle_y;
      }
      
      else
      {
        prev_angle_x = current_angle_x;
        prev_angle_y = current_angle_y;
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
