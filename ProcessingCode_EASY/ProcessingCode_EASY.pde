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
// easy

import processing.serial.*;
import processing.sound.*;

Serial myPort;        // The serial port

SoundFile file;
SoundFile collisionSoundx;
SoundFile collisionSoundy;
//*****************************************************************
//****************** Initialize Variables (START) *****************
//*****************************************************************
boolean showBall = true;

// BALL STUFF FOR X
// current ball pos, vel
float ballpos_x_current = 300;
float ballvel_x_current = 0;
// prev ball pos, vel

float ballpos_x_prev = 300;
float ballvel_x_prev = 0;

// BALL STUFF FOR Y
// current ball pos, vel
float ballpos_y_current = 300;
float ballvel_y_current = 0;
// prev ball pos, vel
float ballpos_y_prev = 300;
float ballvel_y_prev = 0;

// Vel Magnitude
float ballvelmag = 0;
float minThresh = 1.0;

// getting values from serial
int lf = 10;
String myString = null;

// mapping stuff
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

// physics stuff
float wall_slope_x;
float wall_slope_y;
float ball_mass = 0.02; // kg?? idk help
float ballacc_x;
float ballacc_y;
float g = 9.81; // m/s2

boolean x_collision = false;
boolean y_collision = false;

boolean x_prev_collision = false;
boolean y_prev_collision = false;


float x_coll_string; 
float y_coll_string;

int impulse_count_x = 0;
int impulse_count_y = 0;
int impulse_size = 500;

//Audio stuff
boolean play = false;
boolean wasPlaying = false;
float speed = 1.0;
float volume_x = 0.0;
float volume_y = 0.0;


class Wall {
  float x, y, w, h;
  float wallHeight = 10;
  Wall (float x, float y, float w, float h) {
    this.x = x;
    this.y = y;
    this.w = w;
    this.h = h;
  }

  void display() {

    rect(x,y,w,h);

  }

  void display3D() {
    pushMatrix();
    translate(x + w/2, y + h/2, -wallHeight/2);
    fill(200);
    box(w, h, wallHeight); // 3D wall
    popMatrix();
  }

    boolean isXColliding(float ballX, float ballY, float radius) {
      return (ballX + radius >= x && ballX - radius <= x + w) && (ballY + radius >= y && ballY - radius <= y + h);
    }
    boolean isYColliding(float ballX, float ballY, float radius){
      return (ballY + radius >= y && ballY - radius <= y + h) && (ballX + radius >= x && ballX - radius <= x + w);

  }
  // old bad code
  //boolean isColliding(float ballX, float ballY, float radius){
  //  return ballX + radius >= x && ballX - radius <= x + w &&  
  //  ballY + radius >= y && ballY - radius <= y + h;
  //}
}

// maze stuff
ArrayList<Wall> wallsList = new ArrayList<Wall>();



//*****************************************************************
//******************* Initialize Variables (END) ******************
//*****************************************************************


void setup () {
  // set the window size:
  // size - 3D
  //size(600, 600, P3D);
  // size - 2D
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
  lights();
  //strokeWeight(4);
  //fill(255, 10, 10);
  //triangle(50 + 7 * 90 - 45, 65 + 30, 50 + 7 * 90 -30, 65 + 60, 50 + 7 * 90 - 60, 65 + 60);
  //fill(10, 255, 10);
  //triangle(50 + 90 - 45, 65 + 7 * 90 + 30, 50 + 90 - 30, 65 + 7 * 90 + 60, 50 + 90 - 60, 65 + 7 * 90 + 60);
 

  // maze stuff - 2D
  // creating the maze

  wallsList.add(new Wall(150,450,300,2)); // bot
  wallsList.add(new Wall(150, 150, 300, 2)); // top
  wallsList.add(new Wall(150, 150, 2, 300)); // left
  wallsList.add(new Wall(450, 150, 2, 300)); // right
  
  // inside maze walls (simple maze)
  wallsList.add(new Wall(210, 210, 2, 240)); // 1
  wallsList.add(new Wall(270, 390, 2, 60));  // 2
  wallsList.add(new Wall(270, 390, 120, 2)); // 3
  wallsList.add(new Wall(270, 330, 120, 2)); // 4
  wallsList.add(new Wall(270, 210, 2, 120)); // 5
  wallsList.add(new Wall(270, 210, 120, 2)); // 6
  wallsList.add(new Wall(330, 270, 120, 2)); // 7
  
 
  //wallsList.add(new Wall(50, 65, 1350, 2)); // bot
  //wallsList.add(new Wall(50, 785, 1350, 2)); // top
  //wallsList.add(new Wall(50, 65, 2, 720)); // left
  //wallsList.add(new Wall(1400, 65, 2, 720)); // right
  
  //// inside maze walls (simple maze) // mid width = 725, mid height = 425
  //wallsList.add(new Wall(50 + 90, 65 + 90, 2, 90)); // 1
  //wallsList.add(new Wall(50 + 90, 65 + 180, 450, 2));  // 2
  //wallsList.add(new Wall(50, 270 + 65, 450, 2)); // 3
  //wallsList.add(new Wall(50 + 90, 360 + 65, 90*3, 2)); // 4
  //wallsList.add(new Wall(50 + 90, 360 + 65, 2, 180)); // 4
  //wallsList.add(new Wall(50 + 90, 90*6 + 65, 90, 2)); // 4
  //wallsList.add(new Wall(50 + 90, 65 + 90, 90, 2)); // 4
  //wallsList.add(new Wall(50 + 90*3, 65, 2, 90)); // 4
  //wallsList.add(new Wall(50 + 90*5, 65, 2, 90)); // 4
  //wallsList.add(new Wall(50 + 90*7, 65, 2, 90*3)); // 4
  //wallsList.add(new Wall(50 + 90*4, 65 + 90, 2, 90)); // 4
  //wallsList.add(new Wall(50 + 90*4, 65 + 90*4, 2, 90)); // 4
  //wallsList.add(new Wall(50 + 90*4, 65 + 90*6, 2, 90*2)); // 4
  //wallsList.add(new Wall(50 + 90*2, 65 + 90*5, 90*2, 2)); // 4
  //wallsList.add(new Wall(50 + 90, 65 + 90*6, 90*3, 2 )); // 4
  //wallsList.add(new Wall(50, 65 + 90*7, 90*3, 2)); // 4
  //wallsList.add(new Wall(50 + 90*4, 65 + 90*6, 2, 90*2)); // 4
  //wallsList.add(new Wall(50 + 90*6, 65 + 90, 2, 90*2)); // 4
  //wallsList.add(new Wall(50 + 90*6, 65 + 90*3, 90*2, 2)); // 4
  //wallsList.add(new Wall(50 + 90*7, 65, 2, 90*3)); // 4
  //wallsList.add(new Wall(50 + 90*8, 65+90, 2, 90*1)); // 4
  //wallsList.add(new Wall(50 + 90*8, 65 + 90*2, 90*6, 2)); // 4
  //wallsList.add(new Wall(50 + 90*9, 65 + 90, 90*6, 2)); // 4
  //wallsList.add(new Wall(50 + 90*5, 65 + 90*3, 2, 90*4)); // 4
  //wallsList.add(new Wall(50 + 90*5, 65 + 90*7, 90, 2)); // 4
  //wallsList.add(new Wall(50 + 90*5, 65 + 90*8, 90, 2)); // 4
  //wallsList.add(new Wall(50 + 90*6, 65 + 90*4, 2, 90)); // 4
  //wallsList.add(new Wall(50 + 90*6, 65 + 90*4, 90*3, 2)); // 4
  //wallsList.add(new Wall(50 + 90*6, 65 + 90*5, 90, 2 )); // 4
  //wallsList.add(new Wall(50 + 90*6, 65 + 90*6, 90*2, 2 )); // 4
  //wallsList.add(new Wall(50 + 90*6, 65 + 90*6, 2, 90 )); // 4
  //wallsList.add(new Wall(50 + 90*8, 65 + 90*5, 2, 90 )); // 4
  //wallsList.add(new Wall(50 + 90*8, 65 + 90*5, 90*4, 2 )); // 4
  //wallsList.add(new Wall(50 + 90*10, 65 + 90*4, 2, 90*3 )); // 4
  //wallsList.add(new Wall(50 + 90*10, 65 + 90*4, 90, 2 )); // 4
  //wallsList.add(new Wall(50 + 90*11, 65 + 90*3, 2, 90 )); // 4
  //wallsList.add(new Wall(50 + 90*12, 65 + 90*2, 2, 90*5 )); // 4
  //wallsList.add(new Wall(50 + 90*13, 65 + 90*3, 90, 2 ));
  //wallsList.add(new Wall(50 + 90*13, 65 + 90*3, 2, 90 ));
  //wallsList.add(new Wall(50 + 90*13, 65 + 90*4, 90*2, 2 ));
  //wallsList.add(new Wall(50 + 90*13, 65 + 90*5, 2, 90*3 ));
  //wallsList.add(new Wall(50 + 90*13, 65 + 90*4, 90, 2 ));
  //wallsList.add(new Wall(50 + 90*14, 65 + 90*6, 2, 90 ));
  //wallsList.add(new Wall(50 + 90*14, 65 + 90*7, 90, 2 ));
  //wallsList.add(new Wall(50 + 90*11, 65 + 90*6, 2, 90*2 ));
  //wallsList.add(new Wall(50 + 90*9, 65 + 90*6, 2, 90*2 ));
  //wallsList.add(new Wall(50 + 90*7, 65 + 90*7, 90*2, 2 ));
  //wallsList.add(new Wall(50 + 90*9, 65 + 90*2, 2, 90*2 ));
  //wallsList.add(new Wall(50 + 90*9, 65 + 90*3, 90, 2 ));
  

  // maze stuff - 3D
  // outer walls
  //wallsList.add(new Wall(150, 450, 300, wall_thick));   // bottom
  //wallsList.add(new Wall(150, 150 - wall_thick, 300, wall_thick)); // top
  //wallsList.add(new Wall(150 - wall_thick, 150, wall_thick, 300)); // left
  //wallsList.add(new Wall(450 - wall_thick, 150, wall_thick, 300)); // right

  //// inner walls
  //wallsList.add(new Wall(210 - wall_thick/2, 210, wall_thick, 240)); // wall 1
  //wallsList.add(new Wall(270 - wall_thick/2, 390, wall_thick, 60));  // wall 2
  //wallsList.add(new Wall(270, 390 - wall_thick/2, 120, wall_thick)); // wall 3
  //wallsList.add(new Wall(270, 330 - wall_thick/2, 120, wall_thick)); // wall 4
  //wallsList.add(new Wall(270 - wall_thick/2, 210, wall_thick, 120)); // wall 5
  //wallsList.add(new Wall(270, 210 - wall_thick/2, 120, wall_thick)); // wall 6
  //wallsList.add(new Wall(330, 270 - wall_thick/2, 120, wall_thick)); // wall 7

  file = new SoundFile(this, "Loop_attempt.mp3");
  collisionSoundx = new SoundFile(this, "collision.wav");
  collisionSoundy = new SoundFile(this, "collision.wav");
}

void draw () {
  
  // everything happens in the serialEvent()
  background(0); //uncomment if you want to control a ball
  lights();
  stroke(127, 34, 255);     //stroke color
  //strokeWeight(4);        //stroke wider
  
  //fill(255, 10, 10);
  //triangle(50 + 7 * 90 - 45, 65 + 30, 50 + 7 * 90 -30, 65 + 60, 50 + 7 * 90 - 60, 65 + 60);
  //fill(10, 255, 10);
  //triangle(50 + 90 - 45, 65 + 7 * 90 + 30, 50 + 90 - 30, 65 + 7 * 90 + 60, 50 + 90 - 60, 65 + 7 * 90 + 60);
 

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
      if (wall.isXColliding(ballpos_x_current, ballpos_y_prev, radius)) {
        //ballvel_x_current = 0;
      
        ballvel_x_current = -0.3*ballvel_x_current;
        ballpos_x_current = ballpos_x_prev; // stay in place
        x_collision = true; 
        if(abs(ballvel_x_current) > 1) {
          volume_x = map(abs(ballvel_x_current), 0, 40, 0, 1);
          println(volume_x);
          collisionSoundx.amp(volume_x);
          collisionSoundx.play();
        }
        break;
      } 
    }
    
  for (Wall wall : wallsList) {
      if (wall.isYColliding(ballpos_x_current, ballpos_y_current, radius)) {
        //ballvel_y_current = 0;
        ballvel_y_current = -0.3*ballvel_y_current;
        ballpos_y_current = ballpos_y_prev; // stay in place  
        y_collision = true;
        if(abs(ballvel_y_current) > 1) {
          volume_y = map(abs(ballvel_y_current), 0, 40, 0, 1);
          collisionSoundy.amp(volume_y);
          collisionSoundy.play();
        }
        break;
      } 
      
    }
    
    ballvel_x_prev = ballvel_x_current;
    ballpos_x_prev = ballpos_x_current;
    
    ballvel_y_prev = ballvel_y_current;
    ballpos_y_prev = ballpos_y_current;
    

  //line(x1_x, y1_x, x2_x, y2_x); // horizontal tilt
  //line(x1_y, y1_y, x2_y, y2_y); // horizontal tilt

  // ball - 2D
  if (showBall == true){
    strokeWeight(0); 
    fill(255, 255, 57);
  ellipse(ballpos_x_current, ballpos_y_current, radius*2, radius*2);

  }
  // ball - 3D
  //pushMatrix();
  //translate(width/2, height/2, 0);
  //rotateX(-current_angle_y);
  //rotateY(current_angle_x);
  //translate(-width/2, -height/2, 0);
  
   for (Wall wall : wallsList) {
      //wall.display3D();
      strokeWeight(0); 
      fill(120, 120, 255);
       wall.display();
    }
    
   // ellipse(150,450,radius ,radius);
    
     //ball - 3D
    //fill(255,0,0);
    //noStroke();
    //pushMatrix();
    //translate(ballpos_x_current, ballpos_y_current, -radius);
    //sphere(radius);
    //popMatrix();
    
    //popMatrix();
    
    if (x_collision && (!x_prev_collision))
    {
       x_coll_string = 1;
       impulse_count_x ++; //= impulse_count_x +1; 
    }

    
    if (y_collision && (!y_prev_collision))
    {
       y_coll_string = 1;
       impulse_count_y ++; 
    }
    
    if (x_collision && impulse_count_x <= impulse_size)
    {
    x_prev_collision = false;
    }
    else
    {
      x_prev_collision = x_collision;
    }
    
    if (y_collision && impulse_count_y <= impulse_size)
    {
    y_prev_collision = false; 
    }
    else
    {
      y_prev_collision = y_collision;
    }
    
    // audio mag calcs
    ballvelmag = sqrt(sq(ballvel_x_current)+sq(ballvel_y_current));
    
    // sending 2 arduino
   String ballData = nf(ballpos_x_current, 0, 2) + "," + // x pos 
                  nf(ballpos_y_current, 0, 2) +  "," + // y pos
                  nf(x_coll_string,0,2) + "," + // x coll
                  nf(y_coll_string,0,2) + "," + // y coll
                  nf(ballvelmag,0,2 )+ "\n"; // magnitude
                  
                    
   myPort.write(ballData);

   //print(ballData);
   
   
   x_coll_string = 0;
   y_coll_string = 0;
   x_collision = false;
   y_collision = false;
  

  // Audio
  
  if (ballvelmag > minThresh) {
    play = true;
  } else {
    play = false;
  }

  if (play && !wasPlaying) {
    file.loop();
    wasPlaying = true;
  } else if (!play && wasPlaying) {
    file.pause();
    wasPlaying = false;
  }

  if (play) {
    // Example: modulate from 0.5x to 2x speed depending on count value
    speed = map(ballvelmag, minThresh, 50, 0.5, 2.0);
    file.rate(speed);
  }

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
        
        //String ardData = nf(current_angle_x, 0, 2) + "," + 
        //            nf(current_angle_y, 0, 2) + "\n";

        // for debugging arduino commands!
          float ard_1 = float(values[2]);
          
          //float ard_2 = float(values[3]);
          
          String ardData = nf(ard_1, 0, 2) + "\n" ;//+ 
                  //  nf(ard_2, 0, 2) + "\n";
                    
          print(ardData);
          
          //String ard_1 = values[2];
          //print(current_angle_x);
          //print(",");
          //print(current_angle_y);
          //print(",");
          //print(ard_1);
          //print("\n");
        
        
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

void keyPressed()
{
  if (key == TAB)
  {
    showBall = !showBall;
  }
}
