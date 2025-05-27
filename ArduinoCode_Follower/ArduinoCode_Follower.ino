//--------------------------------------------------------------------------
// Code to test basic Hapkit functionality (sensing and force output)
// Updated by Allison Okamura 1.17.2018, Nathan Kau 9.9.2022
//--------------------------------------------------------------------------

/* STUDENT MUST READ
 *  
 * 1. Define the sensor option below appropriate for your hapkit
 * 2. Set the baudrate of your serial monitor to 115200
 * 
 */
 float rp = 0.5/100.0; // m
    float rs = 7.5/100; // m
    float rh = 8.0/100; // m
    float j = rh*rp/rs;
    float ts; 
    float Tp;
    float xh;
    float force;

//#define HALL_EFFECT_SENSOR
#define MR_SENSOR

//#define BILATERAL
//#define SCALED_BILATERAL

// Includes
#include <math.h>
#include "helpers.h"

#define BAUDRATE 9600

/*
 * calculate_handle_position
 * 
 * Calculates the position of the hapkit handle given the sensor reading
 * 
 * Args:
 *  updated_position: Position reading from sensor
 * Return:
 *  Handle position in meters
 */
float calculate_handle_position(float updated_position) {
  // STUDENT CODE HERE
  ts = 0.0153 * updated_position - 12;
  xh = rh * ts*M_PI/180;
  return xh;
}

/*
 * calculate_pulley_torque
 * 
 * Calculates the pulley torque necessary to achieve the desired force
 * 
 * Args:
 *  force: Desired force in Newtons
 * Return:
 *  Motor torque (in Newton-meters) to achieve desired force.
 */
float calculate_pulley_torque(float force) {
  // STUDENT CODE HERE
  Tp = -j*(force);
  return 0.0;
}

/*
 * student_specified_force
 * 
 * Specifies a desired force to render on the hapkit
 * 
 * Args:
 *  handle_position: Handle position in meters
 *  handle_velocity: Approximate handle velocity in meters/second
 * Return:
 *  Desired force in Newtons
 */
float student_specified_force(float handle_position, float handle_velocity, float handle_position_remote) {
  // STUDENT CODE HERE

  #ifdef BILATERAL
  #endif

  #ifdef SCALED_BILATERAL
  #endif
  
  return 0.0;
}

void setup() 
{
  Serial.begin(BAUDRATE);

  initialize_sensor();
  initialize_motor();
  initialize_loop_checker();
}

int count = 0;
void loop()
{
  // Compute position in counts
  float updated_position = read_sensor();
  ts = 0.0153 * updated_position - 4;

  Serial.println(ts,2);


  delay(100);
  // Compute position [m] and velocity [m/s]
  //float handle_position = calculate_handle_position(updated_position);
  //float smoothed_velocity = calculate_smoothed_velocity(handle_position, /*DT=*/0.001);
  
  // Send handle position to remote arduino, receive handle position from remote arduino
  // These messages are sent over Serial (USB cable), so do not use Serial.print in your
  // program or you will interfere with the communication.
  //float remote_position = send_receive_remote_arduino(handle_position);
  
  // Assign a motor output force [N]
  // student_specified_force now gets access to the handle position of the remote hapkit
  //float force = student_specified_force(handle_position, smoothed_velocity, remote_position);
  //float pulley_torque = calculate_pulley_torque(force);
  
  // Command the motor
  //command_motor(pulley_torque);

  // Check if your loop speed is too slow, and if so, print an error message.
  //check_loop_speed();
}
