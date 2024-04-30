#include <robot.h>
#include <Chassis.h>
#include <ir_codes.h>
#include <event_timer.h>

// TODO: Choose your pins
const uint8_t LEFT_LINE_SENSOR_PIN = A0;
const uint8_t RIGHT_LINE_SENSOR_PIN = A2;

ROBOT_STATE robotState = ROBOT_IDLE;
Chassis chassis;

bool isFarAway = 1;

float targetDistance = 13;
float kPwall = 0.5; 
float prevErrorWall = 0;
float kDwall = 0;
bool isCloseLeft = false;
bool isClose = false;

float forwardEffort = 0;
float rightEffort = 0; 


EventTimer lineTimer;
EventTimer turnTimer;
bool checkLineTimer(void)
{
  return lineTimer.checkExpired();
}

bool checkTurnTimer(void){
  return turnTimer.checkExpired();
}

void idle(void)
{
    Serial.println("Idling!");
    chassis.setMotorEfforts(0, 0);
    robotState = ROBOT_IDLE;
}

String keyCodeString; // this may come in handy later
void handleKeyCode(int16_t keyCode)
{ 
    Serial.println(keyCode);

  if(keyCode == ENTER_SAVE) idle();

  switch(keyCode)
  {
    case UP_ARROW:
      chassis.setMotorTargetSpeeds(10, 10);
      robotState = ROBOT_DRIVING;
      break;
    case RIGHT_ARROW:
      chassis.setMotorTargetSpeeds(10, -10);
      robotState = ROBOT_RIGHT;
      break;
    case DOWN_ARROW:
      chassis.setMotorTargetSpeeds(-10, -10);
      robotState = ROBOT_DRIVING;
      break;
    case LEFT_ARROW:
      chassis.setMotorTargetSpeeds(-10, 10);
      robotState = ROBOT_LEFT;
      break;
    case NUM_1: 
      robotState = ROBOT_STANDOFF;
      chassis.setWheelTargetSpeeds(2, 2);
      break;
    case NUM_2:
      robotState = ROBOT_STANDOFF_PROP;
      break;
    case NUM_3: 
      robotState = ROBOT_WALLING;
    default:
      break;
  }
}

void handleNewDistanceReading(float distanceReading)
{
    //Serial.println(distanceReading);
    if(robotState == ROBOT_STANDOFF_PROP){
      float Kp = 1;
      float error = 40 - distanceReading;
      forwardEffort = error * Kp;
    }

    if(robotState == ROBOT_STANDOFF)
    {

      // hysteresis
      if (distanceReading <= 25 && isFarAway){
        //when distance is less than 25 and moving forward, getting closer ==> goal = move backward
        chassis.setWheelTargetSpeeds(-10, -10);
        isFarAway = 0; //we are now moving forward
        }
      
        
      if (distanceReading >= 40 && !isFarAway){
        // getting farther, therfore move forward to maintain 40cm 
        chassis.setWheelTargetSpeeds(2, 2);
        isFarAway = 1; //we are now far
        }

    }

    else if(robotState == ROBOT_WALLING)
    {
      Serial.println(distanceReading);
      Serial.println(isClose);
      Serial.println(robotState);
      Serial.println();

     //hyseterisu 
      if(distanceReading <= 10 && !isClose){
        robotState = ROBOT_LEFT;
        chassis.setWheelTargetSpeeds(-10,10);
        turnTimer.start(1150);
        isClose = true;
        Serial.println("walling --> left");
        return;
      }

      if(distanceReading >= 12 && isClose){
        isClose = false; 
        //robotState = ROBOT_WALLING; // its already walling
        return;
      }
  }
}

void handleTurnTimer(void){
  if(robotState == ROBOT_LEFT){
    Serial.println("handling turn timer");
    chassis.setWheelTargetSpeeds(0,0);
    robotState = ROBOT_WALLING;
  }
}

void handleNewDistanceReadingLeft(float distanceReadingLeft){
  if(robotState == ROBOT_WALLING){

    float errorWall = targetDistance - distanceReadingLeft; 
    float effortWall = errorWall * kPwall - (errorWall - prevErrorWall) * kDwall; 
    float leftSpeed = -10 - effortWall;
    float rightSpeed = -10 + effortWall; 

    chassis.setWheelTargetSpeeds(leftSpeed, rightSpeed);

      Serial.print(distanceReadingLeft);
      Serial.print("        ");
      Serial.print(errorWall);
      Serial.print("        ");
      Serial.print(effortWall);
      Serial.print("        ");
      Serial.print(leftSpeed);
      Serial.print("        ");
      Serial.print(rightSpeed);
      Serial.println("        ");

    prevErrorWall = errorWall;

  }
  if(robotState == ROBOT_LEFT){
    return;
  }
}

void handleLineTimer(void)
{
  if(robotState == ROBOT_LINING)
  {
      // TODO: execute line following

      // don't forget to restart the timer
      lineTimer.restart();
  }
}

bool checkIntersection(void)
{
  // TODO: check for an intersection
  return false;
}

void handleIntersection(void)
{
  // TODO: handle intersection
}

bool checkBatteryTimer(void)
{
  return false;
}

void handleBatteryTimer(void)
{
  
}