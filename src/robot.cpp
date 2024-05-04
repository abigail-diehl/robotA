#include <robot.h>
#include <Chassis.h>
#include <ir_codes.h>
#include <event_timer.h>

// TODO: Choose your pins
// const uint8_t LEFT_LINE_SENSOR_PIN = A0;
// const uint8_t RIGHT_LINE_SENSOR_PIN = A2;

ROBOT_STATE robotState = ROBOT_IDLE;
Chassis chassis;

bool isFarAway = 1;

float targetDistance = 13;
float kPwall = 1; 
float prevErrorWall = 0;
float kDwall = 0.5;
bool isCloseLeft = false;
bool isClose = false;
// float Ki = 0;
// float sumError = 0;

float forwardEffort = 0;
float rightEffort = 0; 

bool onRamp = false;
int romiHeading = NORTH;
int xCoord = 0;
int yCoord = 0;

pose romiPose;
pose initialPose;


EventTimer lineTimer;
EventTimer turnTimer;
EventTimer escapeTimer;
EventTimer counterWeightTimer;

bool checkLineTimer(void)
{
  return lineTimer.checkExpired();
}

bool checkTurnTimer(void){
  return turnTimer.checkExpired();
}

bool checkEscapeTimer(void){
  return escapeTimer.checkExpired();
}

bool checkCounterWeightTimer(void){
  return counterWeightTimer.checkExpired();
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
      Serial.println("walling");
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

      // if(distanceReading > 12 && !isClose){ // nothing in front or on the side = right turn
      //   robotState = ROBOT_RIGHT;
      //   chassis.setWheelTargetSpeeds(10,-10);
      //   turnTimer.start(1150);
      //   isClose = true;
      //   Serial.println("walling --> right");
      //   return;
      // }

      // if(distanceReading < 10 && !isClose && escaping){
      //   robotState = ROBOT_RIGHT;
      //   chassis.setWheelTargetSpeeds(0,0);
      //   escapeTimer.start(3000);
        
        
      // }

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

void handleEscapeTimer(void){
  robotState = ROBOT_DRIVING;
  chassis.setWheelTargetSpeeds(-10,-10);  //drive out :)
}

void handleNewDistanceReadingLeft(float distanceReadingLeft){
  if(robotState == ROBOT_WALLING || robotState == ROBOT_COUNTERWEIGHT){

    float errorWall = targetDistance - distanceReadingLeft; 
    float effortWall = errorWall * kPwall - (errorWall - prevErrorWall) * kDwall; //IR = mech D control 
    float leftSpeed = -5 - effortWall;
    float rightSpeed = -5 + effortWall; 

    chassis.setWheelTargetSpeeds(leftSpeed, rightSpeed);

      // Serial.print(distanceReadingLeft);
      // Serial.print("        ");
      // Serial.print(errorWall);
      // Serial.print("        ");
      // Serial.print(effortWall);
      // Serial.print("        ");
      // Serial.print(leftSpeed);
      // Serial.print("        ");
      // Serial.print(rightSpeed);
      // Serial.println("        ");

    prevErrorWall = errorWall;

  }
  if(robotState == ROBOT_LEFT){
    return;
  }
}

bool checkBatteryTimer(void)
{
  return false;
}

void handleBatteryTimer(void)
{
  
}

void handlePitchUpdate(float& pitch){
  if (pitch > 5 && !onRamp){
    Serial.println("On Ramp");
    robotState = ROBOT_COUNTERWEIGHT;
    onRamp = true;
    handleCounterWeight();
    romiPose.direction = 0;
    romiPose.xPose = 0;
    romiPose.yPose = 0;
    targetDistance = 10;
  }
  if (pitch < 5 && onRamp){
    onRamp = false;
    Serial.println("off ramp");

  }
}

void updatePose(){
  float dr = 0;
  float dl = 0;
  // ICC
  float dTheta = ((dr - dl) / chassis.wheel_track) * 180 / PI; //units deg
  float dZero = (dr + dl) / 2; //units cm


  if (dl == dr){ // if we travel in a straight line
    romiPose.xPose += dZero*cos(romiPose.direction * PI / 180.0);
    romiPose.yPose += dZero*sin(romiPose.direction * PI / 180.0);
  }
  // if we do not travel in a straight line
  else{
    // calculate radius of center of curvature
    float radiusCurvature = (chassis.wheel_track / 2) * ((dr + dl) / (dr - dl));
    romiPose.xPose += radiusCurvature * (sin((romiPose.direction + dTheta) * PI / 180.0) - sin(romiPose.direction * PI / 180.0));
    romiPose.yPose += radiusCurvature * (cos(romiPose.direction * PI / 180.0) - cos((romiPose.direction + dTheta) * PI / 180.0));
    romiPose.direction += dTheta;
  }

  //constrain to -180 to 180 deg
  if (romiPose.direction > 180.0) romiPose.direction -= 360;
  if (romiPose.direction < -180.0) romiPose.direction += 360;

  // Serial.println(romiPose.xPose);
  // Serial.println(romiPose.yPose);
  // Serial.println(romiPose.direction);
}

void handleCounterWeight(){
  if (robotState == ROBOT_COUNTERWEIGHT && onRamp){
      Serial.println(romiPose.yPose);

      //dead reckon and stop
      counterWeightTimer.start(9650);
      Serial.println("counterweight");

      if(romiPose.yPose <= -47.0){
        Serial.println("Reckoning -> Stopped");
        robotState = ROBOT_IDLE;
        romiPose.direction = 0;
        romiPose.xPose = 0;
        romiPose.yPose = 0;
        chassis.setMotorEfforts(0,0);
    }
  }
  if(robotState == ROBOT_COUNTERWEIGHT && !onRamp){
    robotState = ROBOT_WALLING;
  }
}

void handleCounterWeightTimer(){
  if(robotState == ROBOT_COUNTERWEIGHT){
    chassis.setMotorTargetSpeeds(0,0);
    robotState = ROBOT_IDLE;
    Serial.println("handling counterweight timer");
  }
}