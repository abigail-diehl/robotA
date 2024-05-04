#pragma once

#include <Arduino.h>
#include <Chassis.h>

enum ROBOT_STATE
{   
    ROBOT_IDLE, 
    ROBOT_LEFT,
    ROBOT_RIGHT,
    ROBOT_DRIVING,
    ROBOT_STANDOFF, 
    ROBOT_LINING,
    ROBOT_WALLING,
    ROBOT_STANDOFF_PROP,
    ROBOT_COUNTERWEIGHT,
};

struct pose{
  float xPose = 0;
  float yPose = 0;
  float direction = 0;
};


enum HEADING{
  NORTH,
  EAST,
  SOUTH,
  WEST
};

void handleKeyCode(int16_t keyCode);

void handleNewDistanceReading(float distanceReadingForward);
void handleNewDistanceReadingLeft(float distanceReadingLeft);


bool checkTurnTimer(void);
void handleTurnTimer(void);

bool checkCounterWeightTimer(void);
void handleCounterWeightTimer(void);

bool checkEscapeTimer(void);
void handleEscapeTimer(void);

bool checkBatteryTimer(void);
void handleBatteryTimer(void);

void handlePitchUpdate(float& pitch);

void updatePose();

void handleCounterWeight(void);