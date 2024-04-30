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
    ROBOT_STANDOFF_PROP
};

void handleKeyCode(int16_t keyCode);

void handleNewDistanceReading(float distanceReadingForward);
void handleNewDistanceReadingLeft(float distanceReadingLeft);

bool checkLineTimer(void);
void handleLineTimer(void);

bool checkTurnTimer(void);
void handleTurnTimer(void);

bool checkIntersection(void);
void handleIntersection(void);

bool checkBatteryTimer(void);
void handleBatteryTimer(void);