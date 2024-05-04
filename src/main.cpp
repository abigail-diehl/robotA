#include <Arduino.h>
#include <robot.h>
#include <ir_codes.h>
#include <IRdecoder.h>
#include <Sharp-IR.h>
#include <LSM6.h>


/**
 * IR remote decoder.
*/
#define IR_PIN 17 //TODO: Choose your pin
IRDecoder decoder(IR_PIN);

/**
 * HC-SR04
*/
#include <HC-SR04.h>
HC_SR04 hc_sr04(11, 4); // TODO: Choose your pins (echo,trig)
//HC_SR04 hc_sr04_left(3,2);     //echo, trig

SharpIR sharpIR(20); 

/*
    imu!
*/
LSM6 imu;
float pitchAngle = 0;

bool turning = false;

void ISR_HC_SR04(void)
{
    hc_sr04.ISR_echo();
    //hc_sr04_left.ISR_echo();
}

void setup() 
{
    Serial.begin(115200);
    delay(500);
    Serial.println("setup()");

    chassis.init();

    decoder.init();

    imu.init();
    imu.setFullScaleGyro(imu.GYRO_FS1000);

    hc_sr04.init(ISR_HC_SR04);
    //hc_sr04_left.init(ISR_HC_SR04);

    sharpIR.init();

    Serial.println("/setup()");
}

void loop() 
{
    /**
     * Chassis::loop() returns true when the motor control loop fires. We can use that timer to trigger
     * any number of processes that we want to run on the same schedule.
    */
    if(chassis.loop())
    {
        // do stuff here that is synchronized with the motor controller
        updatePose();
        //handleCounterWeight();
    }

    /**
     * But we can also process asynchronous events, such as IR remote presses or distance sensor readings.
    */
    int16_t keyCode = decoder.getKeyCode();
    if(keyCode != -1) handleKeyCode(keyCode);

    /** Check the distance sensor.
     * We return true only if there is a new reading, which is passed by reference.
     * 
     * Note that the construct is still checker/handler: if there is a new reading, handle it
     */
    
    float distanceReadingForward = 0;
    float prevDistanceReadingForward = 0;
    float avgDistanceReadingForward = ( distanceReadingForward + prevDistanceReadingForward ) / 2;

    bool hasNewReadingForward = hc_sr04.getDistance(avgDistanceReadingForward);
    if(hasNewReadingForward && avgDistanceReadingForward < 15){
        handleNewDistanceReading(avgDistanceReadingForward);

        prevDistanceReadingForward = distanceReadingForward; 
    }

    float distanceReadingLeft = 0; 
    bool hasNewReadingLeft = sharpIR.getDistance(distanceReadingLeft);
    if (hasNewReadingLeft && distanceReadingLeft < 15){
        //Serial.println(distanceReadingLeft);
        handleNewDistanceReadingLeft(distanceReadingLeft);
    } 

    /**
     * Line sensors. We do two things here:
     *  - handle the line timer, which schedule line following updates
     *  - check for intersections
     * 
     * We could combine the two, but it's cleaner this way with the checker/handlers.
     * An extra couple ADC reads is hardly a problem.
    */
     if(imu.checkForPitchUpdate(pitchAngle)) handlePitchUpdate(pitchAngle);

    if(checkTurnTimer()) handleTurnTimer();

    //if(checkEscapeTimer() handleEscapeTimer);

    if(checkBatteryTimer()) handleBatteryTimer();

    if(checkCounterWeightTimer()) handleCounterWeightTimer();
}