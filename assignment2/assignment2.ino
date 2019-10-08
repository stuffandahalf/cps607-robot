#include "MotorControl.h"
#include "SonarSensor.h"

/* Macros to change the execution style */
#define SERIAL_DEBUG
//#define DONT_MOVE

// constants for the speed of sound
//#define SPEED_OF_SOUND ((double)343 / 1000) /* mm/us */
//#define SPEED_OF_SOUND (.343) /* mm/us */

#define MOTOR_LA (10)
#define MOTOR_LB (9)
#define MOTOR_RA (6)
#define MOTOR_RB (5)

#define IR_SENSE_F (11)
#define IR_SENSE_R (A4)
#define IR_SENSE_L (A2)

#define DISTANCE_SENSE_TRIGGER  (A1)
#define DISTANCE_SENSE_ECHO     (A0)

#define SENSE_CLEAR     (0u)
#define SENSE_IR_FRONT  (1 << 0)
#define SENSE_IR_LEFT   (1 << 2)
#define SENSE_IR_RIGHT  (1 << 3)

#define SENSE_IR_FRONT_LEFT    (IR_STATUS_FRONT | IR_STATUS_LEFT)
#define SENSE_IR_FRONT_RIGHT   (IR_STATUS_FRONT | IR_STATUS_RIGHT)
#define SENSE_IR_OFF_TABLE     (IR_STATUS_FRONT | IR_STATUS_LEFT | IR_STATUS_RIGHT)

#ifdef SERIAL_DEBUG
#define SPEED (128)
#else
#define SPEED (64)
#endif
//#define SPEED (128)
//#define SPEED (96)

#ifdef SERIAL_DEBUG
#define PRINT_SERIAL(...) Serial.println(__VA_ARGS__);
#else
#define PRINT_SERIAL(...)
#endif

MotorControl *leftMotor;
MotorControl *rightMotor;
SonarSensor *sonarSensor;

template <typename T>
inline bool inRange(T x, T a, T b) { return a <= x && x < b; }

uint8_t getIRSensorStatus()
{
    uint8_t state = SENSE_CLEAR;
    if (digitalRead(IR_SENSE_F)) {
        state |= SENSE_IR_FRONT;
    }
    if (digitalRead(IR_SENSE_L)) {
        state |= SENSE_IR_LEFT;
    }
    if (digitalRead(IR_SENSE_R)) {
        state |= SENSE_IR_RIGHT;
    }
    return state;
}

void setup()
{
#ifdef SERIAL_DEBUG
    Serial.begin(115200);
#endif
    
    leftMotor = new MotorControl(MOTOR_LA, MOTOR_LB);
    rightMotor = new MotorControl(MOTOR_RB, MOTOR_RA);
    sonarSensor = new SonarSensor(DISTANCE_SENSE_TRIGGER, DISTANCE_SENSE_ECHO);
    
    pinMode(IR_SENSE_F, INPUT);
    pinMode(IR_SENSE_R, INPUT);
    pinMode(IR_SENSE_L, INPUT);
    
    pinMode(DISTANCE_SENSE_TRIGGER, OUTPUT);
    pinMode(DISTANCE_SENSE_ECHO, INPUT);
    
    digitalWrite(DISTANCE_SENSE_TRIGGER, LOW);
    
#ifndef DONT_MOVE
    leftMotor->forward(SPEED);
    rightMotor->forward(SPEED);
    //rightMotor->reverse(SPEED);
#else
    leftMotor->brake();
    rightMotor->brake();
#endif
}

void loop()
{
#ifdef DONT_MOVE
    return;
#endif
    
    int16_t distance = sonarSensor->getDistance();
    uint8_t irStatus = getIRSensorStatus();
    if ((irStatus & IR_SENSE_F) || inRange<long>(distance, 0, 150)) {
        rightMotor->brake();
        delay(7);
        leftMotor->brake();
    }
    else {
        rightMotor->forward(SPEED);
        leftMotor->forward(SPEED);
    }
    
    
    
    /*if (getIRSensorStatus()) {
        leftMotor->brake();
        rightMotor->brake();
    }
    else {
        leftMotor->forward(SPEED);
        rightMotor->forward(SPEED);
    }*/
    
    //PRINT_SERIAL(getDistance());
    /*uint8_t sensorStatus;
    if ((sensorStatus = getSensorStatus()) != SENSE_CLEAR) {
        
    }*/
}
