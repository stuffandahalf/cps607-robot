#include "MotorControl.h"

/* Macros to change the execution style */
#define SERIAL_DEBUG
#define DONT_MOVE
//#define BOE_BOT

// constants for the speed of sound
#define SPEED_OF_SOUND ((float)343 / 1000) /* mm/us */

#ifdef BOE_BOT
#define MOTOR_LA ()
#define MOTOR_LB ()
#define MOTOR_RA ()
#define MOTOR_RB ()

#define IR_SENSE_F ()
#define IR_SENSE_R ()
#define IR_SENSE_L ()

#define DISTANCE_SENSE_TRIGGER  ()
#define DISTANCE_SENSE_ECHO     ()
#else
#define MOTOR_LA (10)
#define MOTOR_LB (9)
#define MOTOR_RA (6)
#define MOTOR_RB (5)

#define IR_SENSE_F (A5)
#define IR_SENSE_R (A7)
#define IR_SENSE_L (A2)

#define DISTANCE_SENSE_TRIGGER  (A1)
#define DISTANCE_SENSE_ECHO     (A0)
#endif

#define SENSE_CLEAR     (0u)
#define SENSE_IR_FRONT  (1 << 0)
#define SENSE_IR_LEFT   (1 << 2)
#define SENSE_IR_RIGHT  (1 << 3)

#define SENSE_IR_FRONT_LEFT    (IR_STATUS_FRONT | IR_STATUS_LEFT)
#define SENSE_IR_FRONT_RIGHT   (IR_STATUS_FRONT | IR_STATUS_RIGHT)
#define SENSE_IR_OFF_TABLE     (IR_STATUS_FRONT | IR_STATUS_LEFT | IR_STATUS_RIGHT)

#define SPEED (64)
//#define SPEED (128)
//#define SPEED (96)

#ifdef SERIAL_DEBUG
#define PRINT_SERIAL(...) Serial.println(__VA_ARGS__);
#else
#define PRINT_SERIAL(...)
#endif

MotorControl *leftMotor;
MotorControl *rightMotor;

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

int16_t getDistance()
{
    digitalWrite(DISTANCE_SENSE_TRIGGER, HIGH);
    delayMicroseconds(10);
    digitalWrite(DISTANCE_SENSE_TRIGGER, LOW);
    
    unsigned long duration = pulseIn(DISTANCE_SENSE_ECHO, HIGH) * 1000000;
    if (!duration) {
        return -1;
    }
    
    int16_t distance = duration * SPEED_OF_SOUND; // distance is now in millimeters
    
    return distance;
}

void setup()
{
#ifdef SERIAL_DEBUG
    Serial.begin(115200);
#endif
    
    leftMotor = new MotorControl(MOTOR_LA, MOTOR_LB);
    rightMotor = new MotorControl(MOTOR_RB, MOTOR_RA);
    
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
#endif
}

void loop()
{
    PRINT_SERIAL(getDistance());
    
#ifdef DONT_MOVE
    return;
#endif
    
    int16_t distance = getDistance();
    uint8_t irStatus = getIRSensorStatus();
    /*if ((irStatus & IR_SENSE_F) || distance >= 0) {
        
    }*/
    
    
    
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
