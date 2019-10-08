#include "MotorControl.h"
#include "SonarSensor.h"

/* Macros to change the execution style */
//#define SERIAL_DEBUG
//#define DONT_MOVE

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

#define SENSE_IR_FRONT_LEFT    (SENSE_IR_FRONT | SENSE_IR_LEFT)
#define SENSE_IR_FRONT_RIGHT   (SENSE_IR_FRONT | SENSE_IR_RIGHT)
#define SENSE_IR_OFF_TABLE     (SENSE_IR_FRONT | SENSE_IR_LEFT | SENSE_IR_RIGHT)

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
int16_t lSpeed;
int16_t rSpeed;

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
    
    lSpeed = SPEED;
    rSpeed = SPEED;
    
#ifndef DONT_MOVE
    leftMotor->forward(lSpeed);
    rightMotor->forward(rSpeed);
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
    
    // If an object is close, then go away from it just like if it was driving off the table
    if (inRange<int16_t>(distance, 0, 150)) {
        irStatus |= SENSE_IR_FRONT;
    }
    
    if (irStatus != SENSE_CLEAR) {     // approaching table edge from diagonal
        switch (irStatus) {
        case SENSE_IR_FRONT:
            if (lSpeed > 0) {
                lSpeed *= -1;
            }
            rSpeed = SPEED;
            break;
        case SENSE_IR_RIGHT:
            if (lSpeed > 0) {
                lSpeed *= -1;
                rSpeed = SPEED;
            }
            break;
        case SENSE_IR_LEFT:
            if (rSpeed > 0) {
                rSpeed *= -1;
            }
            lSpeed = SPEED;
            break;
        case SENSE_IR_FRONT_LEFT:
            lSpeed = 0;
            rSpeed = SPEED;
            break;
        case SENSE_IR_FRONT_RIGHT:
            rSpeed = 0;
            lSpeed = SPEED;
            break;
        case SENSE_IR_OFF_TABLE:
            lSpeed = 0;
            rSpeed = 0;
            break;
        }
    }
    else {
        lSpeed = SPEED;
        rSpeed = SPEED;
    }
    
    leftMotor->forward(lSpeed);
    rightMotor->forward(rSpeed);
}
