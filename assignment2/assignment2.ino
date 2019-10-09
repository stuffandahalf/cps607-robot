#include "MotorControl.h"
#include "SonarSensor.h"

/* Macros to change the execution style */
//#define SERIAL_DEBUG
//#define DONT_MOVE
#define USE_DISTANCE

#define MOTOR_LA (10)
#define MOTOR_LB (9)
#define MOTOR_RA (6)
#define MOTOR_RB (5)

#define IR_SENSE_F (11)
#define IR_SENSE_R (A4)
#define IR_SENSE_L (A5)

#define DISTANCE_SENSE_L_TRIGGER  (A1)
#define DISTANCE_SENSE_L_ECHO     (A0)
#define DISTANCE_SENSE_R_TRIGGER  (3)
#define DISTANCE_SENSE_R_ECHO     (4)

#define SONAR_DISTANCE (250)
#define SONAR_SEQUENCE_DELAY (350)

#define SENSE_CLEAR     (0u)
#define SENSE_IR_FRONT  (1 << 0)
#define SENSE_IR_LEFT   (1 << 2)
#define SENSE_IR_RIGHT  (1 << 3)

#define SENSE_IR_FRONT_LEFT    (SENSE_IR_FRONT | SENSE_IR_LEFT)
#define SENSE_IR_FRONT_RIGHT   (SENSE_IR_FRONT | SENSE_IR_RIGHT)
#define SENSE_IR_OFF_TABLE     (SENSE_IR_FRONT | SENSE_IR_LEFT | SENSE_IR_RIGHT)

#define DIRECTION_NONE  (0)
#define DIRECTION_LEFT  (1)
#define DIRECTION_RIGHT (2)

#ifdef SERIAL_DEBUG
#define RSPEED (128)
#else
//#define RSPEED (65) /* reduced battery */
#define RSPEED (50) /* Full battery */
#endif
#define LSPEED (RSPEED - 7)
//#define REVERSE_OFFSET (10)
#define REVERSE_OFFSET (15)

#ifdef SERIAL_DEBUG
#define PRINT_SERIAL(...) Serial.println(__VA_ARGS__);
#else
#define PRINT_SERIAL(...)
#endif

const byte vout[] = { 2 };
const unsigned int vout_count = sizeof(vout) / sizeof(byte);

MotorControl *leftMotor;
MotorControl *rightMotor;
SonarSensor *lSonarSensor;
SonarSensor *rSonarSensor;
int16_t lSpeed;
int16_t rSpeed;
uint8_t lastSensed;

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
    lSonarSensor = new SonarSensor(DISTANCE_SENSE_L_TRIGGER, DISTANCE_SENSE_L_ECHO);
    rSonarSensor = new SonarSensor(DISTANCE_SENSE_R_TRIGGER, DISTANCE_SENSE_R_ECHO);
    
    pinMode(IR_SENSE_F, INPUT);
    pinMode(IR_SENSE_R, INPUT);
    pinMode(IR_SENSE_L, INPUT);
    
    /*pinMode(DISTANCE_SENSE_TRIGGER, OUTPUT);
    pinMode(DISTANCE_SENSE_ECHO, INPUT);*/
    
    //digitalWrite(DISTANCE_SENSE_TRIGGER, LOW);
    
    for (unsigned int i = 0; i < vout_count; i++) {
        pinMode(vout[i], OUTPUT);
        digitalWrite(vout[i], HIGH);
    }
    
    lSpeed = LSPEED;
    rSpeed = RSPEED;
    
#ifndef DONT_MOVE
    leftMotor->forward(lSpeed);
    rightMotor->forward(rSpeed);
#else
    leftMotor->brake();
    rightMotor->brake();
#endif

    lastSensed = SENSE_IR_LEFT;
}

void loop()
{
    /*Serial.print("LEFT: ");
    Serial.print(lSonarSensor->getDistance());
    Serial.print("\tRIGHT: ");
    Serial.print(rSonarSensor->getDistance());
    Serial.println();*/
    
#ifdef DONT_MOVE
    return;
#endif


#ifdef USE_DISTANCE
    int16_t lDistance = lSonarSensor->getDistance();
    int16_t rDistance = rSonarSensor->getDistance();
#endif
    uint8_t irStatus = getIRSensorStatus();
    
#ifdef USE_DISTANCE
    PRINT_SERIAL(distance);
    
    // If an object is close, then go away from it just like if it was driving off the table
    /*if (inRange<int16_t>(lDistance, 0, SONAR_DISTANCE) || inRange<int16_t>(rDistance, 0, SONAR_DISTANCE)) {
        irStatus |= SENSE_IR_FRONT;
        if (lDistance < rDistance) {
            irStatus |= SENSE_IR_LEFT;
        }
        else if (lDistance > rDistance) {
            irStatus |= SENSE_IR_RIGHT;
        }
    }*/
    while (inRange<int16_t>((lDistance = lSonarSensor->getDistance()), 0, SONAR_DISTANCE) || inRange<int16_t>((rDistance = rSonarSensor->getDistance()), 0, SONAR_DISTANCE)) {
        int16_t lSpeed;
        int16_t rSpeed;
        if (lDistance < rDistance) {
            lSpeed = LSPEED;
            rSpeed = (RSPEED + REVERSE_OFFSET) * -1;
            lastSensed = SENSE_IR_FRONT_LEFT;
        }
        else /*if (lDistance > rDistance)*/ {
            lSpeed = (LSPEED + REVERSE_OFFSET) * -1;
            rSpeed = RSPEED;
            lastSensed = SENSE_IR_FRONT_RIGHT;
        }
        leftMotor->forward(lSpeed);
        rightMotor->forward(rSpeed);
        delay(SONAR_SEQUENCE_DELAY);
    }
    
    leftMotor->forward(lSpeed);
    rightMotor->forward(rSpeed);
#endif
    
    if (irStatus != SENSE_CLEAR) {     // approaching table edge from diagonal
        switch (irStatus) {
        case SENSE_IR_FRONT:
            /*if (lastSensed & SENSE_IR_LEFT) {
                rSpeed = (RSPEED + REVERSE_OFFSET) * -1;
                lSpeed = LSPEED;
            }
            else {
                lSpeed = (LSPEED + REVERSE_OFFSET) * -1;
                rSpeed = RSPEED;
            }*/
            switch (lastSensed) {
            case SENSE_IR_FRONT_LEFT:
            case SENSE_IR_LEFT:
                rSpeed = (RSPEED + REVERSE_OFFSET) * -1;
                lSpeed = LSPEED;
                break;
            case SENSE_IR_FRONT_RIGHT:
            case SENSE_IR_RIGHT:
                lSpeed = (LSPEED + REVERSE_OFFSET) * -1;
                rSpeed = RSPEED;
                break;
            case SENSE_IR_FRONT:
            case SENSE_IR_OFF_TABLE:
            default:
                /*lSpeed = 0;
                rSpeed = 0;*/
                lSpeed = (LSPEED + REVERSE_OFFSET) * -1;
                rSpeed = RSPEED;
                break;
            }
            break;
        case SENSE_IR_RIGHT:
            rSpeed = 0;
            lSpeed = (LSPEED + REVERSE_OFFSET) * -1;
            break;
        case SENSE_IR_LEFT:
            rSpeed = (RSPEED + REVERSE_OFFSET) * -1;
            lSpeed = 0;
            break;
            
        case SENSE_IR_FRONT_LEFT:
            lSpeed = LSPEED;
            rSpeed = (RSPEED + REVERSE_OFFSET) * -1;
            break;
        case SENSE_IR_FRONT_RIGHT:
            rSpeed = RSPEED;
            lSpeed = (LSPEED + REVERSE_OFFSET) * -1;
            break;
            
        case SENSE_IR_OFF_TABLE:    // maybe reverse, wait, turn, drive?
            lSpeed = 0;
            rSpeed = 0;
            break;
        }
        lastSensed = irStatus;
    }
    else {
        lSpeed = LSPEED;
        rSpeed = RSPEED;
    }

    leftMotor->forward(lSpeed);
    rightMotor->forward(rSpeed);
}
