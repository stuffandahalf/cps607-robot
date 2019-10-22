//#define SERIAL_DEBUG

#include "MotorControl.h"
#include "SonarSensor.h"

#define MOTOR_RA    (10)
#define MOTOR_RB    (9)
#define MOTOR_LA    (6)
#define MOTOR_LB    (5)

#define IR_SENSE_F  (11)
#define IR_SENSE_R  (A5)
#define IR_SENSE_L  (A4)

#define DISTANCE_SENSE_L_TRIGGER  (A1)
#define DISTANCE_SENSE_L_ECHO     (A0)
#define DISTANCE_SENSE_R_TRIGGER  (3)
#define DISTANCE_SENSE_R_ECHO     (4)

#define BASE_SPEED  (52)
#define R_SPEED     (BASE_SPEED)
#define L_SPEED     (BASE_SPEED + 12)

#define IR_STATUS_CLEAR (0)
#define IR_STATUS_F     (1 << 0)
#define IR_STATUS_R     (1 << 1)
#define IR_STATUS_L     (1 << 2)
#define IR_STATUS_FR    (IR_STATUS_F | IR_STATUS_R)
#define IR_STATUS_FL    (IR_STATUS_F | IR_STATUS_L)
#define IR_STATUS_ALL   (IR_STATUS_F | IR_STATUS_R | IR_STATUS_L)

#define SONAR_DISTANCE  (250)
#define SONAR_SEQUENCE_DELAY (350)

#ifdef SERIAL_DEBUG
#define SERIAL_PRINT(...) Serial.print(__VA_ARGS__)
#define SERIAL_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
#define SERIAL_PRINT(...)
#define SERIAL_PRINTLN(...)
#endif

MotorControl *leftMotor;
MotorControl *rightMotor;

SonarSensor *leftSonar;
SonarSensor *rightSonar;

//bool turnClockwise;

const byte vout[] = { 2 };
const byte vout_count = sizeof(vout) / sizeof(byte);

inline uint8_t getIRStatus()
{
    uint8_t status = IR_STATUS_CLEAR;
    
    if (digitalRead(IR_SENSE_F)) {
        status |= IR_STATUS_F;
    }
    if (digitalRead(IR_SENSE_R)) {
        status |= IR_STATUS_R;
    }
    if (digitalRead(IR_SENSE_L)) {
        status |= IR_STATUS_L;
    }
    
    return status;
}

template <typename T>
inline bool inRange(T x, T a, T b) {  }

void setup()
{
#ifdef SERIAL_DEBUG
    Serial.begin(115200);
#endif
    //turnClockwise = true;
    
    leftMotor = new MotorControl(MOTOR_LB, MOTOR_LA);
    rightMotor = new MotorControl(MOTOR_RA, MOTOR_RB);
    
    leftSonar = new SonarSensor(DISTANCE_SENSE_L_TRIGGER, DISTANCE_SENSE_L_ECHO);
    rightSonar = new SonarSensor(DISTANCE_SENSE_R_TRIGGER, DISTANCE_SENSE_R_ECHO);
    
    pinMode(IR_SENSE_F, INPUT);
    pinMode(IR_SENSE_R, INPUT);
    pinMode(IR_SENSE_L, INPUT);
    
    for (int i = 0; i < vout_count; i++) {
        pinMode(vout[i], OUTPUT);
        digitalWrite(vout[i], HIGH);
    }
    
    pinMode(13, OUTPUT);
}

void loop()
{
    uint8_t irStatus;
    if ((irStatus = getIRStatus())) {
        int16_t lSpeed = L_SPEED;
        int16_t rSpeed = R_SPEED;
        
        switch (irStatus) {
        case IR_STATUS_F:
            rSpeed *= -1;
            break;
        case IR_STATUS_FR:
            rSpeed *= -1;
            // fall through
        case IR_STATUS_R:
            lSpeed *= -1;
            break;
        case IR_STATUS_FL:
            lSpeed = 0;
            // fall through
        case IR_STATUS_L:
            rSpeed *= -1;
            break;
        case IR_STATUS_ALL:
            lSpeed = 0;
            rSpeed = 0;
            break;
        }
        
        
        /*if (irStatus & IR_STATUS_R) {
            turnClockwise = false;
        }
        else {
            turnClockwise = true;
        }*/
        
        leftMotor->forward(lSpeed);
        rightMotor->forward(rSpeed);
        
        return;
    }
    
    bool distanceAffected = false;
    
    //int16_t lDistance;
    //int16_t rDistance;
    
    int16_t lDistance = leftSonar->getDistance();
    int16_t rDistance = rightSonar->getDistance();

    SERIAL_PRINT(lDistance);
    SERIAL_PRINT('\t');
    SERIAL_PRINTLN(rDistance);
    
    while (inRange<int16_t>(lDistance, 0, SONAR_DISTANCE) ||
        inRange<int16_t>(rDistance, 0, SONAR_DISTANCE)) {
        digitalWrite(13, !digitalRead(13));
            
        distanceAffected = true;
            
        int16_t lSpeed = L_SPEED;
        int16_t rSpeed = R_SPEED;
        
        /*if (lDistance < 50 || rDistance < 50 || (getIRSensorStatus() & SENSE_IR_FRONT)) {
            leftMotor->forward(L_SPEED * -1);
            rightMotor->forward(R_SPEED * -1);
            delay(SONAR_SEQUENCE_DELAY);
            leftMotor->brake();
            rightMotor->brake();
        }*/
                    
        if (lDistance < rDistance) {
            //lSpeed = 0;
            rSpeed *= -1;
        }
        else /*if (lDistance > rDistance)*/ {
            //rSpeed = 0;
            lSpeed *= -1;
        }
        
        leftMotor->forward(lSpeed);
        rightMotor->forward(rSpeed);
        
        lDistance = leftSonar->getDistance();
        rDistance = rightSonar->getDistance();
    }
    
    if (distanceAffected) {
        return;
    }
    
    leftMotor->forward(L_SPEED);
    rightMotor->forward(R_SPEED);
}
