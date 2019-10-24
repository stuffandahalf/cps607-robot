//#define SERIAL_DEBUG
#define USE_EDGE
#define USE_DISTANCE
#define USE_DELAY
//#define USE_LED
//#define USE_BATTERY_SENSE

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

#define BATTERY_SENSE (A6)

//#define BASE_SPEED(x)  (52 + (100 - x) / (4 + 4 * (x / 100)))
#define BASE_SPEED(x)  (54)
#define R_SPEED(x)  (BASE_SPEED(x))
#define L_SPEED(x)  (BASE_SPEED(x) + 10)

#define IR_STATUS_CLEAR (0)
#define IR_STATUS_F     (1 << 0)
#define IR_STATUS_R     (1 << 1)
#define IR_STATUS_L     (1 << 2)
#define IR_STATUS_FR    (IR_STATUS_F | IR_STATUS_R)
#define IR_STATUS_FL    (IR_STATUS_F | IR_STATUS_L)
#define IR_STATUS_ALL   (IR_STATUS_F | IR_STATUS_R | IR_STATUS_L)

#define SONAR_DISTANCE  (250)
#define SONAR_SEQUENCE_DELAY (350)

#define TURN_DELAY (75)

#define DIRECTION_LEFT (1)
#define DIRECTION_RIGHT (2)

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
inline bool inRange(T x, T a, T b) { return a <= x && x < b; }

inline int getBatteryStatus()
{
#ifdef USE_BATTERY_SENSE
    int batteryStatus = analogRead(BATTERY_SENSE);
    
    return batteryStatus;
#else
    //return 59;
    return 34;
#endif
}

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
    
#ifdef USE_LED
    pinMode(13, OUTPUT);
#endif
#ifdef USE_BATTERY_SENSE
    pinMode(BATTERY_SENSE, INPUT);
#endif
}

void loop()
{
#ifdef USE_EDGE
    uint8_t irStatus;
    if ((irStatus = getIRStatus())) {
        int16_t lSpeed = L_SPEED(getBatteryStatus());
        int16_t rSpeed = R_SPEED(getBatteryStatus());
#ifdef USE_DELAY
        int16_t turnDelay = 0;
#endif
        
        switch (irStatus) {
        case IR_STATUS_F:
            rSpeed *= -1;
#ifdef USE_DELAY
            turnDelay = TURN_DELAY;
#endif
            break;
        case IR_STATUS_FR:
            rSpeed = 0;
            // fall through
        case IR_STATUS_R:
            lSpeed *= -1;
#ifdef USE_DELAY
            turnDelay = TURN_DELAY;
#endif
            break;
        case IR_STATUS_FL:
            lSpeed = 0;
            // fall through
        case IR_STATUS_L:
            rSpeed *= -1;
#ifdef USE_DELAY
            turnDelay = TURN_DELAY;
#endif
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
#ifdef USE_DELAY
        delay(turnDelay);
#endif
        
        return;
    }
#endif
    
#ifdef USE_DISTANCE
    bool distanceAffected = false;
    
    int16_t lDistance;
    int16_t rDistance;
    
    //int16_t lDistance = leftSonar->getDistance();
    //int16_t rDistance = rightSonar->getDistance();

    SERIAL_PRINT(lDistance);
    SERIAL_PRINT('\t');
    SERIAL_PRINTLN(rDistance);
    
    while (inRange<int16_t>((lDistance = leftSonar->getDistance()), 0, SONAR_DISTANCE) ||
        inRange<int16_t>((rDistance = rightSonar->getDistance()), 0, SONAR_DISTANCE)) {
#ifdef USE_LED
        digitalWrite(13, !digitalRead(13));
#endif
            
        distanceAffected = true;
            
        int16_t lSpeed = L_SPEED(getBatteryStatus());
        int16_t rSpeed = R_SPEED(getBatteryStatus());
#ifdef USE_DELAY
        int16_t turnDelay = 0;
#endif
        
        if (lDistance < 100 || rDistance < 100 /*|| (getIRSensorStatus() & SENSE_IR_FRONT)*/) {
            leftMotor->forward(L_SPEED(getBatteryStatus()) * -1);
            rightMotor->forward(R_SPEED(getBatteryStatus()) * -1);
            //delay(SONAR_SEQUENCE_DELAY);
            delay(100);
            leftMotor->brake();
            rightMotor->brake();
        }
        
        if (/*lDistance != -1 &&*/ lDistance < rDistance) {
            //lSpeed = 0;
            rSpeed *= -1;
#ifdef USE_DELAY
            turnDelay = TURN_DELAY;
#endif
        }
        else /*if (rDistance != -1 && lDistance > rDistance)*/ {
            //rSpeed = 0;
            lSpeed *= -1;
#ifdef USE_DELAY
            turnDelay = TURN_DELAY;
#endif
        }
        
        leftMotor->forward(lSpeed);
        rightMotor->forward(rSpeed);
//#ifdef USE_DELAY
        //delay(turnDelay * 2);
        delay(500);
//#endif
        
        //lDistance = leftSonar->getDistance();
        //rDistance = rightSonar->getDistance();
    }
    
    if (distanceAffected) {
        return;
    }
#endif
    

    leftMotor->forward(L_SPEED(getBatteryStatus()));
    rightMotor->forward(R_SPEED(getBatteryStatus()));

}
