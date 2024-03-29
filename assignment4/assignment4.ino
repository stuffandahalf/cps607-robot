/* Debug macros */
//#define DONT_MOVE
//#define SERIAL_DEBUG
//#define ANALOG_DIGITAL_DEBUG

/* Feature macros */
#define USE_EDGE
#define USE_DISTANCE
#define USE_LINE2
#define USE_FLAME
#define USE_DELAY
//#define USE_LED
#define USE_BATTERY_SENSE

#include "MotorControl.h"
#include "SonarSensor.h"

#define MOTOR_RA    (10)
#define MOTOR_RB    (9)
#define MOTOR_LA    (6)
#define MOTOR_LB    (5)

#define EDGE_F_SONAR_TRIGGER    (A0)
#define EDGE_F_SONAR_ECHO       (A1)
#define EDGE_R_IR_SENSE         (3)
#define EDGE_L_IR_SENSE         (4)

#define LINE_SENSE_FRONT_LEFT   (8)
#define LINE_SENSE_FRONT_RIGHT  (7)
#define LINE_SENSE_REAR         (A3)

#define DISTANCE_SENSE_L_TRIGGER  (A4)
#define DISTANCE_SENSE_L_ECHO     (12)
#define DISTANCE_SENSE_R_TRIGGER  (A5)
#define DISTANCE_SENSE_R_ECHO     (11)

#define FLAME_L_IR_SENSE    (A2)
#define FLAME_R_IR_SENSE    (2)

#define BATTERY_SENSE (A6)

//#define BASE_SPEED(x)   (54 + map(x, 0, 100, 60, 0))
#define BASE_SPEED(x)   (54 + map(x, 0, 100, 90, 0))
//#define BASE_SPEED(x)  (54 + (100 - x) * 45 / 100 - 5)
//#define BASE_SPEED(x)  (54 + (100 - x) * 58 / 100 - 5)
//#define BASE_SPEED(x)  (75)
#define R_SPEED(x)  (BASE_SPEED(x))
#define L_SPEED(x)  (BASE_SPEED(x) + 10)

#define EDGE_STATUS_CLEAR (0)
#define EDGE_STATUS_F     (1 << 0)
#define EDGE_STATUS_R     (1 << 1)
#define EDGE_STATUS_L     (1 << 2)
#define EDGE_STATUS_FR    (EDGE_STATUS_F | EDGE_STATUS_R)
#define EDGE_STATUS_FL    (EDGE_STATUS_F | EDGE_STATUS_L)
#define EDGE_STATUS_ALL   (EDGE_STATUS_F | EDGE_STATUS_R | EDGE_STATUS_L)

#define LINE_STATUS_CLEAR       (0)
#define LINE_STATUS_FRONT_RIGHT (1 << 0)
#define LINE_STATUS_FRONT_LEFT  (1 << 1)
#define LINE_STATUS_REAR        (1 << 2)
#define LINE_STATUS_FRONT_BOTH  (LINE_STATUS_FRONT_RIGHT | LINE_STATUS_FRONT_LEFT)
#define LINE_STATUS_ALL         (LINE_STATUS_FRONT_BOTH | LINE_STATUS_REAR)

#define LINE_DELAY (250)

#define MAX_LINE_ALIGN_CYCLES (128)
#define LINE_ROTATION_DELAY (16)

#define FLAME_STATUS_CLEAR  (0)
#define FLAME_STATUS_LEFT   (1)
#define FLAME_STATUS_RIGHT  (2)
#define FLAME_STATUS_BOTH   (FLAME_STATUS_LEFT | FLAME_STATUS_RIGHT)

//#define SONAR_DISTANCE  (250)
#define SONAR_DISTANCE  (350)
#define SONAR_SEQUENCE_DELAY (350)

//#define TURN_DELAY (75)
#define TURN_DELAY (500)

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

SonarSensor *frontEdgeSonar;

SonarSensor *leftSonar;
SonarSensor *rightSonar;
bool evenCycle;

//bool turnClockwise;

/*const byte vout[] = { 2 };
const byte vout_count = sizeof(vout) / sizeof(byte);*/


template <typename T>
inline bool inRange(T x, T a, T b) { return a <= x && x < b; }

inline uint8_t getEdgeStatus()
{
    uint8_t status = EDGE_STATUS_CLEAR;
    
    //if (digitalRead(IR_SENSE_F)) {
    int16_t frontEdgeDistance = frontEdgeSonar->getDistance();
    
    if (!inRange(frontEdgeDistance, 0, 150)) {
        status |= EDGE_STATUS_F;
    }
    if (digitalRead(EDGE_R_IR_SENSE)) {
        status |= EDGE_STATUS_R;
    }
    if (digitalRead(EDGE_L_IR_SENSE)) {
        status |= EDGE_STATUS_L;
    }
    
    return status;
}

inline uint8_t getLineStatus()
{
    uint8_t status = LINE_STATUS_CLEAR;
    if (digitalRead(LINE_SENSE_FRONT_LEFT)) {
        status |= LINE_STATUS_FRONT_LEFT;
    }
    if (digitalRead(LINE_SENSE_FRONT_RIGHT)) {
        status |= LINE_STATUS_FRONT_RIGHT;
    }
    if (digitalRead(LINE_SENSE_REAR)) {
        status |= LINE_STATUS_REAR;
    }
    return status;
}

inline uint8_t getFlameStatus()
{
    uint8_t status = FLAME_STATUS_CLEAR;
/*#ifdef ANALOG_DIGITAL_DEBUG
    SERIAL_PRINT(analogRead(FLAME_L_IR_SENSE));
    SERIAL_PRINT(" ");
#endif
    if (inRange<int>(analogRead(FLAME_L_IR_SENSE), 70, 250)) {*/
    if (!digitalRead(FLAME_L_IR_SENSE)) {
        status |= FLAME_STATUS_LEFT;
    }
/*#ifdef ANALOG_DIGITAL_DEBUG
    SERIAL_PRINT(analogRead(FLAME_R_IR_SENSE));
    SERIAL_PRINT(" ");
#endif
    if (inRange<int>(analogRead(FLAME_R_IR_SENSE), 150, 305)) {*/
    if (!digitalRead(FLAME_R_IR_SENSE)) {
        status |= FLAME_STATUS_RIGHT;
    }
    return status;
}

inline int getBatteryStatus()
{
#ifdef USE_BATTERY_SENSE
    return map(analogRead(BATTERY_SENSE), 150, 650, 0, 100);
#else
    //return 59;
    return 34;
#endif
}

void locateLine()
{
    int16_t lSpeed = L_SPEED(getBatteryStatus());
    int16_t rSpeed = R_SPEED(getBatteryStatus());
    
    rSpeed *= -1;
    
    
    leftMotor->forward(lSpeed);
    rightMotor->forward(rSpeed);
    
    while (!(getLineStatus() & LINE_STATUS_FRONT_RIGHT));
    while (getLineStatus() & LINE_STATUS_FRONT_RIGHT);
    
    leftMotor->brake();
    rightMotor->brake();
}

void setup()
{
#ifdef SERIAL_DEBUG
    Serial.begin(115200);
#endif
    //turnClockwise = true;
    
    evenCycle = true;
    
    leftMotor = new MotorControl(MOTOR_LB, MOTOR_LA);
    rightMotor = new MotorControl(MOTOR_RA, MOTOR_RB);
    
    leftSonar = new SonarSensor(DISTANCE_SENSE_L_TRIGGER, DISTANCE_SENSE_L_ECHO);
    rightSonar = new SonarSensor(DISTANCE_SENSE_R_TRIGGER, DISTANCE_SENSE_R_ECHO);
    frontEdgeSonar = new SonarSensor(EDGE_F_SONAR_TRIGGER, EDGE_F_SONAR_ECHO);
    
    //pinMode(IR_SENSE_F, INPUT);
    pinMode(EDGE_R_IR_SENSE, INPUT);
    pinMode(EDGE_L_IR_SENSE, INPUT);
    
    pinMode(LINE_SENSE_FRONT_LEFT, INPUT);
    pinMode(LINE_SENSE_FRONT_RIGHT, INPUT);
    pinMode(LINE_SENSE_REAR, INPUT);
    
    pinMode(FLAME_L_IR_SENSE, INPUT);
    pinMode(FLAME_R_IR_SENSE, INPUT);
    
    //pinMode(DISTANCE_IR_SENSE, INPUT);
    
    /*for (int i = 0; i < vout_count; i++) {
        pinMode(vout[i], OUTPUT);
        digitalWrite(vout[i], HIGH);
    }*/
    
#ifdef USE_LED
    pinMode(13, OUTPUT);
#endif
#ifdef USE_BATTERY_SENSE
    pinMode(BATTERY_SENSE, INPUT);
#endif
    //lineAlign(0);
}

void loop()
{
    /*SERIAL_PRINT("BATTERY: ");
    SERIAL_PRINT(getBatteryStatus());
    SERIAL_PRINT("\tSPEED: ");
    SERIAL_PRINTLN(BASE_SPEED(getBatteryStatus()));*/
    
    /*SERIAL_PRINT("FRONT: ");
    SERIAL_PRINT(digitalRead(LINE_SENSE_F));
    SERIAL_PRINT("\tREAR: ");
    SERIAL_PRINTLN(digitalRead(LINE_SENSE_R));*/
    
    /*SERIAL_PRINT("FRONT: ");
    SERIAL_PRINT(frontEdgeSonar->getDistance());
    SERIAL_PRINT("\TLEFT: ");
    SERIAL_PRINT(leftSonar->getDistance());
    SERIAL_PRINT("\tRIGHT: ");
    SERIAL_PRINTLN(rightSonar->getDistance());*/
    
    //SERIAL_PRINTLN(getEdgeStatus());
    
    /*SERIAL_PRINT("LINE_SENSE STATUS: ");
    SERIAL_PRINTLN(getLineStatus());*/
    
    SERIAL_PRINT("FLAME_SENSE STATUS: ");
    SERIAL_PRINTLN(getFlameStatus());
    
#ifdef DONT_MOVE
    return;
#endif

    evenCycle = !evenCycle;
    
#ifdef USE_EDGE
    uint8_t edgeStatus;
    if ((edgeStatus = getEdgeStatus())) {
        int16_t lSpeed = L_SPEED(getBatteryStatus());
        int16_t rSpeed = R_SPEED(getBatteryStatus());
#ifdef USE_DELAY
        int16_t turnDelay = 0;
#endif
        
        switch (edgeStatus) {
        case EDGE_STATUS_F:
            rSpeed *= -1;
#ifdef USE_DELAY
            turnDelay = TURN_DELAY;
#endif
            break;
        case EDGE_STATUS_FR:
            rSpeed = 0;
            // fall through
        case EDGE_STATUS_R: /* TEST THIS */
            lSpeed *= -1;
#ifdef USE_DELAY
            turnDelay = TURN_DELAY;
#endif
            break;
        case EDGE_STATUS_FL:
            lSpeed = 0;
            // fall through
        case EDGE_STATUS_L: /* AND THIS */
            rSpeed *= -1;
#ifdef USE_DELAY
            turnDelay = TURN_DELAY;
#endif
            break;
        case EDGE_STATUS_ALL:
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
#ifdef USE_LINE
        onLine = false;
#endif
        return;
    }
#endif

#ifdef USE_LINE2
    uint8_t lineStatus = getLineStatus();
    if (lineStatus != LINE_STATUS_CLEAR) {
        int16_t lSpeed = L_SPEED(getBatteryStatus()) + 5;
        int16_t rSpeed = R_SPEED(getBatteryStatus()) + 5;
        int16_t lineDelay = 0;
        
        switch (lineStatus & LINE_STATUS_FRONT_BOTH) {
        case LINE_STATUS_FRONT_LEFT:
            lSpeed *= -1;
            lineDelay = LINE_DELAY;
            break;
        case LINE_STATUS_FRONT_RIGHT:
            rSpeed *= -1;
            lineDelay = LINE_DELAY;
            break;
        case LINE_STATUS_FRONT_BOTH:
            while (!(getLineStatus() & LINE_STATUS_REAR)) {
                leftMotor->forward(lSpeed);
                rightMotor->forward(rSpeed);
            }
            locateLine();
            break;
        }
        
        leftMotor->forward(lSpeed);
        rightMotor->forward(rSpeed);
        delay(lineDelay);
        return;
    }
#endif


#ifdef USE_FLAME
    //SERIAL_PRINTLN("GOT HERE");

    uint8_t flameStatus = getFlameStatus();
    if (flameStatus != FLAME_STATUS_CLEAR) {
        switch (flameStatus) {
        case FLAME_STATUS_LEFT:
            //rightMotor->forward(R_SPEED(getBatteryStatus()) * -1);
            rightMotor->brake();
#ifdef USE_DELAY
            delay(75);
#endif
            break;
        case FLAME_STATUS_RIGHT:
            //leftMotor->forward(L_SPEED(getBatteryStatus()) * -1);
            leftMotor->brake();
#ifdef USE_DELAY
            delay(75);
#endif
            break;
        case FLAME_STATUS_BOTH:
            // Can't check for sensor distance, because candles sit too low for distance sensor
            leftMotor->brake();
            rightMotor->brake();
            break;
        /*case FLAME_STATUS_CLEAR:
        default:
            break;*/
        }
        
        return;
    }
#endif

#ifdef USE_DISTANCE
    bool distanceAffected = false;
    
    int16_t lDistance;
    int16_t rDistance;
    
    /*int16_t lDistance = leftSonar->getDistance();
    int16_t rDistance = rightSonar->getDistance();*/

    /*SERIAL_PRINT(lDistance);
    SERIAL_PRINT('\t');
    SERIAL_PRINTLN(rDistance);*/
    
    while (inRange<int16_t>((lDistance = leftSonar->getDistance()), 0, SONAR_DISTANCE) ||
        inRange<int16_t>((rDistance = rightSonar->getDistance()), 0, SONAR_DISTANCE)) {
    //while (!digitalRead(DISTANCE_IR_SENSE)) {
#ifdef USE_LED
        digitalWrite(13, !digitalRead(13));
#endif
            
        distanceAffected = true;
            
        int16_t lSpeed = L_SPEED(getBatteryStatus());
        int16_t rSpeed = R_SPEED(getBatteryStatus());
#ifdef USE_DELAY
        int16_t turnDelay = 0;
#endif
        
        if (lDistance < 100 || rDistance < 100) {
            leftMotor->forward(L_SPEED(getBatteryStatus()) * -1);
            rightMotor->forward(R_SPEED(getBatteryStatus()) * -1);
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
        delay(500);
        
        /*lDistance = leftSonar->getDistance();
        rDistance = rightSonar->getDistance();*/
    }
    
    if (distanceAffected) {
        return;
    }
#endif

    leftMotor->forward(L_SPEED(getBatteryStatus()));
    rightMotor->forward(R_SPEED(getBatteryStatus()));

}
