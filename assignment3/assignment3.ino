//#define DONT_MOVE
//#define SERIAL_DEBUG
#define USE_EDGE
#define USE_DISTANCE
//#define USE_LINE
#define USE_LINE2
#define USE_DELAY
//#define USE_LED
#define USE_BATTERY_SENSE

#include "MotorControl.h"
#include "SonarSensor.h"

#define MOTOR_RA    (10)
#define MOTOR_RB    (9)
#define MOTOR_LA    (6)
#define MOTOR_LB    (5)

//#define IR_SENSE_F  (11)
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
//#define DISTANCE_IR_SENSE (11)

#define BATTERY_SENSE (A2)

#ifdef USE_LED
#define LED (13)
#endif

//#define BASE_SPEED(x)  (54 + (100 - x) * 45 / 100 - 5)
#define BASE_SPEED(x)  (54 + (100 - x) * 58 / 100 - 5)
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

#define LINE_STATUS_CLEAR   (0)
#define LINE_STATUS_FRONT_RIGHT (1 << 0)
#define LINE_STATUS_FRONT_LEFT  (1 << 1)
#define LINE_STATUS_REAR        (1 << 2)
#define LINE_STATUS_FRONT_BOTH  (LINE_STATUS_FRONT_RIGHT | LINE_STATUS_FRONT_LEFT)
#define LINE_STATUS_ALL         (LINE_STATUS_FRONT_BOTH | LINE_STATUS_REAR)

#define LINE_DELAY (250)

#define MAX_LINE_ALIGN_CYCLES (128)
#define LINE_ROTATION_DELAY (16)


#define SONAR_DISTANCE  (250)
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

const byte vout[] = { 2 };
const byte vout_count = sizeof(vout) / sizeof(byte);

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

template <typename T>
inline bool inRange(T x, T a, T b) { return a <= x && x < b; }

inline int getBatteryStatus()
{
#ifdef USE_BATTERY_SENSE
    return map(analogRead(BATTERY_SENSE), 150, 650, 0, 100);
#else
    //return 59;
    return 34;
#endif
}

#if 0
inline int degreesToCycles(int degrees)
{
    return map(degrees, 0, 360, 0, MAX_LINE_ALIGN_CYCLES);
}

inline int cycleToDegrees(int cycles)
{
    return map(cycles, 0, MAX_LINE_ALIGN_CYCLES, 0, 360);
}

bool lineAlign(int maxAlignCycles)
{
    int leftMultiplier = 1;
    int rightMultiplier = -1;
    
    if (maxAlignCycles < 0) {
        leftMultiplier = -1;
        rightMultiplier = 1;
        maxAlignCycles *= -1;
    }
    
    if (maxAlignCycles == 0) {
        maxAlignCycles = MAX_LINE_ALIGN_CYCLES;
    }
    
    uint16_t cycles = 0;
    uint8_t lineStatus;
    while ((lineStatus = getLineStatus()) != LINE_STATUS_BOTH && cycles < maxAlignCycles) {
        /*if (cycles % 2) {
            delay(100);
        }*/
        leftMotor->forward(L_SPEED(getBatteryStatus()) * leftMultiplier - 10);
        rightMotor->forward(R_SPEED(getBatteryStatus()) * rightMultiplier - 10);
        delay(LINE_ROTATION_DELAY);
        leftMotor->brake();
        rightMotor->brake();
        cycles++;
    }
    
    leftMotor->forward(L_SPEED(getBatteryStatus()));
    rightMotor->forward(R_SPEED(getBatteryStatus()));
    
    return cycles != maxAlignCycles;
}
#endif

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
    
    //pinMode(DISTANCE_IR_SENSE, INPUT);
    
    for (int i = 0; i < vout_count; i++) {
        pinMode(vout[i], OUTPUT);
        digitalWrite(vout[i], HIGH);
    }
    
#ifdef USE_LED
    pinMode(LED, OUTPUT);
#endif
#ifdef USE_BATTERY_SENSE
    pinMode(BATTERY_SENSE, INPUT);
#endif
    //lineAlign(0);
}

void loop()
{
    SERIAL_PRINT("BATTERY: ");
    SERIAL_PRINT(getBatteryStatus());
    SERIAL_PRINT("\tSPEED: ");
    SERIAL_PRINTLN(BASE_SPEED(getBatteryStatus()));
    
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
    
#ifdef USE_LINE
    uint8_t lineStatus = getLineStatus();
    switch (lineStatus) {
    case LINE_STATUS_F:
        break;
    case LINE_STATUS_R:
        delay(50);
        if (onLine) {
            // sweep 180* to relocate line. if not found, online = false
            SERIAL_PRINTLN("RELOCATING LINE");
            if (lineAlign(MAX_LINE_ALIGN_CYCLES / 4) || lineAlign(MAX_LINE_ALIGN_CYCLES / 2 * -1)) {
                onLine = true;
                SERIAL_PRINTLN("FOUND LINE");
            }
            else {
                onLine = false;
                SERIAL_PRINTLN("LOST LINE");
                delay(500);
            }
        }
        else {
            // sweep 360* to locate line.
            SERIAL_PRINTLN("LOCATING LINE");
            if (lineAlign(0)) {
                onLine = true;
                SERIAL_PRINTLN("FOUND LINE");
            }
            else {
                onLine = false;
                SERIAL_PRINTLN("NO LINE");
                //delay(150);
            }
        }
        break;
    case LINE_STATUS_BOTH:
        SERIAL_PRINTLN("ON LINE");
        onLine = true;
        break;
    case LINE_STATUS_CLEAR:
    default:
        onLine = false;
        break;
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
        digitalWrite(LED, !digitalRead(LED));
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
