#include "MotorControl.h"
#include "Node.h"
#include "SonarSensor.h"

// logic control macros
#define SERIAL_DEBUG

// peripheral pin macros
#define FLAME_SENSE     ()

#define LINE_SENSE_L    ()
#define LINE_SENSE_R    ()
#define LINE_SENSE_B    ()

#define DISTANCE_SENSE_TRIGGER  ()
#define DISTANCE_SENSE_ECHO     ()

// serial debug macros
#ifdef SERIAL_DEBUG
#define SERIAL_PRINTLN(...) Serial.println(__VA_ARGS__)
#define SERIAL_PRINT(...)   Serial.print(__VA_ARGS__)
#else
#define SERIAL_PRINTLN(...)
#define SERIAL_PRINT(...)
#endif


MotorControl *left_motor;
MotorControl *right_motor;

Node *current_node;

void find_start_node();

void setup()
{
#ifdef SERIAL_DEBUG
    Serial.begin(115200);
#endif
    
    
    
    init_nodes();
    find_start_node();
}

void loop()
{
    
}

void find_start_node()
{
    // rotate clockwise until aligned on tape
    // check for flame
    // rotate clockwise until aligned on tape
    // check for flame
    // use combination of where flame detected to determine position
}
