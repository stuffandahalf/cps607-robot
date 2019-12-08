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

#define BATTERY_SENSE   (A6)

// serial debug macros
#ifdef SERIAL_DEBUG
#define SERIAL_PRINTLN(...) Serial.println(__VA_ARGS__)
#define SERIAL_PRINT(...)   Serial.print(__VA_ARGS__)
#else
#define SERIAL_PRINTLN(...)
#define SERIAL_PRINT(...)
#endif

// Motor PWM
//#define BASE_SPEED(x)   (54 + map(x, 0, 100, 60, 0))
#define BASE_SPEED(x)   (54 + map(x, 0, 100, 90, 0))
//#define BASE_SPEED(x)  (54 + (100 - x) * 45 / 100 - 5)
//#define BASE_SPEED(x)  (54 + (100 - x) * 58 / 100 - 5)
//#define BASE_SPEED(x)  (75)
#define R_SPEED(x)  (BASE_SPEED(x))
#define L_SPEED(x)  (BASE_SPEED(x) + 10)

MotorControl *left_motor;
MotorControl *right_motor;

Node *current_node;

void find_start_node();

void setup()
{
#ifdef SERIAL_DEBUG
    Serial.begin(115200);
#endif
    
    // init variables and sensors
    
    init_nodes();
    find_start_node();
    LinkedList<Node *> *path = new LinkedList<Node *>();
    reachable_node(start_node_1, end_node, path);
    for (LinkedList<Node *>::ListNode *ln = path->getFirst(); ln != NULL; ln = ln->next) {
        SERIAL_PRINTLN(ln->value->id);
    }
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
    
    current_node = start_node_1;
}
