#include "MotorControl.h"
#include "Node.h"
#include "SonarSensor.h"

// logic control macros
#define SERIAL_DEBUG
#define NO_SENSORS

// peripheral pin macros
#define MOTOR_RA    (10)
#define MOTOR_RB    (9)
#define MOTOR_LA    (6)
#define MOTOR_LB    (5)

#define FLAME_SENSE     (A5)

#define LINE_SENSE_L    (8)
#define LINE_SENSE_R    (7)
#define LINE_SENSE_B    (A3)

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

SonarSensor *distance_sensor;

Node *current_node;
Direction current_direction;

uint16_t turn_cw();
uint16_t turn_ccw();
void find_start_node();

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

void setup()
{
#ifdef SERIAL_DEBUG
    Serial.begin(115200);
#endif
    
    // init variables and sensors
#ifndef NO_SENSORS
    left_motor = new MotorControl(MOTOR_LB, MOTOR_LA);
    right_motor = new MotorControl(MOTOR_RA, MOTOR_RB);

    distance_sensor = new SonarSensor(DISTANCE_SENSE_TRIGGER, DISTANCE_SENSE_ECHO);
    
    pinMode(LINE_SENSE_L, INPUT);
    pinMode(LINE_SENSE_R, INPUT);
    pinMode(LINE_SENSE_B, INPUT);
    
    pinMode(FLAME_SENSE, INPUT);
#endif
    
    current_node = NULL;
    current_direction = DIRECTION_INVALID;
    
    init_nodes();
    find_start_node();
    
    Path path(start_node_1);
    reachable_node(path.start, end_node, &path);
    SERIAL_PRINT("\t");
    SERIAL_PRINTLN(path.start->id);
    for (LinkedList<Edge *>::ListNode *ln = path.edges.getFirst(); ln != NULL; ln = ln->next) {
        //SERIAL_PRINTLN(ln->value->id);
        SERIAL_PRINT(ln->value->direction);
        SERIAL_PRINT("\t");
        SERIAL_PRINTLN(ln->value->node->id);
    }
}

void loop()
{
    
}

uint16_t turn_cw()
{
    uint16_t cycles_elapsed = 0;
    
    left_motor->forward(L_SPEED(getBatteryStatus()));
    right_motor->reverse(R_SPEED(getBatteryStatus()));
    while (!digitalRead(LINE_SENSE_R)) {
        cycles_elapsed++;
    }
    while (digitalRead(LINE_SENSE_R)) {
        cycles_elapsed++;
    }
    left_motor->brake();
    right_motor->brake();
    
    return cycles_elapsed;
}

uint16_t turn_ccw()
{
    uint16_t cycles_elapsed = 0;
    
    left_motor->reverse(L_SPEED(getBatteryStatus()));
    right_motor->forward(R_SPEED(getBatteryStatus()));
    while (!digitalRead(LINE_SENSE_L)) {
        cycles_elapsed++;
    }
    while (digitalRead(LINE_SENSE_L)) {
        cycles_elapsed++;
    }
    left_motor->brake();
    right_motor->brake();
    
    return cycles_elapsed;
}

// Finds starting node and aligns along the horizontal axis
void find_start_node()
{
#ifdef NO_SENSORS
    current_node = start_node_1;
    return;
#endif
    
    bool flame_on_first;
    bool flame_on_second;
    uint16_t first_cycles = 0;
    uint16_t second_cycles = 0;
    
    // initial tape alignment
    turn_cw();
    
    // measure cycles to next tape
    first_cycles = turn_cw();
    flame_on_first = digitalRead(FLAME_SENSE);
    
    // measure cycles to get back
    second_cycles = turn_cw();
    flame_on_second = digitalRead(FLAME_SENSE);
    
    if (flame_on_first && flame_on_second) {
        // something has gone wrong
        // can't determine starting node, error
        current_node = NULL;
    }
    else if (first_cycles < second_cycles) {   // on left edge of 90*
        // first cycle = to right edge
        // second cycle = to left edge
        
        if (!flame_on_first && !flame_on_second) {
            current_node = start_node_1;
            current_direction = DIRECTION_WEST;
        }
        else if (flame_on_first && !flame_on_second) {
            // starting position 2
            current_node = start_node_2;
            current_direction = DIRECTION_SOUTH;
        }
        else if (!flame_on_first && flame_on_second) {
            // starting position 3
            current_node = start_node_3;
            current_direction = DIRECTION_NORTH;
        }
    }
    else {  // on right edge of 90*
        // first cycle = to left edge
        // second cycle = to right edge
        
        if (!flame_on_first && !flame_on_second) {
            current_node = start_node_1;
            current_direction = DIRECTION_NORTH;
        }
        else if (!flame_on_first && flame_on_second) {
            // starting position 2
            current_node = start_node_2;
            current_direction = DIRECTION_WEST;
        }
        else if (flame_on_first && !flame_on_second) {
            // starting position 3
            current_node = start_node_3;
            current_direction = DIRECTION_EAST;
        }
    }
}
