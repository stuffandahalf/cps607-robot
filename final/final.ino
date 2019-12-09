#include "MotorControl.h"
#include "Node.h"
#include "SonarSensor.h"
#include "ServoTimer2.h"
//#include <Servo.h>

// logic control macros
#define SERIAL_DEBUG
//#define NO_SENSORS

#define USE_BATTERY_SENSE

// peripheral pin macros
#define MOTOR_RA    (10)
#define MOTOR_RB    (9)
#define MOTOR_LA    (6)
#define MOTOR_LB    (5)

#define FLAME_SENSE     (A5)

#define LINE_SENSE_L    (4)
#define LINE_SENSE_R    (2)
#define LINE_SENSE_B    (3)

#define DISTANCE_SENSE_TRIGGER  (8)
#define DISTANCE_SENSE_ECHO     (7)

#define LIFT_SERVO      (11)

#define BATTERY_SENSE   (A6)

#define LED             (13)

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

#define SERVO_LOW   (1000)
#define SERVO_HIGH  (2000)

MotorControl *left_motor;
MotorControl *right_motor;

SonarSensor *distance_sensor;

ServoTimer2 lift_servo;

Node *current_node;
Direction current_direction;

#define SEARCH_NODE_COUNT   (5)
Node **search_path;

uint16_t turn_cw();
uint16_t turn_ccw();
void find_start_node();
void set_search_path();
void move(Edge *edge);

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
    
    lift_servo.attach(LIFT_SERVO);
    lift_servo.read();
    lift_servo.write(SERVO_LOW);
    
    pinMode(LINE_SENSE_L, INPUT);
    pinMode(LINE_SENSE_R, INPUT);
    pinMode(LINE_SENSE_B, INPUT);
    
    pinMode(FLAME_SENSE, INPUT);
#endif

    pinMode(LED, OUTPUT);
    digitalWrite(LED, LOW);
    
    current_node = NULL;
    current_direction = DIRECTION_INVALID;
    
    init_nodes();
    //find_start_node();
    
    /*Path path(start_node_1);
    SERIAL_PRINTLN(F("HELLO WORLD"));
    reachable_node(path.start, end_node, &path);
    SERIAL_PRINT(F("\t"));
    SERIAL_PRINTLN(path.start->id);
    for (LinkedList<Edge *>::ListNode *ln = path.edges.getFirst(); ln != NULL; ln = ln->next) {
        //SERIAL_PRINTLN(ln->value->id);
        SERIAL_PRINT(ln->value->direction);
        SERIAL_PRINT(F("\t"));
        SERIAL_PRINTLN(ln->value->node->id);
    }*/
    
    /*for (int i = 0; i < 180; i++) {
        lift_servo.write(i);
    }*/
    //lift_servo.write(0);
    
    
    /*for (int i = 0; i < 1000; i++) {
        lift_servo.write(lift_servo.read() + 1);
        //delay(10);
    }*/
    /*lift_servo.write(SERVO_HIGH);
    delay(3000);
    //lift_servo.read();
    lift_servo.write(SERVO_LOW);*/
    
    /*left_motor->forward(L_SPEED(getBatteryStatus()));
    right_motor->forward(R_SPEED(getBatteryStatus()));
    delay(3000);
    left_motor->brake();
    right_motor->brake();*/
    
    /*for(;;) {
        //SERIAL_PRINTLN(lift_servo.read());
        lift_servo.write(lift_servo.read() - 1);
    }*/
    
    for (;;) {
        SERIAL_PRINT(digitalRead(LINE_SENSE_L));
        SERIAL_PRINT('\t');
        SERIAL_PRINT(digitalRead(LINE_SENSE_B));
        SERIAL_PRINT('\t');
        SERIAL_PRINTLN(digitalRead(LINE_SENSE_R));
    }
    
    for(;;);
}

void loop()
{
    SERIAL_PRINTLN(F("node\tdirection"));
    SERIAL_PRINT(current_node->id);
    SERIAL_PRINT(F("\t"));
    SERIAL_PRINTLN(current_direction);
    
    set_search_path();
    for (int i = 0; i < SEARCH_NODE_COUNT; i++) {
        Path current_path(current_node);
        Node *end_node = search_path[i];
        reachable_node(current_node, end_node, &current_path);
        
        for (LinkedList<Edge *>::ListNode *ln = current_path.edges.getFirst(); ln != NULL; ln = ln->next) {
            move(ln->value);
            if (ln->value->node != end_node) {
                // check for flame and if found, recover ball
            }
            
            SERIAL_PRINT(current_node->id);
            SERIAL_PRINT(F("\t"));
            SERIAL_PRINTLN(current_direction);
        }
    }
    for (;;);
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

void realign()
{
    if (digitalRead(LINE_SENSE_L)) {
        while (digitalRead(LINE_SENSE_L)) {
            left_motor->brake();
        }
        left_motor->forward(L_SPEED(getBatteryStatus()));
    }
    if (digitalRead(LINE_SENSE_R)) {
        while (digitalRead(LINE_SENSE_R)) {
            right_motor->brake();
        }
        right_motor->forward(R_SPEED(getBatteryStatus()));
    }
}

void move(Edge *edge)
{
    int delta = direction_delta(current_direction, edge->direction);
    void (*turn_func)() = &turn_ccw;
    
    while (delta != 0) {
        Edge *nearest_edge = current_node->edges;
        int nearest_edge_delta = direction_delta(current_direction, nearest_edge->direction);
        for (uint8_t i = 0; i < current_node->edge_count; i++) {
            int this_edge_delta = direction_delta(current_direction, current_node->edges[i].direction);
            
            if (this_edge_delta > 0 && (nearest_edge_delta <= 0 || this_edge_delta < nearest_edge_delta)) {
                nearest_edge = &current_node->edges[i];
                nearest_edge_delta = this_edge_delta;
            }
        }
        
        //turn_func();
        current_direction = nearest_edge->direction;
        delta = direction_delta(current_direction, edge->direction);
    }
    
    
    left_motor->forward(L_SPEED(getBatteryStatus()));
    right_motor->forward(R_SPEED(getBatteryStatus()));
    while (!digitalRead(LINE_SENSE_R)) {
        realign();
    }
    while (digitalRead(LINE_SENSE_R)) {
        realign();
    }
    left_motor->brake();
    right_motor->brake();
    
    current_node = edge->node;
    current_direction = edge->direction;
    
}

// Finds starting node and aligns along the horizontal axis
void find_start_node()
{
#ifdef NO_SENSORS
    current_direction = DIRECTION_NORTH;
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

void set_search_path()
{
    if (current_node == start_node_1) {
        search_path = new Node *[SEARCH_NODE_COUNT] {
            start_node_3,
            left_intersection,
            right_intersection,
            start_node_2,
            end_node
        };
    }
    else if (current_node == start_node_2) {
        search_path = new Node *[SEARCH_NODE_COUNT] {
            end_node,
            left_intersection,
            right_intersection,
            start_node_1,
            start_node_3
        };
    }
    else if (current_node == start_node_3) {
        search_path = new Node *[SEARCH_NODE_COUNT] {
            start_node_1,
            right_intersection,
            left_intersection,
            end_node,
            start_node_2
        };
    }
    else if (current_node == end_node) {
        search_path = new Node *[SEARCH_NODE_COUNT] {
            start_node_2,
            right_intersection,
            left_intersection,
            start_node_3,
            start_node_1
        };
    }
}
