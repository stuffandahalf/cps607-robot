//#define SERIAL_DEBUG      // Uncomment to enable serial output
//#define USING_INTERRUPTS  // uncomment this to switch to using interrupts, couldn't get transistor AND gate working
//#define DONT_MOVE

// Motor control pins
// Must be PWM capable
#define FB1         (6)
#define FB2         (5)
#define LR1         (9)
#define LR2         (10)

#ifdef USING_INTERRUPTS
// IR sensors are active low, AND them all
// together and use output for interrupt
#define IR_INT      (2)
#endif

#define IR_SENSE_FL (A0)    // General IR input
#define IR_SENSE_FR (A1)
#define IR_SENSE_BL (A5)
#define IR_SENSE_BR (A4)


#define IRSTATUS_CLEAR              (0u)
#define IRSTATUS_FRONT_RIGHT        (1 << 0)
#define IRSTATUS_FRONT_LEFT         (1 << 1)
#define IRSTATUS_BACK_RIGHT         (1 << 3)
#define IRSTATUS_BACK_LEFT          (1 << 4)

#define IRSTATUS_FRONT_EDGE         (IRSTATUS_FRONT_LEFT | IRSTATUS_FRONT_RIGHT)
#define IRSTATUS_BACK_EDGE          (IRSTATUS_BACK_LEFT | IRSTATUS_BACK_RIGHT)
#define IRSTATUS_LEFT_EDGE          (IRSTATUS_FRONT_LEFT | IRSTATUS_BACK_LEFT)
#define IRSTATUS_RIGHT_EDGE         (IRSTATUS_FRONT_RIGHT | IRSTATUS_BACK_RIGHT)

#define IRSTATUS_FRONT_RIGHT_EDGES  (IRSTATUS_FRONT_EDGE | IRSTATUS_BACK_RIGHT)
#define IRSTATUS_FRONT_LEFT_EDGES   (IRSTATUS_FRONT_EDGE | IRSTATUS_BACK_LEFT)
#define IRSTATUS_BACK_RIGHT_EDGES   (IRSTATUS_BACK_EDGE | IRSTATUS_FRONT_RIGHT)
#define IRSTATUS_BACK_LEFT_EDGES    (IRSTATUS_BACK_EDGE | IRSTATUS_FRONT_LEFT)

#define IRSTATUS_TRAPPED            (15u)

#define DIRECTION_NONE              (0u)
#define DIRECTION_LEFT              (1 << 0)
#define DIRECTION_RIGHT             (1 << 1)
#define DIRECTION_FORWARD           (1 << 2)
#define DIRECTION_REVERSE           (1 << 3)
#define DIRECTION_UNDEFINED         (1 << 7)

//#define RIGHT_TURN_PWM (185)
#define RIGHT_TURN_PWM (200)
#define LEFT_TURN_PWM (200)
//#define TURN_DELAY (250)
#define TURN_DELAY (100)
#define SPEED_PWM (75)

// When running the motors at a low speed, need to start with a higher PWM to avoid stalling
//#define KICKSTART_PWM   100
//#define KICKSTART_PWM   150
#define KICKSTART_PWM   125
#define KICKSTART_DELAY (255 - KICKSTART_PWM)

class MotorControl
{
private:
    uint8_t outA;
    uint8_t outB;
    
    int16_t pwm;

public:
    MotorControl(uint8_t outA, uint8_t outB)
    {
        this->outA = outA;
        this->outB = outB;
        
        pinMode(this->outA, OUTPUT);
        pinMode(this->outB, OUTPUT);
        
        brake();
    }
    
    inline void resume() { this->forward(); }
    
    inline void forward() { this->forward(this->pwm); }
    inline void forward(int16_t pwm)
    {
        if (pwm < 0) {
            reverse(pwm * -1);
            return;
        }
        this->pwm = pwm & 0xFF;
        
        digitalWrite(this->outB, LOW);
        analogWrite(this->outA, KICKSTART_PWM);
        delay(KICKSTART_DELAY);
        analogWrite(this->outA, pwm & 0xFF);
    }
    
    inline void reverse() { this->forward(this->pwm * -1); }
    inline void reverse(int16_t pwm)
    {
        if (pwm < 0) {
            forward(pwm * -1);
            return;
        }
        this->pwm = (pwm & 0xFF) * -1;
        digitalWrite(this->outA, LOW);
        analogWrite(this->outB, KICKSTART_PWM);
        delay(KICKSTART_DELAY);
        analogWrite(this->outB, pwm & 0xFF);
    }
    
    inline void brake()
    {
        digitalWrite(this->outA, HIGH);
        digitalWrite(this->outB, HIGH);
    }
    
    inline void standby()
    {
        digitalWrite(this->outA, LOW);
        digitalWrite(this->outB, LOW); 
    }
    
    inline int16_t getPwm() { return this->pwm; }
};

uint8_t getIRStatus()
{
    uint8_t state = IRSTATUS_CLEAR;
    if (digitalRead(IR_SENSE_FL)) {
        state |= IRSTATUS_FRONT_LEFT;
    }
    if (digitalRead(IR_SENSE_FR)) {
        state |= IRSTATUS_FRONT_RIGHT;
    }
    if (digitalRead(IR_SENSE_BL)) {
        state |= IRSTATUS_BACK_LEFT;
    }
    if (digitalRead(IR_SENSE_BR)) {
        state |= IRSTATUS_BACK_RIGHT;
    }
    return state;
}

void printIRStatus(uint8_t status)
{
    bool any = false;
    bool first = true;
    
#ifdef SERIAL_DEBUG
    if (status == IRSTATUS_CLEAR) {
        Serial.println("CLEAR");
        return;
    }

    if (status & IRSTATUS_FRONT_LEFT) {
        any = true;
        Serial.print("FL");
        first = false;
    }
    if (status & IRSTATUS_FRONT_RIGHT) {
        any = true;
        if (first) {
            first = false;
        }
        else {
            Serial.print(" | ");
        }
        Serial.print("FR");
    }
    if (status & IRSTATUS_BACK_LEFT) {
        any = true;
        if (first) {
            first = false;
        }
        else {
            Serial.print(" | ");
        }
        Serial.print("BL");
    }
    if (status & IRSTATUS_BACK_RIGHT) {
        any = true;
        if (first) {
            first = false;
        }
        else {
            Serial.print(" | ");
        }
        Serial.print("BR");
    }
    if (any) {
        Serial.println();
    }
#endif
}

#ifdef USING_INTERRUPTS
bool interrupted;
bool handled;
#endif
uint8_t lastDirection;
MotorControl *mcForwardBackward;
MotorControl *mcLeftRight;

#ifdef USING_INTERRUPTS
void IR_ISR()   // interrupt service routine when an IR sensor goes active
{
    mcForwardBackward->brake();
    mcLeftRight->brake();
    interrupted = true;
    handled = false;
}
#endif

void setup()
{
#ifdef SERIAL_DEBUG
    Serial.begin(115200);
#endif

    mcForwardBackward = new MotorControl(FB1, FB2);
    mcLeftRight = new MotorControl(LR1, LR2);
    
    pinMode(IR_SENSE_FL, INPUT);
    pinMode(IR_SENSE_FR, INPUT);
    pinMode(IR_SENSE_BL, INPUT);
    pinMode(IR_SENSE_BR, INPUT);
    
#ifdef USING_INTERRUPTS
    interrupted = false;
    handled = false;
    
    pinMode(IR_INT, INPUT);
    attachInterrupt(digitalPinToInterrupt(IR_INT), IR_ISR, FALLING);
#endif
    
    lastDirection = DIRECTION_FORWARD;
    mcLeftRight->brake();
#ifdef DONT_MOVE
    mcForwardBackward->brake();
#else
    mcForwardBackward->forward(SPEED_PWM);
#endif
}

// left = forward
// right = reverse

void loop()
{
#ifdef DONT_MOVE
    return;
#endif

#ifdef USING_INTERRUPTS
    if (!interrupted) {
        return;
    }
    if (handled) {
        return;
    }
#endif

    uint8_t direction = DIRECTION_UNDEFINED;
    uint16_t ir_status;
    
#ifdef SERIAL_DEBUG
    ir_status = getIRStatus();
    printIRStatus(ir_status);
#endif
    if ((ir_status = getIRStatus()) != IRSTATUS_CLEAR) {
#ifndef USING_INTERRUPTS
        //mcForwardBackward->brake();
#endif
        switch (ir_status) {
        // corner cases
        case IRSTATUS_FRONT_LEFT:
            if (mcForwardBackward->getPwm() >= 0) {
                mcForwardBackward->reverse(SPEED_PWM);
                direction = DIRECTION_REVERSE;
                if (lastDirection & DIRECTION_LEFT) {
                    mcLeftRight->reverse(RIGHT_TURN_PWM);
                    direction |= DIRECTION_RIGHT;
                }
                else {
                    mcLeftRight->forward(LEFT_TURN_PWM);
                    direction |= DIRECTION_LEFT;
                }
                delay(TURN_DELAY);
            }
            break;
        case IRSTATUS_FRONT_RIGHT:
            if (mcForwardBackward->getPwm() >= 0) {
                mcForwardBackward->reverse(SPEED_PWM);
                direction = DIRECTION_REVERSE;
                if (lastDirection & DIRECTION_RIGHT) {
                    mcLeftRight->forward(LEFT_TURN_PWM);
                    direction |= DIRECTION_LEFT;
                }
                else {
                    mcLeftRight->reverse(RIGHT_TURN_PWM);
                    direction |= DIRECTION_RIGHT;
                }
                delay(TURN_DELAY);
            }
            break;
        case IRSTATUS_BACK_LEFT:
            if (mcForwardBackward->getPwm() <= 0) {
                mcForwardBackward->forward(SPEED_PWM);
                direction = DIRECTION_FORWARD;
                if (lastDirection & DIRECTION_RIGHT) {
                    mcLeftRight->forward(LEFT_TURN_PWM);
                    direction |= DIRECTION_LEFT;
                }
                else {
                    mcLeftRight->reverse(RIGHT_TURN_PWM);
                    direction |= DIRECTION_RIGHT;
                }
                delay(TURN_DELAY);
            }
            break;
        case IRSTATUS_BACK_RIGHT:
            if (mcForwardBackward->getPwm() <= 0) {
                mcForwardBackward->forward(SPEED_PWM);
                direction = DIRECTION_FORWARD;
                if (lastDirection & DIRECTION_LEFT) {
                    mcLeftRight->reverse(RIGHT_TURN_PWM);
                    direction |= DIRECTION_RIGHT;
                }
                else {
                    mcLeftRight->forward(LEFT_TURN_PWM);
                    direction |= DIRECTION_LEFT;
                }
                delay(TURN_DELAY);
            }
            break;
        
        // edge cases
        case IRSTATUS_FRONT_EDGE:
            if (mcForwardBackward->getPwm() >= 0) {
                mcForwardBackward->reverse(SPEED_PWM);
                mcLeftRight->brake();
                direction = DIRECTION_REVERSE;
            }
            break;
        case IRSTATUS_BACK_EDGE:
            if (mcForwardBackward->getPwm() <= 0) {
                mcForwardBackward->forward(SPEED_PWM);
                mcLeftRight->brake();
                direction = DIRECTION_FORWARD;
            }
            break;
        case IRSTATUS_LEFT_EDGE:    // swap forwards / reverse?
            mcLeftRight->reverse(RIGHT_TURN_PWM);
            direction &= ~(DIRECTION_LEFT | DIRECTION_RIGHT);
            direction |= DIRECTION_RIGHT;
            delay(TURN_DELAY);
            break;
        case IRSTATUS_RIGHT_EDGE:
            mcLeftRight->forward(LEFT_TURN_PWM);
            direction &= ~(DIRECTION_LEFT | DIRECTION_RIGHT);
            direction |= DIRECTION_LEFT;
            delay(TURN_DELAY);
            break;
            
        // 2 edges
        case IRSTATUS_FRONT_LEFT_EDGES: // reverse and go right
#if 0
            mcForwardBackward->brake();
#else
            mcForwardBackward->reverse(SPEED_PWM);
            mcLeftRight->reverse(RIGHT_TURN_PWM);
            delay(TURN_DELAY / 2);
            mcForwardBackward->reverse();
#endif
            break;
        case IRSTATUS_FRONT_RIGHT_EDGES:
#if 1
            mcForwardBackward->brake();
#else
            //mcLeftRight->reverse(TURN_PWM);
            //mcForwardBackward->reverse();
#endif
            break;
        case IRSTATUS_BACK_LEFT_EDGES:
            mcForwardBackward->forward(SPEED_PWM);
            mcLeftRight->reverse(RIGHT_TURN_PWM);
            direction = DIRECTION_FORWARD | DIRECTION_RIGHT;
            delay(TURN_DELAY);
            break;
        case IRSTATUS_BACK_RIGHT_EDGES:
            mcForwardBackward->forward(SPEED_PWM);
            mcLeftRight->forward(LEFT_TURN_PWM);
            direction = DIRECTION_FORWARD | DIRECTION_LEFT;
            delay(TURN_DELAY);
            break;
            
        case IRSTATUS_TRAPPED:
            // Trapped, nowhere to move, will resume if relocated
            break;
        }
        //mcLeftRight->brake();
        //mcForwardBackward->resume();
    }
    else {
        mcLeftRight->brake();
    }
    
    if (direction != DIRECTION_UNDEFINED) {
        lastDirection = direction;
    }
    
#ifdef USING_INTERRUPTS
    handled = true;
#endif
}
