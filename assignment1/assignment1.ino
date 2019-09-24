// Motor control pins
// Must be PWM capable
#define FB1         (6)
#define FB2         (5)
#define LR1         (9)
#define LR2         (10)

//#define USING_INTERRUPTS  // uncomment this to swittch to using interrupts, couldn't get transistor AND gate working

#ifdef USING_INTERRUPTS
// IR sensors are active low, AND them all
// together and use output for interrupt
#define IR_INT      (2)
#endif

#define IR_SENSE_FL (A0)    // General IR input
#define IR_SENSE_FR (A1)
#define IR_SENSE_BL (A5)
#define IR_SENSE_BR (A4)

#define SERIAL_DEBUG

#ifdef SERIAL_DEBUG
#define PRINT(...) Serial.println(__VA_ARGS__)
#else
#define PRINT(...)
#endif

#define IRSTATUS_CLEAR              (0u)
#define IRSTATUS_FRONT_RIGHT        (1u)
#define IRSTATUS_FRONT_LEFT         (2u)
#define IRSTATUS_BACK_RIGHT         (4u)
#define IRSTATUS_BACK_LEFT          (8u)

#define IRSTATUS_FRONT_EDGE         (IRSTATUS_FRONT_LEFT | IRSTATUS_FRONT_RIGHT)
#define IRSTATUS_BACK_EDGE          (IRSTATUS_BACK_LEFT | IRSTATUS_BACK_RIGHT)
#define IRSTATUS_LEFT_EDGE          (IRSTATUS_FRONT_LEFT | IRSTATUS_BACK_LEFT)
#define IRSTATUS_RIGHT_EDGE         (IRSTATUS_FRONT_RIGHT | IRSTATUS_BACK_RIGHT)

#define IRSTATUS_FRONT_RIGHT_EDGES  (IRSTATUS_FRONT_EDGE | IRSTATUS_BACK_RIGHT)
#define IRSTATUS_FRONT_LEFT_EDGES   (IRSTATUS_FRONT_EDGE | IRSTATUS_BACK_LEFT)
#define IRSTATUS_BACK_RIGHT_EDGES   (IRSTATUS_BACK_EDGE | IRSTATUS_FRONT_RIGHT)
#define IRSTATUS_BACK_LEFT_EDGES    (IRSTATUS_BACK_EDGE | IRSTATUS_FRONT_LEFT)

#define IRSTATUS_TRAPPED            (15u)

//#define TURN_PWM (80)
//#define TURN_PWM (255)
#define TURN_PWM (200)
#define SPEED_PWM (70)

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
        analogWrite(this->outA, 100);
        delay(75);
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
        analogWrite(this->outB, 100);
        delay(75);
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

    mcLeftRight->brake();
    mcForwardBackward->forward(SPEED_PWM);
}

// left = forward
// right = reverse

void loop()
{
#ifdef USING_INTERRUPTS
    if (!interrupted) {
        return;
    }
    if (handled) {
        return;
    }
#endif
    uint16_t ir_status = getIRStatus();
    printIRStatus(ir_status);
    
    if ((ir_status = getIRStatus()) != IRSTATUS_CLEAR) {
#ifndef USING_INTERRUPTS
        //mcForwardBackward->brake();
#endif
        switch (ir_status) {
        // corner cases
        case IRSTATUS_FRONT_LEFT:
            if (mcForwardBackward->getPwm() >= 0) {
                mcForwardBackward->reverse();
                mcLeftRight->forward(TURN_PWM);
            }
            break;
        case IRSTATUS_FRONT_RIGHT:
            if (mcForwardBackward->getPwm() >= 0) {
                mcForwardBackward->reverse();
                mcLeftRight->reverse(TURN_PWM);
            }
            break;
        case IRSTATUS_BACK_LEFT:
            if (mcForwardBackward->getPwm() <= 0) {
                mcForwardBackward->reverse();
                mcLeftRight->reverse(TURN_PWM);
            }
            break;
        case IRSTATUS_BACK_RIGHT:
            if (mcForwardBackward->getPwm() <= 0) {
                mcForwardBackward->reverse();
                mcLeftRight->forward(TURN_PWM);
            }
            break;
        
        // edge cases
        case IRSTATUS_FRONT_EDGE:
            if (mcForwardBackward->getPwm() >= 0) {
                mcForwardBackward->reverse();
                mcLeftRight->brake();
            }
            break;
        case IRSTATUS_BACK_EDGE:
            if (mcForwardBackward->getPwm() <= 0) {
                mcForwardBackward->reverse();
                mcLeftRight->brake();
            }
            break;
        //case IRSTATUS_LEFT_EDGE:
            //mcLeftRight->reverse(TURN_PWM);
            //break;
        //case IRSTATUS_RIGHT_EDGE:
            //mcLeftRight->forward(TURN_PWM);
            //break;
            
        //// 2 edges
        //case IRSTATUS_FRONT_LEFT_EDGES:
            //mcLeftRight->forward(TURN_PWM);
            //mcForwardBackward->reverse();
            //break;
        //case IRSTATUS_FRONT_RIGHT_EDGES:
            //mcLeftRight->reverse(TURN_PWM);
            //mcForwardBackward->reverse();
            //break;
        //case IRSTATUS_BACK_LEFT_EDGES:
            //mcLeftRight->forward(TURN_PWM);
            //mcForwardBackward->reverse();
            //break;
        //case IRSTATUS_BACK_RIGHT_EDGES:
            //mcLeftRight->reverse(TURN_PWM);
            //mcForwardBackward->reverse();
            //break;
            
        //case IRSTATUS_TRAPPED:
            //// Trapped, nowhere to move, will resume if relocated
            //break;
        }
        //mcLeftRight->brake();
        //mcForwardBackward->resume();
    }
    else {
        mcLeftRight->brake();
    }
    
#ifdef USING_INTERRUPTS
    handled = true;
#endif
}
