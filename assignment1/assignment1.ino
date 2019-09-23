// Motor control pins
// Must be PWM capable
#define FB1         (6)
#define FB2         (5)
#define LR1         (9)
#define LR2         (10)

// IR sensors are active low, AND them all
// together and use output for interrupt
#define IR_INT      (2)

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

#define TURN_PWM (80)

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
    if (!digitalRead(IR_SENSE_FL)) {
        state |= IRSTATUS_FRONT_LEFT;
    }
    if (!digitalRead(IR_SENSE_FR)) {
        state |= IRSTATUS_FRONT_RIGHT;
    }
    if (!digitalRead(IR_SENSE_BL)) {
        state |= IRSTATUS_BACK_LEFT;
    }
    if (!digitalRead(IR_SENSE_BR)) {
        state |= IRSTATUS_BACK_RIGHT;
    }
    return state;
}

bool interrupted;
bool handled;
MotorControl *mcForwardBackward;
MotorControl *mcLeftRight;

void IR_ISR()   // interrupt service routine when an IR sensor goes active
{
    mcForwardBackward->brake();
    mcLeftRight->brake();
    interrupted = true;
    handled = false;
}

void setup()
{
#ifdef SERIAL_DEBUG
    Serial.begin(115200);
#endif

    mcForwardBackward = new MotorControl(FB1, FB2);
    mcLeftRight = new MotorControl(LR1, LR2);
    interrupted = false;
    handled = false;
    
    pinMode(IR_INT, INPUT);
    
    pinMode(IR_SENSE_FL, INPUT);
    pinMode(IR_SENSE_FR, INPUT);
    pinMode(IR_SENSE_BL, INPUT);
    pinMode(IR_SENSE_BR, INPUT);
    
    attachInterrupt(digitalPinToInterrupt(IR_INT), IR_ISR, FALLING);
    
    //pinMode(IR_SENSE1, INPUT);
    
    /*mcForwardBackward->forward(64);
    delay(750);
    mcForwardBackward->brake();
    //mcForwardBackward->standby();
    delay(100);
    mcForwardBackward->reverse(64);
    delay(750);
    //mcForwardBackward->standby();
    mcForwardBackward->brake();*/
    mcForwardBackward->forward(50);
    
    /*mcLeftRight->forward(255);
    delay(250);
    mcLeftRight->reverse(255);
    delay(250);
    mcLeftRight->brake();*/
    //delay(2000);
    //mcLeftRight->brake();
}

// left = reverse
// right = forward

void loop()
{
    if (!interrupted) {
        return;
    }
    if (handled) {
        return;
    }
    uint16_t ir_status;
    while ((ir_status = getIRStatus()) != IRSTATUS_CLEAR) {
        mcLeftRight->brake();   // might need to reconsider
        switch (ir_status) {
        // corner cases
        case IRSTATUS_FRONT_LEFT:
            mcForwardBackward->reverse();
            mcLeftRight->reverse(TURN_PWM);
            /*delay(750);
            mcLeftRight->brake();
            mcForwardBackward->forward();*/
            break;
        case IRSTATUS_FRONT_RIGHT:
            mcForwardBackward->reverse();
            mcLeftRight->forward(TURN_PWM);
            /*delay(750);
            mcLeftRight->brake();
            mcForwardBackward->forward();*/
            break;
        case IRSTATUS_BACK_LEFT:
            mcForwardBackward->reverse();
            mcLeftRight->reverse(TURN_PWM);
            break;
        case IRSTATUS_BACK_RIGHT:
            mcForwardBackward->reverse();
            mcLeftRight->forward(TURN_PWM);
            break;
        
        // edge cases
        case IRSTATUS_FRONT_EDGE:
        case IRSTATUS_BACK_EDGE:
            mcForwardBackward->reverse();
            break;
        case IRSTATUS_LEFT_EDGE:
            mcLeftRight->reverse(TURN_PWM);
            break;
        case IRSTATUS_RIGHT_EDGE:
            mcLeftRight->forward(TURN_PWM);
            break;
            
        // 2 edges
        case IRSTATUS_FRONT_LEFT_EDGES:
            mcLeftRight->forward(TURN_PWM);
            mcForwardBackward->reverse();
            break;
        case IRSTATUS_FRONT_RIGHT_EDGES:
            mcLeftRight->reverse(TURN_PWM);
            mcForwardBackward->reverse();
            break;
        case IRSTATUS_BACK_LEFT_EDGES:
            mcLeftRight->forward(TURN_PWM);
            mcForwardBackward->reverse();
            break;
        case IRSTATUS_BACK_RIGHT_EDGES:
            mcLeftRight->reverse(TURN_PWM);
            mcForwardBackward->reverse();
            break;
            
        case IRSTATUS_TRAPPED:
            // Trapped, nowhere to move, will resume if relocated
            break;
        }
    }
    mcLeftRight->brake();
    mcForwardBackward->resume();
    
    handled = true;
}
