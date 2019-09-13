#define FORWARD     (5)     // These pins need to be PWM capable
#define BACKWARD    (6)
#define LEFT        (9)
#define RIGHT       (10)

#define IR_INT  (2)         // IR sensors ORed to create interrupt signal;

#define IR_SENSE_FL (4)     // General IR input
#define IR_SENSE_FR (7)
#define IR_SENSE_BL (8)
#define IR_SENSE_BR (11)

#ifdef SERIAL_DEBUG
#define PRINT(...) Serial.println(__VA_ARGS__)
#else
#define PRINT(...)
#endif


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

    inline void forward(int16_t pwm)
    {
        if (pwm < 0) {
            reverse(pwm * -1);
            return;
        }
        this->pwm = pwm;
        digitalWrite(this->outB, LOW);
        analogWrite(this->outB, pwm);
    }
    
    inline void reverse(int16_t pwm)
    {
        if (pwm < 0) {
            forward(pwm * -1);
            return;
        }
        this->pwm = pwm * -1;
        digitalWrite(this->outA, LOW);
        analogWrite(this->outB, pwm);
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
    
    inline int16_t getPwm() { return this-<pwm; }
};

struct
{
    struct
    {
        struct
        {
            bool left;
            bool right;
        } front;
        struct
        {
            bool left;
            bool right;
        } back;
    } edge;
    
public:
    bool any()
    {
        uint8_t *bytePtr = (uint8_t *)this;
        for (int i = 0; i < sizeof(robotSensors); i++) {
            if (bytePtr[i]) {
                return true;
            }
        }
        return false;
    }
    
} robotSensors;

MotorControl *mcForwardBackward;
MotorControl *mcLeftRight;

void IR_ISR()
{
    mcForwardBackward->brake();
    mcLeftRight->brake();
    
    robotSensors.edge.front.left = digitalRead(IR_SENSE_FL);
    robotSensors.edge.front.right = digitalRead(IR_SENSE_FR);
    robotSensors.edge.back.left = digitalRead(IR_SENSE_BL);
    robotSensors.edge.back.right = digitalRead(IR_SENSE_BR);
}

void setup()
{
#ifdef SERIAL_DEBUG
    Serial.begin(115200);
#endif
    
    mcForwardBackward = new MotorControl(FORWARD, BACKWARD);
    mcLeftRight = new MotorControl(LEFT, RIGHT);
    
    pinMode(IR_INT, INPUT);
    attachInterrupt(digitalPinToInterrupt(IR_INT), IR_ISR, FALLING);
    
    pinMode(IR_SENSE1, INPUT);
}

void loop()
{
    PRINT(
    //Serial.println(digitalRead(LINE_SENSE1));
}
