#define FORWARD     (5)
#define BACKWARD    (6)
#define LEFT        (9)
#define RIGHT       (10)

#define LINE_SENSE1 (2)

class MotorControl
{
private:
    uint8_t outA;
    uint8_t outB;

public:
    MotorControl(uint8_t outA, uint8_t outB)
    {
        this->outA = outA;
        this->outB = outB;
        
        pinMode(this->outA, OUTPUT);
        pinMode(this->outB, OUTPUT);
        
        brake();
    }

    inline void forward(uint16_t pwm)
    {
        digitalWrite(this->outB, LOW);
        analogWrite(this->outB, pwm);
    }
    
    inline void reverse(uint16_t pwm)
    {
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
};

MotorControl *mcForwardBackward;
MotorControl *mcLeftRight;

void setup()
{
    Serial.begin(115200);
    
    mcForwardBackward = new MotorControl(FORWARD, BACKWARD);
    mcLeftRight = new MotorControl(LEFT, RIGHT);
    
    pinMode(LINE_SENSE1, INPUT);
}

void loop()
{
    Serial.println(digitalRead(LINE_SENSE1));
}
