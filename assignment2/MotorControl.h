#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

class MotorControl {
private:
    uint8_t outA;
    uint8_t outB;
    
    int16_t pwm;
    bool brakeStatus;
    
    //uint8_t kickstartPwm;
    //uint8_t kickstartDelay;

public:
    MotorControl(uint8_t outA, uint8_t outB)
    {
        //this->kickstartPwm = KICKSTART_PWM;
        //this->kickstartDelay = KICKSTART_DELAY;
        
        this->outA = outA;
        this->outB = outB;
        
        pinMode(this->outA, OUTPUT);
        pinMode(this->outB, OUTPUT);
        
        brake();
    }
    
    /*MotorControl(uint8_t outA, uint8_t outB, uint8_t kickstartPwm, uint8_t kickstartDelay) : MotorControl(outA, outB)
    {
        this->kickstartPwm = kickstartPwm;
        this->kickstartDelay = kickstartDelay;
    }*/
    
    //inline void resume() { this->forward(); }
    
    //inline void forward() { this->forward(this->pwm); }
    inline void forward(int16_t pwm)
    {
        if (pwm == 0) {
            this->brake();
            return;
        }
        if (pwm < 0) {
            reverse(pwm * -1);
            return;
        }
        this->pwm = pwm & 0xFF;
        
        digitalWrite(this->outB, LOW);
        /*analogWrite(this->outA, KICKSTART_PWM);
        delay(KICKSTART_DELAY);*/
        analogWrite(this->outA, pwm & 0xFF);
        this->brakeStatus = false;
    }
    
    //inline void reverse() { this->forward(this->pwm * -1); }
    inline void reverse(int16_t pwm)
    {
        if (pwm < 0) {
            forward(pwm * -1);
            return;
        }
        this->pwm = (pwm & 0xFF) * -1;
        digitalWrite(this->outA, LOW);
        /*analogWrite(this->outB, KICKSTART_PWM);
        delay(KICKSTART_DELAY);*/
        analogWrite(this->outB, pwm & 0xFF);
        this->brakeStatus = false;
    }
    
    inline void brake()
    {
        digitalWrite(this->outA, HIGH);
        digitalWrite(this->outB, HIGH);
        /*this->forward(this->pwm * -1);
        delayMicroseconds(100);
        digitalWrite(this->outA, LOW);
        digitalWrite(this->outB, LOW);*/
        this->brakeStatus = true;
    }
    
    inline void standby()
    {
        digitalWrite(this->outA, LOW);
        digitalWrite(this->outB, LOW); 
    }
    
    inline bool getBrakeStatus() { return this->brakeStatus; }
    inline int16_t getPwm() { return this->pwm; }
};

#endif
