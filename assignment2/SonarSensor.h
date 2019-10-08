#ifndef SONAR_SENSOR_H
#define SONAR_SENSOR_H

#define SPEED_OF_SOUND (.343)

class SonarSensor {
private:
    byte trigger;
    byte echo;

public:
    SonarSensor(byte trigger, byte echo)
    {
        this->trigger = trigger;
        this->echo = echo;
        
        pinMode(this->trigger, OUTPUT);
        pinMode(this->echo, INPUT);
        
        digitalWrite(this->trigger, LOW);
    }
    
    int16_t getDistance()
    {
        digitalWrite(this->trigger, HIGH);
        delayMicroseconds(10);
        digitalWrite(this->trigger, LOW);
        
        unsigned long duration = pulseIn(this->echo, HIGH);
        if (!duration) {
            return -1;
        }
        
        int16_t distance = duration * SPEED_OF_SOUND; // distance is now in millimeters
        
        return distance;
    }
};

#endif
