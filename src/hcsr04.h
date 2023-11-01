#ifndef HCSR04_H
#define HCSR04_H

#include <Arduino.h>

class hcsr04
{
private:
    byte _trigPin;
    byte _echoPin;
    float _distance;

public:
    hcsr04(byte trigPin, byte echoPin);
    void init();
    float getDistance();
};

// constructor, set trigPin and echoPin
hcsr04::hcsr04(byte trigPin, byte echoPin)
{
    this->_trigPin = trigPin;
    this->_echoPin = echoPin;
    init();
}

// init function
void hcsr04::init()
{
    pinMode(this->_trigPin, OUTPUT);
    pinMode(this->_echoPin, INPUT);
}

// get distance function, return distance in cm
float hcsr04::getDistance()
{
    digitalWrite(this->_trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(this->_trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(this->_trigPin, LOW);
    _distance = pulseIn(this->_echoPin, HIGH) / 58.2;
    return _distance;
}

#endif