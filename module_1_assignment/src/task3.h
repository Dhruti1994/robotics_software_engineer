#ifndef TASK3_H
#define TASK3_H

#include <iostream>
#include <string>

template <typename T>
class Sensor {
private:
    T reading;

public:
    
    Sensor(T value) : reading(value) {}

    
    T getReading() {
        return reading;
    }

    
    void displayReading(const std::string& sensorType) {
        std::cout << sensorType << " Reading: " << reading << std::endl;
    }
};


class TemperatureSensor : public Sensor<double> {
public:
    
    TemperatureSensor(double temp) : Sensor<double>(temp) {}
};


class DistanceSensor : public Sensor<double> {
public:
    
    DistanceSensor(double dist) : Sensor<double>(dist) {}
};

#endif 
