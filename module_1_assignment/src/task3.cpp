#include "task3.h"

int main() {
    
    TemperatureSensor tempSensor(25.5); 
    DistanceSensor distSensor(150.0);  

    
    tempSensor.displayReading("Temperature");
    distSensor.displayReading("Distance");

    
    Sensor<std::string> stringSensor("Active");
    Sensor<char> charSensor('A');

    
    stringSensor.displayReading("String Sensor");
    charSensor.displayReading("Char Sensor");

    return 0;
}
