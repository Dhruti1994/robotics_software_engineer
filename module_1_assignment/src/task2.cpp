#include <iostream>

class Robot {
private:
    float temperature; 
    int distance; 

public:
    
    Robot(float temp, int dist) : temperature(temp), distance(dist) {}

    
    void displayTemperature() {
        std::cout << "Temperature: " << temperature << "°C" << std::endl;
    }

    
    void displayDistance() {
        std::cout << "Distance: " << distance << " cm" << std::endl;
    }
};

int main() {
    
    Robot robot(20.0, 100);

    robot.displayTemperature();
    robot.displayDistance();

    return 0;
}
