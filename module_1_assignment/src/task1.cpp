#include <iostream>
#include <string>

namespace RobotNamespace {


    class Robot {
    private:
        std::string name;
        float speed; 
        float weight; 
        float size;
        int numberOfSensors;

    public:
        
        Robot(std::string robotName, float robotSpeed, float robotWeight, float robotSize, int sensors)
            : name(robotName), speed(robotSpeed), weight(robotWeight), size(robotSize), numberOfSensors(sensors) {}

        
        void moveForward() {
            std::cout << name << " is moving forward at " << speed << " m/s." << std::endl;
        }

    
        void moveBackward() {
            std::cout << name << " is moving backward at " << speed << " m/s." << std::endl;
        }

    
        void stop() {
            std::cout << name << " has stopped." << std::endl;
        }

        
        void displayDetails() {
            std::cout << "Robot Name: " << name << std::endl;
            std::cout << "Speed: " << speed << " m/s" << std::endl;
            std::cout << "Weight: " << weight << " kg" << std::endl;
            std::cout << "Size: " << size << " cubic meters" << std::endl;
            std::cout << "Number of Sensors: " << numberOfSensors << std::endl;
        }
    };
}

int main() {
    
    RobotNamespace::Robot robot1("Cirkitbot", 1.5, 50.0, 0.8, 4);

    robot1.displayDetails();

    robot1.moveForward();
    robot1.moveBackward();
    robot1.stop();

    return 0;
}
