#include <frankx/frankx.hpp>
using namespace frankx;

int main(int argc, char *argv[])
{
    // Connect to the robot with the FCI IP address
    Robot robot("172.16.0.2");

// Reduce velocity and acceleration of the robot
    robot.setDynamicRel(0.05);

// Move the end-effector 20cm in positive x-direction
    auto motion = LinearRelativeMotion(Affine(0.2, 0.0, 0.0));

// Finally move the robot
    robot.move(motion);
}