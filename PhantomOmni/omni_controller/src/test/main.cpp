#include "node/Omni.h"

#include <vector>

int main()
{
    RobotBase * robot = new Omni();

    Eigen::VectorXd theta(7);
    theta << 1, 2, 3, 4, 5, 6, 7;
    std::cout << robot->fwdKin(theta) << std::endl;

    delete robot;

    return 0;
}

