#include <iostream>
#include <Eigen/Dense>
#include "transformation.h"

int main()
{
    Eigen::Matrix3d R;
    R << 1, 0, 0,
         0, 1, 0,
         0, 0, 1;
    std::cout << rotationMatrixToQuaternion(R);
    return 0;
}