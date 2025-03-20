#include <iostream>
#include <Eigen/Dense>
#include "transformation.h"

int main()
{
    // 测试旋转矩阵
    Eigen::Matrix3d rotationMatrix;
    rotationMatrix << 1, 0, 0,
        0, 0.707107, -0.707107,
        0, 0.707107, 0.707107;
    std::cout << "旋转矩阵(RotationMatrix): " << rotationMatrix << std::endl;

    // 测试欧拉角
    Eigen::Vector3d eulerAngle(M_PI / 4, 0, 0); // 假设欧拉角为(45°, 0, 0)
    std::string order = "xyz";                  // 旋转顺序
    std::cout << "欧拉角(EulerAngle): " << eulerAngle.transpose() << std::endl;

    // 测试轴角
    double angle = M_PI / 4;       // 45度旋转
    Eigen::Vector3d axis(1, 0, 0); // 绕x轴旋转
    Eigen::AngleAxisd axisAngle(angle, axis);
    std::cout << "轴角(AngleAxis): " << axisAngle.angle() << " " << axisAngle.axis().transpose() << std::endl;

    // 测试四元数
    Eigen::Quaterniond quaternion(0.382683, 0, 0, 0.92388);
    std::cout << "四元数(Quaternion): " << quaternion << std::endl;

    // 分隔符
    std::cout << "----------------------------------------------------" << std::endl;

    // 1. 旋转矩阵 -> 欧拉角
    Eigen::Vector3d eulerAngleFromMatrix = rotationMatrixToEulerAngle(rotationMatrix, order);
    std::cout << "1. 旋转矩阵转欧拉角: " << eulerAngleFromMatrix.transpose() << std::endl;

    // 分隔符
    std::cout << "----------------------------------------------------" << std::endl;

    // 2. 旋转矩阵 -> 轴角
    Eigen::AngleAxisd axisAngleFromMatrix = rotationMatrixToAxisAngle(rotationMatrix);
    std::cout << "2. 旋转矩阵转轴角: \n"
              << axisAngleFromMatrix.angle() << " "
              << axisAngleFromMatrix.axis().transpose() << std::endl;

    // 分隔符
    std::cout << "----------------------------------------------------" << std::endl;

    // 3. 旋转矩阵 -> 四元数
    Eigen::Quaterniond quaternionFromMatrix = rotationMatrixToQuaternion(rotationMatrix);
    std::cout << "3. 旋转矩阵转四元数: " << quaternionFromMatrix.coeffs().transpose() << std::endl;

    // 分隔符
    std::cout << "----------------------------------------------------" << std::endl;

    // 4. 欧拉角 -> 旋转矩阵
    Eigen::Matrix3d rotationFromMatrix = eulerAngleToRotationMatrix(eulerAngle, order);
    std::cout << "4. 欧拉角转旋转矩阵: \n"
              << rotationFromMatrix << std::endl;

    // 分隔符
    std::cout << "----------------------------------------------------" << std::endl;

    // 5. 欧拉角 -> 轴角
    Eigen::AngleAxisd axisAngleFromEuler = eulerAngleToAxisAngle(eulerAngle, order);
    std::cout << "5. 欧拉角转轴角: \n"
              << axisAngleFromEuler.angle() << " "
              << axisAngleFromEuler.axis().transpose() << std::endl;

    // 分隔符
    std::cout << "----------------------------------------------------" << std::endl;

    // 6. 欧拉角 -> 四元数
    Eigen::Quaterniond quaternionFromEuler = eulerAngleToQuaternion(eulerAngle, order);
    std::cout << "6. 欧拉角转四元数: " << quaternionFromEuler.coeffs().transpose() << std::endl;

    // 分隔符
    std::cout << "----------------------------------------------------" << std::endl;

    // 7. 轴角 -> 旋转矩阵
    Eigen::Matrix3d rotationMatrixFromAxisAngle = axisAngleToRotationMatrix(axisAngle);
    std::cout << "7. 轴角转旋转矩阵: \n"
              << rotationMatrixFromAxisAngle << std::endl;

    // 分隔符
    std::cout << "----------------------------------------------------" << std::endl;

    // 8. 轴角 -> 欧拉角
    Eigen::Vector3d eulerFromAxisAngle = axisAngleToEulerAngle(axisAngle, order);
    std::cout << "8. 轴角转欧拉角: " << eulerFromAxisAngle.transpose() << std::endl;

    // 分隔符
    std::cout << "----------------------------------------------------" << std::endl;

    // 9. 轴角 -> 四元数
    Eigen::Quaterniond quaternionFromAxisAngle = axisAngleToQuaternion(axisAngle);
    std::cout << "9. 轴角转四元数: " << quaternionFromAxisAngle.coeffs().transpose() << std::endl;

    // 分隔符
    std::cout << "----------------------------------------------------" << std::endl;

    // 10. 四元数 -> 旋转矩阵
    Eigen::Matrix3d rotationMatrixFromQuaternion = quaternionToRotationMatrix(quaternion);
    std::cout << "10. 四元数转旋转矩阵: \n"
              << rotationMatrixFromQuaternion << std::endl;

    // 分隔符
    std::cout << "----------------------------------------------------" << std::endl;

    // 11. 四元数 -> 欧拉角
    Eigen::Vector3d eulerFromQuaternion = quaternionToEulerAngle(quaternion, order);
    std::cout << "11. 四元数转欧拉角: " << eulerFromQuaternion.transpose() << std::endl;

    // 分隔符
    std::cout << "----------------------------------------------------" << std::endl;

    // 12. 四元数 -> 轴角
    Eigen::AngleAxisd axisAngleFromQuaternion = quaternionToAxisAngle(quaternion);
    std::cout << "12. 四元数转轴角: \n"
              << axisAngleFromQuaternion.angle() << " "
              << axisAngleFromQuaternion.axis().transpose() << std::endl;

    return 0;
}