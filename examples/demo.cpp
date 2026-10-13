/**
 * @file demo.cpp
 * @brief 演示四种旋转表示的相互转换，并对 12 条转换路径做一次往返自检。
 *
 * 构建运行：
 *   cmake -S . -B build && cmake --build build && ./build/examples/demo
 */

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "transformation.h"

namespace
{

constexpr double kTolerance = 1e-9;

/** 两个旋转之间的差异（矩阵差的 Frobenius 范数），避免直接比较欧拉角带来的多解干扰 */
double rotationError(const Eigen::Matrix3d& A, const Eigen::Matrix3d& B) { return (A - B).norm(); }

double quaternionError(const Eigen::Quaterniond& a, const Eigen::Quaterniond& b)
{
    return std::min((a.coeffs() - b.coeffs()).norm(), (a.coeffs() + b.coeffs()).norm());
}

} // namespace

int main()
{
    std::cout << std::fixed << std::setprecision(6);

    const std::string order = "xyz";
    const Eigen::Vector3d euler(M_PI / 6.0, -M_PI / 4.0, M_PI / 3.0);

    const Eigen::Matrix3d R = eulerAngleToRotationMatrix(euler, order);
    const Eigen::AngleAxisd axisAngle = eulerAngleToAxisAngle(euler, order);
    const Eigen::Quaterniond quaternion = eulerAngleToQuaternion(euler, order);

    std::cout << "=== 四种表示（同一个旋转，欧拉角序列 " << order << "）===\n";
    std::cout << "欧拉角 : " << euler.transpose() << "\n\n";
    std::cout << "旋转矩阵:\n" << R << "\n\n";
    std::cout << "轴角   : angle = " << axisAngle.angle() << "  axis = " << axisAngle.axis().transpose() << "\n";
    std::cout << "四元数 : (x y z w) = " << quaternion.coeffs().transpose() << "\n\n";

    std::cout << "=== 12 条转换路径往返自检 ===\n";
    const std::vector<std::pair<std::string, double>> cases = {
        {"R -> Euler -> R", rotationError(eulerAngleToRotationMatrix(rotationMatrixToEulerAngle(R, order), order), R)},
        {"R -> AxisAngle -> R", rotationError(axisAngleToRotationMatrix(rotationMatrixToAxisAngle(R)), R)},
        {"R -> Quaternion -> R", rotationError(quaternionToRotationMatrix(rotationMatrixToQuaternion(R)), R)},
        {"Euler -> R -> Euler", rotationError(eulerAngleToRotationMatrix(rotationMatrixToEulerAngle(R, order), order), R)},
        {"Euler -> AxisAngle -> Euler",
         rotationError(eulerAngleToRotationMatrix(axisAngleToEulerAngle(axisAngle, order), order), R)},
        {"Euler -> Quaternion -> Euler",
         rotationError(eulerAngleToRotationMatrix(quaternionToEulerAngle(quaternion, order), order), R)},
        {"AxisAngle -> R -> AxisAngle",
         rotationError(axisAngleToRotationMatrix(rotationMatrixToAxisAngle(axisAngleToRotationMatrix(axisAngle))), R)},
        {"AxisAngle -> Euler -> AxisAngle",
         rotationError(axisAngleToRotationMatrix(eulerAngleToAxisAngle(axisAngleToEulerAngle(axisAngle, order), order)), R)},
        {"AxisAngle -> Quaternion -> AxisAngle",
         rotationError(axisAngleToRotationMatrix(quaternionToAxisAngle(axisAngleToQuaternion(axisAngle))), R)},
        {"Quaternion -> R -> Quaternion",
         quaternionError(rotationMatrixToQuaternion(quaternionToRotationMatrix(quaternion)), quaternion)},
        {"Quaternion -> Euler -> Quaternion",
         quaternionError(eulerAngleToQuaternion(quaternionToEulerAngle(quaternion, order), order), quaternion)},
        {"Quaternion -> AxisAngle -> Quaternion",
         quaternionError(axisAngleToQuaternion(quaternionToAxisAngle(quaternion)), quaternion)},
    };

    int passed = 0;
    for (const auto& [name, error] : cases)
    {
        const bool ok = error < kTolerance;
        passed += ok ? 1 : 0;
        std::cout << (ok ? "[PASS] " : "[FAIL] ") << name << "  误差 = " << error << "\n";
    }

    std::cout << "\n结果: " << passed << "/" << cases.size() << " 通过\n";
    return passed == static_cast<int>(cases.size()) ? 0 : 1;
}
