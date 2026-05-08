#include <iostream>
#include <iomanip>
#include <cmath>
#include <Eigen/Dense>
#include "transformation.h"

// 打印辅助函数
void print(const std::string& name, const Eigen::Matrix3d& m) {
    std::cout << name << ":\n" << m << "\n" << std::endl;
}
void print(const std::string& name, const Eigen::Vector3d& v) {
    std::cout << name << ": " << v.transpose() << std::endl;
}
void print(const std::string& name, const Eigen::AngleAxisd& a) {
    std::cout << name << ": angle=" << a.angle() << " axis=" << a.axis().transpose() << std::endl;
}
void print(const std::string& name, const Eigen::Quaterniond& q) {
    std::cout << name << ": " << q.coeffs().transpose() << std::endl;
}

// 往返测试：A -> B -> A，比较是否一致
template<typename T>
bool check(const std::string& label, const T& original, const T& roundtripped, double tol = 1e-6) = delete;

bool check(const std::string& label, const Eigen::Vector3d& a, const Eigen::Vector3d& b, double tol = 1e-6) {
    double diff = (a - b).norm();
    bool ok = diff < tol;
    std::cout << (ok ? "[PASS] " : "[FAIL] ") << label << " (diff=" << diff << ")" << std::endl;
    return ok;
}
bool check(const std::string& label, const Eigen::Matrix3d& a, const Eigen::Matrix3d& b, double tol = 1e-6) {
    double diff = (a - b).norm();
    bool ok = diff < tol;
    std::cout << (ok ? "[PASS] " : "[FAIL] ") << label << " (diff=" << diff << ")" << std::endl;
    return ok;
}
bool check(const std::string& label, const Eigen::AngleAxisd& a, const Eigen::AngleAxisd& b, double tol = 1e-6) {
    double angleDiff = std::abs(a.angle() - b.angle());
    double axisDiff = (a.axis() - b.axis()).norm();
    bool ok = (angleDiff < tol) && (axisDiff < tol);
    std::cout << (ok ? "[PASS] " : "[FAIL] ") << label
              << " (angleDiff=" << angleDiff << " axisDiff=" << axisDiff << ")" << std::endl;
    return ok;
}
bool check(const std::string& label, const Eigen::Quaterniond& a, const Eigen::Quaterniond& b, double tol = 1e-6) {
    // 四元数符号可以同时取反，取最小差异
    double diff = std::min((a.coeffs() - b.coeffs()).norm(), (a.coeffs() + b.coeffs()).norm());
    bool ok = diff < tol;
    std::cout << (ok ? "[PASS] " : "[FAIL] ") << label << " (diff=" << diff << ")" << std::endl;
    return ok;
}

int main() {
    std::cout << std::fixed << std::setprecision(4);
    const std::string order = "xyz";

    // 初始值
    Eigen::Matrix3d R;
    R << 1, 0, 0,
         0, 0.707107, -0.707107,
         0, 0.707107, 0.707107;
    Eigen::Vector3d euler(M_PI / 4, 0, 0);
    Eigen::AngleAxisd axisAngle(M_PI / 4, Eigen::Vector3d::UnitX());
    Eigen::Quaterniond quat(axisAngle);

    int pass = 0, total = 0;

    // 旋转矩阵往返测试
    std::cout << "=== 旋转矩阵往返测试 ===" << std::endl;
    pass += check("R->Euler->R",    R, eulerAngleToRotationMatrix(rotationMatrixToEulerAngle(R, order), order)); total++;
    pass += check("R->AxisAng->R",  R, axisAngleToRotationMatrix(rotationMatrixToAxisAngle(R)));                    total++;
    pass += check("R->Quat->R",     R, quaternionToRotationMatrix(rotationMatrixToQuaternion(R)));                   total++;

    // 欧拉角往返测试
    std::cout << "\n=== 欧拉角往返测试 ===" << std::endl;
    pass += check("Euler->R->Euler",      euler, rotationMatrixToEulerAngle(eulerAngleToRotationMatrix(euler, order), order)); total++;
    pass += check("Euler->AxisAng->Euler", euler, axisAngleToEulerAngle(eulerAngleToAxisAngle(euler, order), order));      total++;
    pass += check("Euler->Quat->Euler",    euler, quaternionToEulerAngle(eulerAngleToQuaternion(euler, order), order));     total++;

    // 轴角往返测试
    std::cout << "\n=== 轴角往返测试 ===" << std::endl;
    pass += check("AxisAng->R->AxisAng",      axisAngle, rotationMatrixToAxisAngle(axisAngleToRotationMatrix(axisAngle)));      total++;
    pass += check("AxisAng->Euler->AxisAng",  axisAngle, eulerAngleToAxisAngle(axisAngleToEulerAngle(axisAngle, order), order)); total++;
    pass += check("AxisAng->Quat->AxisAng",   axisAngle, quaternionToAxisAngle(axisAngleToQuaternion(axisAngle)));               total++;

    // 四元数往返测试
    std::cout << "\n=== 四元数往返测试 ===" << std::endl;
    pass += check("Quat->R->Quat",      quat, rotationMatrixToQuaternion(quaternionToRotationMatrix(quat)));                   total++;
    pass += check("Quat->Euler->Quat",  quat, eulerAngleToQuaternion(quaternionToEulerAngle(quat, order), order));             total++;
    pass += check("Quat->AxisAng->Quat", quat, axisAngleToQuaternion(quaternionToAxisAngle(quat)));                             total++;

    std::cout << "\n=== 结果: " << pass << "/" << total << " 通过 ===" << std::endl;
    return pass == total ? 0 : 1;
}
