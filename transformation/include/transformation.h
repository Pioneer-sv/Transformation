#ifndef __TRANSFORMATION_H__
#define __TRANSFORMATION_H__

/*
 * 待实现功能：轴角置零化、注释规范化、置零处理算法时间复杂度优化
 */

#include <cmath>
#include <string>
#include <iostream>
#include <Eigen/Dense>
#include <functional>
#include <unordered_map>

// 启用C++标准数学库中的常量
#define _USE_MATH_DEFINES

// 旋转序列映射表
static const std::unordered_map<std::string, std::array<int, 3>> orderMap = 
{
    {"zxy", {2, 0, 1}}, {"zyx", {2, 1, 0}}, 
    {"yxz", {1, 0, 2}}, {"yzx", {1, 2, 0}},
    {"xyz", {0, 1, 2}}, {"xzy", {0, 2, 1}},
    {"ZXY", {2, 0, 1}}, {"ZYX", {2, 1, 0}}, 
    {"YXZ", {1, 0, 2}}, {"YZX", {1, 2, 0}}, 
    {"XYZ", {0, 1, 2}}, {"XZY", {0, 2, 1}}
};

/** 
 * @brief 将欧拉角旋转序列转化为索引顺序
 * @param order 欧拉角旋转序列
 * @return 三元整型数组
 */
inline std::array<int, 3> parseOrder(const std::string& order)
{
    auto it = orderMap.find(order);

    // 查找映射表中是否有对应元素
    if (it != orderMap.end())
    {
        return it->second;
    }

    // 抛出异常以处理无效输入
    throw std::invalid_argument("输入的旋转序列带有重复轴或输入错误，无法转换为索引顺序！");
}

/** 
 * @brief 判断旋转矩阵是否正交
 * @param order 要判断的旋转矩阵
 */
void rotationMatrixIsUnitary(const Eigen::Matrix3d& R)
{
    if (!R.isUnitary())
    {
        throw std::invalid_argument("输入的旋转矩阵不是正交阵，无法转换为轴角！");
    }
}

/** 
 * @brief 将接近零的数置零
 * @param mat 要处理的矩阵或向量
 * @param threshold 临界点 1e-10
 * Eigen::MatrixBase<Derived>表示Eigen任意矩阵类型，Eigen::MatrixBase是Eigen库中的基类模板，
 * 所有矩阵和向量都继承自它，Derived是模板参数，当传入参数时，处理的对象类型自然确定下来
 */
template <class Derived>
auto roundNearZero(const Eigen::MatrixBase<Derived>& mat, double threshold = 1e-10) 
{
    using PlainMatrix = typename Derived::PlainObject;
    PlainMatrix result = mat.eval(); // 确保返回具体派生类对象

    for (int i = 0; i < result.rows(); i++) 
    {
        for (int j = 0; j < result.cols(); j++) 
        {
            if (std::abs(result(i, j)) < threshold) 
            {
                result(i, j) = 0.0;
            }
        }
    }
    return result;
}

/**
 * @brief 处理异常并返回默认值
 * @param e 异常对象，捕获到的标准异常
 */
inline void handleException(const std::exception& e) 
{
    std::cerr << "异常信息：" << e.what() << std::endl; // 输出异常信息
}

/** 
 * @brief 欧拉角转绕X轴的旋转矩阵
 * @param eulerAngle 欧拉角
 * @param n 角度在旋转序列中的位置
 * @return 旋转矩阵
 */
inline Eigen::Matrix3d XR(const Eigen::Vector3d& eulerAngle, int n) 
{ 
    return Eigen::AngleAxisd(eulerAngle[n], Eigen::Vector3d::UnitX()).toRotationMatrix(); 
}

/** 
 * @brief 欧拉角转绕Y轴的旋转矩阵
 * @param eulerAngle 欧拉角
 * @param n 角度在旋转序列中的位置
 * @return 旋转矩阵
 */
inline Eigen::Matrix3d YR(const Eigen::Vector3d& eulerAngle, int n) 
{ 
    return Eigen::AngleAxisd(eulerAngle[n], Eigen::Vector3d::UnitY()).toRotationMatrix(); 
}

/** 
 * @brief 欧拉角转绕Z轴的旋转矩阵
 * @param eulerAngle 欧拉角
 * @param n 角度在旋转序列中的位置
 * @return 旋转矩阵
 */
inline Eigen::Matrix3d ZR(const Eigen::Vector3d& eulerAngle, int n) 
{ 
    return Eigen::AngleAxisd(eulerAngle[n], Eigen::Vector3d::UnitZ()).toRotationMatrix(); 
}

/** 
 * @brief 欧拉角转轴角
 * @param eulerAngle 欧拉角
 * @param n 角度在旋转序列中的位置
 * @return 轴角
 */
inline Eigen::AngleAxisd X(const Eigen::Vector3d& eulerAngle, int n) 
{ 
    return Eigen::AngleAxisd(eulerAngle[n], Eigen::Vector3d::UnitX()); 
}

/** 
 * @brief 欧拉角转轴角
 * @param eulerAngle 欧拉角
 * @param n 角度在旋转序列中的位置
 * @return 轴角
 */
inline Eigen::AngleAxisd Y(const Eigen::Vector3d& eulerAngle, int n) 
{ 
    return Eigen::AngleAxisd(eulerAngle[n], Eigen::Vector3d::UnitY()); 
}

/** 
 * @brief 欧拉角转轴角
 * @param eulerAngle 欧拉角
 * @param n 角度在旋转序列中的位置
 * @return 轴角
 */
inline Eigen::AngleAxisd Z(const Eigen::Vector3d& eulerAngle, int n) 
{ 
    return Eigen::AngleAxisd(eulerAngle[n], Eigen::Vector3d::UnitZ()); 
}

/** 
 * @brief 旋转矩阵转欧拉角
 * @param R 旋转矩阵
 * @param order 欧拉角变换顺序
 * @return 欧拉角
 */
Eigen::Vector3d rotationMatrixToEulerAngle(const Eigen::Matrix3d& R, const std::string& order)
{
    try 
    {
        // 解析旋转序列
        const std::array<int, 3> indices = parseOrder(order);

        // 将接近零的数置零
        return roundNearZero(R.eulerAngles(indices[0], indices[1], indices[2]));

        // return R.eulerAngles(indices[0], indices[1], indices[2]);
    }
    catch (const std::exception& e)
    {
        handleException(e); // 调用异常处理函数
        return Eigen::Vector3d::Zero();
    }
}

/** 
 * @brief 旋转矩阵转轴角
 * @param R 旋转矩阵
 * @return 轴角
 */
Eigen::AngleAxisd rotationMatrixToAxisAngle(const Eigen::Matrix3d& R)
{
    try 
    {
       double numerator = sqrt(pow(R(2, 1) - R(1, 2), 2) + 
                            pow(R(0, 2) - R(2, 0), 2) + 
                            pow(R(1, 0) - R(0, 1), 2));

        double angle = atan2(numerator, R.trace() - 1); // 角

        // 处理角度接近零的情况
        if (std::abs(angle) < 1e-6) 
        {
            return Eigen::AngleAxisd(angle, Eigen::Vector3d::Zero());  // 返回零向量表示无显著旋转
        }

        Eigen::Vector3d axis((R(2, 1) - R(1, 2)) / (2 * sin(angle)), 
                            (R(0, 2) - R(2, 0)) / (2 * sin(angle)), 
                            (R(1, 0) - R(0, 1)) / (2 * sin(angle))); // 轴

        return Eigen::AngleAxisd(angle, axis.normalized()); // 返回计算所得轴角 
    }
    catch (const std::exception& e)
    {
        handleException(e); // 调用异常处理函数
        return Eigen::AngleAxisd(0, Eigen::Vector3d(0, 0, 1));
    }
}

/** 
 * @brief 旋转矩阵转四元数
 * @param R 旋转矩阵
 * @return 四元数
 */
Eigen::Quaterniond rotationMatrixToQuaternion(const Eigen::Matrix3d& R)
{
    return Eigen::Quaterniond(R);
}

/** 
 * @brief 欧拉角转旋转矩阵
 * @param eulerAngle 欧拉角
 * @param order 欧拉角变换顺序
 * @return 旋转矩阵
 */
Eigen::Matrix3d eulerAngleToRotationMatrix(const Eigen::Vector3d& eulerAngle, const std::string& order)
{
    // 创建旋转矩阵映射表，用于将旋转序列映射到对应的匿名函数对象上
    static const std::unordered_map<char, std::function<Eigen::Matrix3d(int)>> rotationMap =
        {
            {'x', [&](int n) { return XR(eulerAngle, n); }},
            {'y', [&](int n) { return YR(eulerAngle, n); }},
            {'z', [&](int n) { return ZR(eulerAngle, n); }}
        };

    if (order.size() != 3)
    {
        throw std::invalid_argument("无效的旋转序列！");
    }

    // 通过键值获取对应的lambda表达式并传入参数(使用at而非[]会在order输入错误且字符个数仍是3时抛出异常)
    return roundNearZero(rotationMap.at(order[0])(0) * rotationMap.at(order[1])(1) * rotationMap.at(order[2])(2));
}

/** 
 * @brief 欧拉角转轴角
 * @param eulerAngle 欧拉角
 * @param order 欧拉角变换顺序
 * @return 轴角
 */
Eigen::AngleAxisd eulerAngleToAxisAngle(const Eigen::Vector3d& eulerAngle, const std::string& order)
{
    return rotationMatrixToAxisAngle(eulerAngleToRotationMatrix(eulerAngle, order)); // 欧拉角先转为旋转矩阵，再转为轴角
}

/** 
 * @brief 欧拉角转四元数
 * @param eulerAngle 欧拉角
 * @param order 欧拉角变换顺序
 * @return 四元数
 */
Eigen::Quaterniond eulerAngleToQuaternion(const Eigen::Vector3d& eulerAngle, const std::string& order)
{
    static const std::unordered_map<char, std::function<Eigen::AngleAxisd(int)>> rotationMap = 
        {
            {'x', [&](int n) { return X(eulerAngle, n); }},
            {'y', [&](int n) { return Y(eulerAngle, n); }},
            {'z', [&](int n) { return Z(eulerAngle, n); }}
        };

    if (order.size() != 3)
    {
        throw std::invalid_argument("无效的旋转序列！");
    } 

    return rotationMap.at(order[0])(0) * rotationMap.at(order[1])(1) * rotationMap.at(order[2])(2);
}

/** 
 * @brief 轴角转旋转矩阵
 * @param axisAngle 轴角
 * @return 旋转矩阵
 */
Eigen::Matrix3d axisAngleToRotationMatrix(const Eigen::AngleAxisd& axisAngle)
{
    return roundNearZero(axisAngle.toRotationMatrix());
}

/** 
 * @brief 轴角转欧拉角
 * @param axisAngle 轴角
 * @param order 欧拉角变换顺序
 * @return 欧拉角
 */
Eigen::Vector3d axisAngleToEulerAngle(const Eigen::AngleAxisd& axisAngle, const std::string& order)
{
    try 
    {
        // 解析旋转序列
        const std::array<int, 3> indices = parseOrder(order);

        return roundNearZero(axisAngle.toRotationMatrix().eulerAngles(indices[0], indices[1], indices[2]));
    }
    catch (const std::exception& e)
    {
        handleException(e); // 调用异常处理函数
        return Eigen::Vector3d::Zero();
    } 
}

/** 
 * @brief 轴角转四元数
 * @param axisAngle 轴角
 * @return 四元数
 */
Eigen::Quaterniond axisAngleToQuaternion(const Eigen::AngleAxisd& axisAngle)
{
    return Eigen::Quaterniond(axisAngle);
}

/** 
 * @brief 四元数转旋转矩阵
 * @param quaternion 四元数
 * @return 旋转矩阵
 */
Eigen::Matrix3d quaternionToRotationMatrix(const Eigen::Quaterniond& quaternion)
{
    return roundNearZero(Eigen::Matrix3d(quaternion.toRotationMatrix()));
}

/** 
 * @brief 四元数转欧拉角
 * @param quaternion 四元数
 * @param order 欧拉角变换顺序
 * @return 欧拉角
 */
Eigen::Vector3d quaternionToEulerAngle(const Eigen::Quaterniond& quaternion, const std::string& order)
{
    try 
    {
        // 解析旋转序列
        const std::array<int, 3> indices = parseOrder(order);

        return roundNearZero(Eigen::Vector3d(quaternion.toRotationMatrix().eulerAngles(indices[0], indices[1], indices[2])));
    }
    catch (const std::exception& e)
    {
        handleException(e); // 调用异常处理函数
        return Eigen::Vector3d::Zero();
    } 
}

/** 
 * @brief 四元数转轴角
 * @param quaternion 四元数
 * @return 轴角
 */
Eigen::AngleAxisd quaternionToAxisAngle(const Eigen::Quaterniond& quaternion)
{
    return rotationMatrixToAxisAngle(quaternionToRotationMatrix(quaternion)); // 四元数先转为旋转矩阵，再转为轴角
}

#endif                                                                                                                                                      
