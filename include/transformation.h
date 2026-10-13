#ifndef TRANSFORMATION_TRANSFORMATION_H_
#define TRANSFORMATION_TRANSFORMATION_H_

/**
 * @file transformation.h
 * @brief 基于 Eigen 的旋转表示转换库：旋转矩阵 / 欧拉角 / 轴角 / 四元数 四种表示之间任意互转。
 *
 * 设计约定：
 *  - 旋转矩阵必须是合法旋转矩阵（RᵀR = I 且 det(R) = +1），否则抛出 std::invalid_argument；
 *  - 欧拉角序列为 3 个字符（x/y/z 各一次，大小写均可），语义为
 *    R = R_order[0](euler[0]) · R_order[1](euler[1]) · R_order[2](euler[2])，
 *    与 Eigen::Matrix3d::eulerAngles(i, j, k) 保持一致；
 *  - 轴角对象保证轴为单位向量：恒等旋转返回 (0, UnitX())，避免出现零轴；
 *  - 非法输入一律抛异常，库内不打印日志、不返回"零值"兜底。
 */

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include <Eigen/Dense>

/** 接近零的元素置零阈值 */
inline constexpr double kTransformationZeroThreshold = 1e-10;
/** 判定旋转角是否为 0 或 π 的阈值 */
inline constexpr double kTransformationAngleEpsilon = 1e-9;
/** 旋转矩阵正交性判定容差 */
inline constexpr double kTransformationUnitaryTolerance = 1e-6;
/** 四元数模长下限，低于该值无法归一化 */
inline constexpr double kTransformationQuaternionEpsilon = 1e-12;

namespace transformation_detail
{

/** 旋转序列映射表：序列 -> 各轴角度在欧拉角向量中的下标 */
inline const std::unordered_map<std::string, std::array<int, 3>>& orderMap()
{
    static const std::unordered_map<std::string, std::array<int, 3>> map = {
        {"zxy", {2, 0, 1}}, {"zyx", {2, 1, 0}}, {"yxz", {1, 0, 2}},
        {"yzx", {1, 2, 0}}, {"xyz", {0, 1, 2}}, {"xzy", {0, 2, 1}}};
    return map;
}

/** 将旋转序列统一为小写，便于大小写混用 */
inline std::string toLower(const std::string& text)
{
    std::string result = text;
    for (char& c : result)
    {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    return result;
}

/** 校验旋转矩阵并抛出异常 */
inline void requireUnitary(const Eigen::Matrix3d& R, const char* functionName);

/** 绕指定轴旋转 angle 的旋转矩阵，axis 取值 x/y/z */
inline Eigen::Matrix3d axisRotationMatrix(char axis, double angle)
{
    switch (axis)
    {
    case 'x':
        return Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX()).toRotationMatrix();
    case 'y':
        return Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY()).toRotationMatrix();
    case 'z':
        return Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    default:
        throw std::invalid_argument(std::string("无效的旋转轴：") + axis);
    }
}

/** 绕指定轴旋转 angle 的四元数，axis 取值 x/y/z */
inline Eigen::Quaterniond axisRotationQuaternion(char axis, double angle)
{
    switch (axis)
    {
    case 'x':
        return Eigen::Quaterniond(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX()));
    case 'y':
        return Eigen::Quaterniond(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY()));
    case 'z':
        return Eigen::Quaterniond(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()));
    default:
        throw std::invalid_argument(std::string("无效的旋转轴：") + axis);
    }
}

/**
 * @brief 规范化轴角对象：保证轴为单位向量，零轴或零角度退化为恒等旋转
 * @param axisAngle 待规范化的轴角
 * @return 轴为单位向量的轴角对象
 */
inline Eigen::AngleAxisd normalizeAxisAngle(const Eigen::AngleAxisd& axisAngle)
{
    const double axisNorm = axisAngle.axis().norm();

    // 零轴或零角度都表示恒等旋转，此时轴不唯一，任取 UnitX 以保证单位轴约定
    if (axisNorm < kTransformationAngleEpsilon || std::abs(axisAngle.angle()) < kTransformationAngleEpsilon)
    {
        return Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
    }

    return Eigen::AngleAxisd(axisAngle.angle(), axisAngle.axis() / axisNorm);
}

} // namespace transformation_detail

/**
 * @brief 将欧拉角旋转序列转化为索引顺序
 * @param order 欧拉角旋转序列，x/y/z 各一次，大小写均可，如 "xyz"、"ZYX"
 * @return 三元整型数组，表示三个角度在欧拉角向量中的下标
 * @throw std::invalid_argument 序列长度不为 3，或含有重复轴/非法字符
 */
inline std::array<int, 3> parseOrder(const std::string& order)
{
    if (order.size() != 3)
    {
        throw std::invalid_argument("旋转序列必须由 3 个字符组成（x/y/z 各一次），当前为：" + order);
    }

    const auto it = transformation_detail::orderMap().find(transformation_detail::toLower(order));
    if (it == transformation_detail::orderMap().end())
    {
        throw std::invalid_argument("无效的旋转序列：" + order + "，x/y/z 必须各出现一次");
    }

    return it->second;
}

/**
 * @brief 判断是否为合法旋转矩阵（正交且行列式为 +1）
 * @param R 待判断的矩阵
 * @param tolerance 容差
 * @return 合法返回 true；含缩放/剪切或镜像（det = −1）返回 false
 */
inline bool rotationMatrixIsUnitary(const Eigen::Matrix3d& R, double tolerance = kTransformationUnitaryTolerance)
{
    const bool orthogonal = (R.transpose() * R - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff() <= tolerance;
    const bool properRotation = std::abs(R.determinant() - 1.0) <= tolerance;
    return orthogonal && properRotation;
}

namespace transformation_detail
{

inline void requireUnitary(const Eigen::Matrix3d& R, const char* functionName)
{
    if (!rotationMatrixIsUnitary(R))
    {
        throw std::invalid_argument(std::string(functionName) +
                                    "：输入矩阵不是合法旋转矩阵（需满足 RᵀR = I 且 det(R) = +1）");
    }
}

} // namespace transformation_detail

/**
 * @brief 将接近零的元素置零，抑制浮点误差带来的 −0.0 与 1e-17 级噪声
 * @tparam Derived Eigen 矩阵或向量类型
 * @param mat 待处理的矩阵或向量
 * @param threshold 置零阈值
 * @return 与输入同类型的矩阵或向量
 */
template <class Derived>
auto roundNearZero(const Eigen::MatrixBase<Derived>& mat, double threshold = kTransformationZeroThreshold)
{
    using PlainMatrix = typename Derived::PlainObject;
    using Scalar = typename Derived::Scalar;

    // 使用 Eigen 的按元素 select，避免手写双层循环，便于编译器向量化
    return PlainMatrix((mat.array().abs() < threshold).select(Scalar(0), mat));
}

/**
 * @brief 旋转矩阵转欧拉角
 * @param R 旋转矩阵
 * @param order 欧拉角旋转序列
 * @return 欧拉角，euler[i] 对应 order[i] 轴
 * @throw std::invalid_argument 旋转矩阵非法或序列非法
 * @note 返回值沿用 Eigen::eulerAngles 的角度范围，且万向锁附近解不唯一
 */
inline Eigen::Vector3d rotationMatrixToEulerAngle(const Eigen::Matrix3d& R, const std::string& order)
{
    transformation_detail::requireUnitary(R, "rotationMatrixToEulerAngle");

    const std::array<int, 3> indices = parseOrder(order);
    return roundNearZero(Eigen::Vector3d(R.eulerAngles(indices[0], indices[1], indices[2])));
}

/**
 * @brief 旋转矩阵转轴角
 * @param R 旋转矩阵
 * @return 轴角，角度范围 [0, π]，轴为单位向量
 * @throw std::invalid_argument 旋转矩阵非法
 * @note 旋转角接近 π 时反对称分量退化，改由对称部分提取轴，避免 0/0
 */
inline Eigen::AngleAxisd rotationMatrixToAxisAngle(const Eigen::Matrix3d& R)
{
    transformation_detail::requireUnitary(R, "rotationMatrixToAxisAngle");

    // R − Rᵀ = 2 sin(θ)·[n]×，据此取反对称分量
    const Eigen::Vector3d antisymmetric(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
    const double sinTheta = 0.5 * antisymmetric.norm();
    const double cosTheta = std::clamp((R.trace() - 1.0) * 0.5, -1.0, 1.0);
    const double angle = std::atan2(sinTheta, cosTheta); // [0, π]，小角度时比 acos 更精确

    if (angle < kTransformationAngleEpsilon)
    {
        // 恒等旋转：轴不唯一，返回单位轴以满足 AngleAxisd 的约定
        return Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
    }

    if (M_PI - angle < kTransformationAngleEpsilon)
    {
        // θ ≈ π 时 sin(θ) ≈ 0，反对称分量退化，改用对称部分 R = 2nnᵀ − I 提取轴
        const double xx = (R(0, 0) + 1.0) * 0.5;
        const double yy = (R(1, 1) + 1.0) * 0.5;
        const double zz = (R(2, 2) + 1.0) * 0.5;

        Eigen::Vector3d axis;
        if (xx >= yy && xx >= zz)
        {
            axis(0) = std::sqrt(std::max(0.0, xx));
            axis(1) = R(0, 1) / (2.0 * axis(0));
            axis(2) = R(0, 2) / (2.0 * axis(0));
        }
        else if (yy >= zz)
        {
            axis(1) = std::sqrt(std::max(0.0, yy));
            axis(0) = R(0, 1) / (2.0 * axis(1));
            axis(2) = R(1, 2) / (2.0 * axis(1));
        }
        else
        {
            axis(2) = std::sqrt(std::max(0.0, zz));
            axis(0) = R(0, 2) / (2.0 * axis(2));
            axis(1) = R(1, 2) / (2.0 * axis(2));
        }

        return Eigen::AngleAxisd(angle, axis.normalized());
    }

    // 一般情况：sin(θ) > 0，反对称分量归一化后即为转轴
    return Eigen::AngleAxisd(angle, antisymmetric.normalized());
}

/**
 * @brief 旋转矩阵转四元数
 * @param R 旋转矩阵
 * @return 单位四元数
 * @throw std::invalid_argument 旋转矩阵非法
 */
inline Eigen::Quaterniond rotationMatrixToQuaternion(const Eigen::Matrix3d& R)
{
    transformation_detail::requireUnitary(R, "rotationMatrixToQuaternion");

    return Eigen::Quaterniond(roundNearZero(R)).normalized();
}

/**
 * @brief 欧拉角转旋转矩阵
 * @param eulerAngle 欧拉角，eulerAngle[i] 对应 order[i] 轴
 * @param order 欧拉角旋转序列
 * @return 旋转矩阵
 * @throw std::invalid_argument 序列非法
 */
inline Eigen::Matrix3d eulerAngleToRotationMatrix(const Eigen::Vector3d& eulerAngle, const std::string& order)
{
    const std::string normalizedOrder = transformation_detail::toLower(order);
    parseOrder(normalizedOrder);

    const Eigen::Matrix3d R = transformation_detail::axisRotationMatrix(normalizedOrder[0], eulerAngle[0]) *
                              transformation_detail::axisRotationMatrix(normalizedOrder[1], eulerAngle[1]) *
                              transformation_detail::axisRotationMatrix(normalizedOrder[2], eulerAngle[2]);
    return roundNearZero(R);
}

/**
 * @brief 欧拉角转轴角
 * @param eulerAngle 欧拉角
 * @param order 欧拉角旋转序列
 * @return 轴角
 * @throw std::invalid_argument 序列非法
 */
inline Eigen::AngleAxisd eulerAngleToAxisAngle(const Eigen::Vector3d& eulerAngle, const std::string& order)
{
    return rotationMatrixToAxisAngle(eulerAngleToRotationMatrix(eulerAngle, order));
}

/**
 * @brief 欧拉角转四元数
 * @param eulerAngle 欧拉角
 * @param order 欧拉角旋转序列
 * @return 单位四元数
 * @throw std::invalid_argument 序列非法
 */
inline Eigen::Quaterniond eulerAngleToQuaternion(const Eigen::Vector3d& eulerAngle, const std::string& order)
{
    const std::string normalizedOrder = transformation_detail::toLower(order);
    parseOrder(normalizedOrder);

    const Eigen::Quaterniond q = transformation_detail::axisRotationQuaternion(normalizedOrder[0], eulerAngle[0]) *
                                 transformation_detail::axisRotationQuaternion(normalizedOrder[1], eulerAngle[1]) *
                                 transformation_detail::axisRotationQuaternion(normalizedOrder[2], eulerAngle[2]);
    return q.normalized();
}

/**
 * @brief 轴角转旋转矩阵
 * @param axisAngle 轴角，轴会自动归一化；零轴视为恒等旋转
 * @return 旋转矩阵
 */
inline Eigen::Matrix3d axisAngleToRotationMatrix(const Eigen::AngleAxisd& axisAngle)
{
    return roundNearZero(transformation_detail::normalizeAxisAngle(axisAngle).toRotationMatrix());
}

/**
 * @brief 轴角转欧拉角
 * @param axisAngle 轴角
 * @param order 欧拉角旋转序列
 * @return 欧拉角
 * @throw std::invalid_argument 序列非法
 */
inline Eigen::Vector3d axisAngleToEulerAngle(const Eigen::AngleAxisd& axisAngle, const std::string& order)
{
    return rotationMatrixToEulerAngle(axisAngleToRotationMatrix(axisAngle), order);
}

/**
 * @brief 轴角转四元数
 * @param axisAngle 轴角
 * @return 单位四元数
 */
inline Eigen::Quaterniond axisAngleToQuaternion(const Eigen::AngleAxisd& axisAngle)
{
    return Eigen::Quaterniond(transformation_detail::normalizeAxisAngle(axisAngle)).normalized();
}

/**
 * @brief 四元数转旋转矩阵
 * @param quaternion 四元数，内部会归一化，因此非单位四元数也可直接传入
 * @return 旋转矩阵
 * @throw std::invalid_argument 四元数模长过小
 */
inline Eigen::Matrix3d quaternionToRotationMatrix(const Eigen::Quaterniond& quaternion)
{
    if (quaternion.norm() < kTransformationQuaternionEpsilon)
    {
        throw std::invalid_argument("quaternionToRotationMatrix：四元数模长过小，无法表示旋转");
    }

    return roundNearZero(quaternion.normalized().toRotationMatrix());
}

/**
 * @brief 四元数转欧拉角
 * @param quaternion 四元数
 * @param order 欧拉角旋转序列
 * @return 欧拉角
 * @throw std::invalid_argument 四元数模长过小或序列非法
 */
inline Eigen::Vector3d quaternionToEulerAngle(const Eigen::Quaterniond& quaternion, const std::string& order)
{
    return rotationMatrixToEulerAngle(quaternionToRotationMatrix(quaternion), order);
}

/**
 * @brief 四元数转轴角
 * @param quaternion 四元数
 * @return 轴角
 * @throw std::invalid_argument 四元数模长过小
 */
inline Eigen::AngleAxisd quaternionToAxisAngle(const Eigen::Quaterniond& quaternion)
{
    return rotationMatrixToAxisAngle(quaternionToRotationMatrix(quaternion));
}

#endif // TRANSFORMATION_TRANSFORMATION_H_
