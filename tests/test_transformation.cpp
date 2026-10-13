/**
 * @file test_transformation.cpp
 * @brief transformation 库的测试套件：已知值校验 + 全组合往返 + 边界情况 + 异常行为 + 随机抽样。
 *
 * 直接运行（或经 ctest）即可，返回码非 0 表示有用例失败。
 * 说明：万向锁附近欧拉角解不唯一，因此涉及欧拉角的往返一律比较"旋转矩阵"而不是角度本身。
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <random>
#include <string>
#include <type_traits>

#include <Eigen/Dense>

#include "transformation.h"

namespace
{

constexpr double kTol = 1e-9;
constexpr int kRandomSamples = 300;
const std::array<std::string, 6> kOrders = {"xyz", "xzy", "yxz", "yzx", "zxy", "zyx"};

int g_checks = 0;
int g_failures = 0;

// ---------------------------------------------------------------- 测试框架

void checkImpl(bool ok, const std::string& what, const char* file, int line)
{
    ++g_checks;
    if (!ok)
    {
        ++g_failures;
        std::cout << "  [FAIL] " << file << ":" << line << "  " << what << "\n";
    }
}

template <typename Exception, typename Callable>
void checkThrowsImpl(Callable&& callable, const std::string& what, const char* file, int line)
{
    ++g_checks;
    try
    {
        callable();
    }
    catch (const Exception&)
    {
        return;
    }
    catch (const std::exception& e)
    {
        ++g_failures;
        std::cout << "  [FAIL] " << file << ":" << line << "  " << what << "：异常类型不符（实际 " << typeid(e).name()
                  << "）\n";
        return;
    }

    ++g_failures;
    std::cout << "  [FAIL] " << file << ":" << line << "  " << what << "：未抛出异常\n";
}

#define CHECK(cond) checkImpl((cond), #cond, __FILE__, __LINE__)
#define CHECK_THROWS(expr, Exception) \
    checkThrowsImpl<Exception>([&] { static_cast<void>(expr); }, #expr, __FILE__, __LINE__)

void runGroup(const std::string& name, void (*test)())
{
    const int before = g_failures;
    test();
    std::cout << (g_failures == before ? "[ OK ] " : "[FAIL] ") << name << "\n";
}

// ---------------------------------------------------------------- 工具函数

Eigen::Matrix3d rotX(double angle) { return Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX()).toRotationMatrix(); }
Eigen::Matrix3d rotY(double angle) { return Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY()).toRotationMatrix(); }
Eigen::Matrix3d rotZ(double angle) { return Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix(); }

/**
 * 两个旋转之间的差异，取矩阵差的 Frobenius 范数。
 * 不用 acos((tr(AᵀB) − 1) / 2)：该式在误差趋近 0 时导数发散，会把 1e-13 的误差放大成 1e-6。
 */
double rotationError(const Eigen::Matrix3d& A, const Eigen::Matrix3d& B) { return (A - B).norm(); }

/** 四元数差异，考虑 q 与 −q 表示同一旋转 */
double quaternionError(const Eigen::Quaterniond& a, const Eigen::Quaterniond& b)
{
    return std::min((a.coeffs() - b.coeffs()).norm(), (a.coeffs() + b.coeffs()).norm());
}

/** 均匀随机单位四元数（Shoemake 方法），固定种子保证可复现 */
Eigen::Quaterniond randomUnitQuaternion(std::mt19937& gen)
{
    std::uniform_real_distribution<double> uniform(0.0, 1.0);
    const double u1 = uniform(gen);
    const double u2 = uniform(gen);
    const double u3 = uniform(gen);
    const double r1 = std::sqrt(1.0 - u1);
    const double r2 = std::sqrt(u1);

    return Eigen::Quaterniond(r2 * std::sin(2.0 * M_PI * u2), r2 * std::cos(2.0 * M_PI * u2),
                              r1 * std::sin(2.0 * M_PI * u3), r1 * std::cos(2.0 * M_PI * u3));
}

// ---------------------------------------------------------------- 用例

void testParseOrder()
{
    CHECK(parseOrder("xyz") == (std::array<int, 3>{0, 1, 2}));
    CHECK(parseOrder("zyx") == (std::array<int, 3>{2, 1, 0}));
    CHECK(parseOrder("zxy") == (std::array<int, 3>{2, 0, 1}));
    CHECK(parseOrder("XYZ") == parseOrder("xyz"));
    CHECK(parseOrder("ZyX") == parseOrder("zyx"));

    CHECK_THROWS(parseOrder(""), std::invalid_argument);
    CHECK_THROWS(parseOrder("xy"), std::invalid_argument);
    CHECK_THROWS(parseOrder("xyzz"), std::invalid_argument);
    CHECK_THROWS(parseOrder("xxy"), std::invalid_argument);
    CHECK_THROWS(parseOrder("abc"), std::invalid_argument);
}

void testKnownValues()
{
    // 绕 z 轴 90°：期望值手写
    Eigen::Matrix3d expectedZ90;
    expectedZ90 << 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    const Eigen::Vector3d eulerZ90(0.0, 0.0, M_PI / 2.0);

    CHECK((eulerAngleToRotationMatrix(eulerZ90, "xyz") - expectedZ90).norm() < kTol);
    CHECK((axisAngleToRotationMatrix(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ())) - expectedZ90).norm() < kTol);
    CHECK((quaternionToRotationMatrix(Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()))) -
           expectedZ90)
              .norm() < kTol);

    // 欧拉角序列语义：R = R_order[0](e0) · R_order[1](e1) · R_order[2](e2)
    const Eigen::Vector3d euler(0.3, -0.4, 0.5);
    CHECK((eulerAngleToRotationMatrix(euler, "xyz") - rotX(0.3) * rotY(-0.4) * rotZ(0.5)).norm() < kTol);
    CHECK((eulerAngleToRotationMatrix(euler, "zyx") - rotZ(0.3) * rotY(-0.4) * rotX(0.5)).norm() < kTol);

    // 反向：旋转矩阵 → 欧拉角
    const Eigen::Vector3d eulerBack = rotationMatrixToEulerAngle(expectedZ90, "xyz");
    CHECK(std::abs(eulerBack(2) - M_PI / 2.0) < kTol);
    CHECK(eulerBack.head<2>().norm() < kTol);

    // 旋转矩阵 → 轴角
    const Eigen::AngleAxisd axisAngle = rotationMatrixToAxisAngle(expectedZ90);
    CHECK(std::abs(axisAngle.angle() - M_PI / 2.0) < kTol);
    CHECK((axisAngle.axis() - Eigen::Vector3d::UnitZ()).norm() < kTol);

    // 旋转矩阵 → 四元数
    const Eigen::Quaterniond quat = rotationMatrixToQuaternion(expectedZ90);
    CHECK(quaternionError(quat, Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()))) < kTol);
    CHECK(std::abs(quat.norm() - 1.0) < kTol);
}

void testRoundTrips()
{
    std::mt19937 gen(20241013);

    for (const std::string& order : kOrders)
    {
        for (int i = 0; i < kRandomSamples; ++i)
        {
            const Eigen::Quaterniond q = randomUnitQuaternion(gen);
            const Eigen::Matrix3d R = q.toRotationMatrix();
            const Eigen::AngleAxisd aa = rotationMatrixToAxisAngle(R);
            const Eigen::Vector3d euler = rotationMatrixToEulerAngle(R, order);

            // 以旋转矩阵为基准比较：欧拉角多解不影响结论
            CHECK(rotationError(axisAngleToRotationMatrix(aa), R) < kTol);
            CHECK(rotationError(eulerAngleToRotationMatrix(euler, order), R) < kTol);
            CHECK(rotationError(quaternionToRotationMatrix(rotationMatrixToQuaternion(R)), R) < kTol);
            CHECK(rotationError(axisAngleToRotationMatrix(eulerAngleToAxisAngle(euler, order)), R) < kTol);
            CHECK(rotationError(quaternionToRotationMatrix(eulerAngleToQuaternion(euler, order)), R) < kTol);

            // 中间表示互相转换：一律回到 R 上比较
            CHECK(rotationError(eulerAngleToRotationMatrix(axisAngleToEulerAngle(aa, order), order), R) < kTol);
            CHECK(rotationError(eulerAngleToRotationMatrix(quaternionToEulerAngle(q, order), order), R) < kTol);
            CHECK(rotationError(quaternionToRotationMatrix(axisAngleToQuaternion(aa)), R) < kTol);
            CHECK(rotationError(axisAngleToRotationMatrix(quaternionToAxisAngle(q)), R) < kTol);
            CHECK(quaternionError(rotationMatrixToQuaternion(quaternionToRotationMatrix(q)), q) < kTol);
        }
    }
}

void testEdgeCases()
{
    // 恒等旋转：轴必须仍是单位向量（旧实现返回零轴）
    const Eigen::AngleAxisd identity = rotationMatrixToAxisAngle(Eigen::Matrix3d::Identity());
    CHECK(identity.angle() < kTol);
    CHECK(std::abs(identity.axis().norm() - 1.0) < kTol);
    CHECK((axisAngleToRotationMatrix(identity) - Eigen::Matrix3d::Identity()).norm() < kTol);

    // 恰好 180°：反对称分量退化，必须仍能还原正确的轴（旧实现轴误差约 0.3）
    const std::array<Eigen::Vector3d, 4> axes = {Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(),
                                                  Eigen::Vector3d::UnitZ(), Eigen::Vector3d(1.0, 2.0, 3.0).normalized()};
    for (const Eigen::Vector3d& axis : axes)
    {
        const Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI, axis).toRotationMatrix();
        const Eigen::AngleAxisd aa = rotationMatrixToAxisAngle(R);

        CHECK(std::abs(aa.angle() - M_PI) < 1e-6);
        CHECK(std::abs(std::abs(aa.axis().dot(axis)) - 1.0) < 1e-6);
        CHECK(rotationError(axisAngleToRotationMatrix(aa), R) < 1e-6);
    }

    // 接近 180° 与极小角度
    const Eigen::Vector3d tilted = Eigen::Vector3d(1.0, -2.0, 0.5).normalized();
    for (const double delta : {1e-6, 1e-9, 1e-12})
    {
        const Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI - delta, tilted).toRotationMatrix();
        CHECK(rotationError(axisAngleToRotationMatrix(rotationMatrixToAxisAngle(R)), R) < 1e-6);
    }

    const Eigen::AngleAxisd tiny = rotationMatrixToAxisAngle(rotX(1e-12));
    CHECK(tiny.angle() < kTol);
    CHECK(std::abs(tiny.axis().norm() - 1.0) < kTol);

    // 万向锁：中间角为 90° 时欧拉角不唯一，但旋转必须一致
    const Eigen::Vector3d gimbal(0.5, M_PI / 2.0, 0.3);
    const Eigen::Matrix3d R = eulerAngleToRotationMatrix(gimbal, "xyz");
    CHECK(rotationError(eulerAngleToRotationMatrix(rotationMatrixToEulerAngle(R, "xyz"), "xyz"), R) < kTol);

    // 角度超出 (−π, π] 也不应破坏旋转等价性
    const Eigen::Vector3d wild(3.0 * M_PI, -2.5 * M_PI, 4.0);
    const Eigen::Matrix3d Rw = eulerAngleToRotationMatrix(wild, "yzx");
    CHECK(rotationError(eulerAngleToRotationMatrix(rotationMatrixToEulerAngle(Rw, "yzx"), "yzx"), Rw) < kTol);

    // 零轴与非单位轴的轴角对象：按约定视为恒等 / 自动归一化
    CHECK((axisAngleToRotationMatrix(Eigen::AngleAxisd(0.7, Eigen::Vector3d::Zero())) - Eigen::Matrix3d::Identity()).norm() <
          kTol);
    CHECK((axisAngleToRotationMatrix(Eigen::AngleAxisd(0.7, Eigen::Vector3d(0.0, 0.0, 2.0))) - rotZ(0.7)).norm() < kTol);

    // 非单位四元数：内部归一化后与单位四元数等价
    const Eigen::Quaterniond scaled(1.5, 1.5, 1.5, 1.5);
    CHECK((quaternionToRotationMatrix(scaled) - quaternionToRotationMatrix(Eigen::Quaterniond(0.5, 0.5, 0.5, 0.5))).norm() <
          kTol);
}

void testErrorHandling()
{
    Eigen::Matrix3d notARotation;
    notARotation << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
    const Eigen::Matrix3d reflection = Eigen::Vector3d(1.0, 1.0, -1.0).asDiagonal();

    // 非旋转矩阵：三个入口都应拒绝，而不是静默给出错误结果
    CHECK_THROWS(rotationMatrixToEulerAngle(notARotation, "xyz"), std::invalid_argument);
    CHECK_THROWS(rotationMatrixToAxisAngle(notARotation), std::invalid_argument);
    CHECK_THROWS(rotationMatrixToQuaternion(notARotation), std::invalid_argument);
    CHECK_THROWS(rotationMatrixToAxisAngle(reflection), std::invalid_argument); // 镜像：det = −1
    CHECK(!rotationMatrixIsUnitary(notARotation));
    CHECK(!rotationMatrixIsUnitary(reflection));
    CHECK(rotationMatrixIsUnitary(rotY(0.8)));

    // 非法旋转序列：所有涉及序列的入口都应抛出同一种异常
    CHECK_THROWS(eulerAngleToRotationMatrix(Eigen::Vector3d::Zero(), "xxy"), std::invalid_argument);
    CHECK_THROWS(eulerAngleToQuaternion(Eigen::Vector3d::Zero(), "abc"), std::invalid_argument);
    CHECK_THROWS(eulerAngleToAxisAngle(Eigen::Vector3d::Zero(), "xy"), std::invalid_argument);
    CHECK_THROWS(rotationMatrixToEulerAngle(Eigen::Matrix3d::Identity(), "XY"), std::invalid_argument);
    CHECK_THROWS(axisAngleToEulerAngle(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitX()), "xzz"), std::invalid_argument);
    CHECK_THROWS(quaternionToEulerAngle(Eigen::Quaterniond::Identity(), ""), std::invalid_argument);

    // 零四元数无法表示旋转
    CHECK_THROWS(quaternionToRotationMatrix(Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0)), std::invalid_argument);

    // 大小写混用的序列与全小写等价
    const Eigen::Vector3d euler(0.2, 0.3, -0.4);
    CHECK((eulerAngleToRotationMatrix(euler, "XYZ") - eulerAngleToRotationMatrix(euler, "xyz")).norm() < kTol);
    CHECK(quaternionError(eulerAngleToQuaternion(euler, "ZyX"), eulerAngleToQuaternion(euler, "zyx")) < kTol);
}

void testRoundNearZero()
{
    static_assert(std::is_same<decltype(roundNearZero(Eigen::Matrix3d::Identity())), Eigen::Matrix3d>::value,
                  "roundNearZero 应保持输入的具体类型");
    static_assert(std::is_same<decltype(roundNearZero(Eigen::Vector3d::Zero())), Eigen::Vector3d>::value,
                  "roundNearZero 应保持输入的具体类型");

    const Eigen::Vector3d v(1e-12, -1e-15, 1.0);
    CHECK((roundNearZero(v) - Eigen::Vector3d(0.0, 0.0, 1.0)).norm() < kTol);

    Eigen::Matrix3d m = Eigen::Matrix3d::Identity();
    m(0, 1) = -1e-14;
    CHECK(std::abs(roundNearZero(m)(0, 1)) < kTol);

    const Eigen::Vector3d w(1e-4, 1e-5, 1.0);
    CHECK((roundNearZero(w, 1e-3) - Eigen::Vector3d(0.0, 0.0, 1.0)).norm() < kTol);
    CHECK(std::abs(roundNearZero(Eigen::Vector3d(1e-2, 1.0, 0.0), 1e-3)(0) - 1e-2) < kTol); // 高于阈值不置零

    // 表达式输入同样可用（内部求值到具体类型）
    const Eigen::Vector3d unit = Eigen::Vector3d(3.0, 4.0, 0.0).normalized();
    CHECK((roundNearZero(unit) - unit).norm() < kTol);
}

void testProperties()
{
    std::mt19937 gen(7);

    for (int i = 0; i < kRandomSamples; ++i)
    {
        const Eigen::Quaterniond q = randomUnitQuaternion(gen);
        const Eigen::Matrix3d R = q.toRotationMatrix();
        const Eigen::AngleAxisd aa = rotationMatrixToAxisAngle(R);
        const Eigen::Vector3d euler = rotationMatrixToEulerAngle(R, "xyz");

        // 输出仍是旋转矩阵：正交且行列式为 +1
        for (const Eigen::Matrix3d& candidate :
             {eulerAngleToRotationMatrix(euler, "xyz"), axisAngleToRotationMatrix(aa), quaternionToRotationMatrix(q)})
        {
            CHECK(rotationMatrixIsUnitary(candidate));
            CHECK(std::abs(candidate.determinant() - 1.0) < kTol);
        }

        // 输出仍是单位四元数
        CHECK(std::abs(eulerAngleToQuaternion(euler, "xyz").norm() - 1.0) < kTol);
        CHECK(std::abs(rotationMatrixToQuaternion(R).norm() - 1.0) < kTol);

        // 轴角输出的轴为单位向量、角度落在 [0, π]
        CHECK(std::abs(aa.axis().norm() - 1.0) < kTol);
        CHECK(aa.angle() >= 0.0 && aa.angle() <= M_PI + kTol);
    }
}

} // namespace

int main()
{
    std::cout << "transformation 测试套件\n";
    runGroup("parseOrder：序列解析与大小写", testParseOrder);
    runGroup("已知值：与手算结果对照", testKnownValues);
    runGroup("往返：6 种序列 × 随机旋转", testRoundTrips);
    runGroup("边界：恒等 / 180° / 万向锁 / 零轴", testEdgeCases);
    runGroup("异常：非法矩阵与非法序列", testErrorHandling);
    runGroup("roundNearZero：置零与类型保持", testRoundNearZero);
    runGroup("性质：输出仍是合法旋转", testProperties);

    std::cout << "\n共 " << g_checks << " 项断言，" << g_failures << " 项失败\n";
    return g_failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
