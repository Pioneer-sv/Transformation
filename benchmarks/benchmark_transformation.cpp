/**
 * @file benchmark_transformation.cpp
 * @brief 性能基准：量化 roundNearZero 向量化改造的收益，并给出各接口开销与 Eigen 直接实现的对比。
 *
 * 构建运行（需要 Google Benchmark）：
 *   cmake -S . -B build -DTRANSFORMATION_BUILD_BENCHMARKS=ON -DCMAKE_BUILD_TYPE=Release
 *   cmake --build build -j && ./build/benchmarks/benchmark_transformation
 *
 * 基准写法说明：
 *  1. 转换类接口的输入取自预先算好的数组并按下标轮转，避免编译器把常量输入的计算提出循环
 *     （常量折叠 / LICM）从而测出空壳；"ours" 与 "eigen" 两组只差调用的实现；
 *  2. DoNotOptimize 一律传非 const 左值：Google Benchmark 1.8 起 DoNotOptimize(const Tp&) 已废弃，
 *     传 const 对象会触发 deprecated 警告（CI 开了 -Werror）。
 */

#include <cmath>
#include <cstddef>
#include <random>
#include <vector>

#include <benchmark/benchmark.h>
#include <Eigen/Dense>

#include "transformation.h"

namespace
{

constexpr double kNoise = 1e-12; // 小于 kTransformationZeroThreshold 的量级

/** v2.0.0 之前的实现：双层 for 循环逐元素置零，仅用于对比 */
template <class Derived>
typename Derived::PlainObject roundNearZeroLoop(const Eigen::MatrixBase<Derived>& mat,
                                                double threshold = kTransformationZeroThreshold)
{
    typename Derived::PlainObject result = mat.eval();
    for (int i = 0; i < result.rows(); ++i)
    {
        for (int j = 0; j < result.cols(); ++j)
        {
            if (std::abs(result(i, j)) < threshold)
            {
                result(i, j) = 0.0;
            }
        }
    }
    return result;
}

/** 构造含少量噪声元素的矩阵，模拟旋转矩阵转换后的输出 */
Eigen::MatrixXd noisyMatrix(int size)
{
    Eigen::MatrixXd matrix = Eigen::MatrixXd::Random(size, size);
    const int noisyCount = size * size / 10;
    for (int i = 0; i < noisyCount; ++i)
    {
        matrix(i % size, (i / size) % size) = kNoise;
    }
    return matrix;
}

/** 预先生成的一批输入，基准循环里按下标轮转取用 */
struct Inputs
{
    static constexpr std::size_t kCount = 512;

    std::vector<Eigen::Vector3d> eulers;
    std::vector<Eigen::Matrix3d> matrices;
    std::vector<Eigen::Quaterniond> quaternions;

    Inputs()
    {
        std::mt19937 gen(2026);
        std::uniform_real_distribution<double> uniform(-1.0, 1.0);

        for (std::size_t i = 0; i < kCount; ++i)
        {
            const Eigen::Vector3d euler(M_PI * uniform(gen), M_PI * uniform(gen), M_PI * uniform(gen));
            eulers.push_back(euler);
            matrices.push_back(eulerAngleToRotationMatrix(euler, "xyz"));
            quaternions.push_back(eulerAngleToQuaternion(euler, "xyz"));
        }
    }
};

const Inputs& inputs()
{
    static const Inputs data;
    return data;
}

// ---------------------------------------------------------------- roundNearZero

/** 3×3 旋转矩阵（库的典型用法）：固定尺寸，编译器可完全展开 */
static void BM_RoundNearZeroFixed(benchmark::State& state)
{
    const bool vectorized = state.range(0) != 0;
    Eigen::Matrix3d input = Eigen::Matrix3d::Identity();
    input(0, 1) = kNoise;
    input(2, 0) = -kNoise;
    benchmark::DoNotOptimize(input);

    for (auto _ : state)
    {
        Eigen::Matrix3d output = vectorized ? roundNearZero(input) : roundNearZeroLoop(input);
        benchmark::DoNotOptimize(output);
    }
}
BENCHMARK(BM_RoundNearZeroFixed)->Arg(0)->Arg(1)->Unit(benchmark::kNanosecond);

/** 动态尺寸大矩阵：放大向量化与逐元素循环的差异 */
static void BM_RoundNearZeroDynamic(benchmark::State& state)
{
    const bool vectorized = state.range(0) != 0;
    const int size = static_cast<int>(state.range(1));
    const Eigen::MatrixXd input = noisyMatrix(size);

    for (auto _ : state)
    {
        const Eigen::MatrixXd output = vectorized ? roundNearZero(input) : roundNearZeroLoop(input);
        benchmark::DoNotOptimize(output.data());
    }
    state.SetItemsProcessed(state.iterations() * input.size());
}
BENCHMARK(BM_RoundNearZeroDynamic)
        ->Args({0, 512})
        ->Args({1, 512})
        ->Args({0, 2048})
        ->Args({1, 2048})
        ->Unit(benchmark::kMicrosecond);

// ---------------------------------------------------------------- 转换接口
// Arg(0)：本库实现；Arg(1)：Eigen 直接实现（基线）

static void BM_EulerToRotationMatrix(benchmark::State& state)
{
    const bool ours = state.range(0) == 0;
    const std::vector<Eigen::Vector3d>& eulers = inputs().eulers;
    std::size_t index = 0;

    for (auto _ : state)
    {
        const Eigen::Vector3d& euler = eulers[index++ % eulers.size()];
        Eigen::Matrix3d result =
                ours ? eulerAngleToRotationMatrix(euler, "xyz")
                     : (Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()) *
                        Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
                        Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ()))
                               .toRotationMatrix();
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_EulerToRotationMatrix)->Arg(0)->Arg(1)->Unit(benchmark::kNanosecond);

static void BM_MatrixToQuaternion(benchmark::State& state)
{
    const bool ours = state.range(0) == 0;
    const std::vector<Eigen::Matrix3d>& matrices = inputs().matrices;
    std::size_t index = 0;

    for (auto _ : state)
    {
        const Eigen::Matrix3d& matrix = matrices[index++ % matrices.size()];
        Eigen::Quaterniond result = ours ? rotationMatrixToQuaternion(matrix) : Eigen::Quaterniond(matrix);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_MatrixToQuaternion)->Arg(0)->Arg(1)->Unit(benchmark::kNanosecond);

static void BM_MatrixToAxisAngle(benchmark::State& state)
{
    const bool ours = state.range(0) == 0;
    const std::vector<Eigen::Matrix3d>& matrices = inputs().matrices;
    std::size_t index = 0;

    for (auto _ : state)
    {
        const Eigen::Matrix3d& matrix = matrices[index++ % matrices.size()];
        Eigen::AngleAxisd result = ours ? rotationMatrixToAxisAngle(matrix) : Eigen::AngleAxisd(matrix);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_MatrixToAxisAngle)->Arg(0)->Arg(1)->Unit(benchmark::kNanosecond);

static void BM_MatrixToRotationVector(benchmark::State& state)
{
    const std::vector<Eigen::Matrix3d>& matrices = inputs().matrices;
    std::size_t index = 0;

    for (auto _ : state)
    {
        const Eigen::Matrix3d& matrix = matrices[index++ % matrices.size()];
        Eigen::Vector3d result = rotationMatrixToRotationVector(matrix);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_MatrixToRotationVector)->Unit(benchmark::kNanosecond);

static void BM_Slerp(benchmark::State& state)
{
    const bool ours = state.range(0) == 0;
    const std::vector<Eigen::Quaterniond>& quaternions = inputs().quaternions;
    const Eigen::Quaterniond from = Eigen::Quaterniond::Identity();
    std::size_t index = 0;

    for (auto _ : state)
    {
        const Eigen::Quaterniond& to = quaternions[index++ % quaternions.size()];
        Eigen::Quaterniond result = ours ? slerp(from, to, 0.37) : from.slerp(0.37, to);
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_Slerp)->Arg(0)->Arg(1)->Unit(benchmark::kNanosecond);

static void BM_RotationDistance(benchmark::State& state)
{
    const bool ours = state.range(0) == 0;
    const std::vector<Eigen::Quaterniond>& quaternions = inputs().quaternions;
    const Eigen::Quaterniond from = Eigen::Quaterniond::Identity();
    std::size_t index = 0;

    for (auto _ : state)
    {
        const Eigen::Quaterniond& to = quaternions[index++ % quaternions.size()];
        double result = ours ? rotationDistance(from, to) : 2.0 * std::acos(std::min(1.0, std::abs(from.dot(to))));
        benchmark::DoNotOptimize(result);
    }
}
BENCHMARK(BM_RotationDistance)->Arg(0)->Arg(1)->Unit(benchmark::kNanosecond);

/** squad 轨迹求值：8 个路点，参数逐轮推进以覆盖整条轨迹 */
static void BM_SquadEvaluate(benchmark::State& state)
{
    const std::vector<Eigen::Quaterniond>& quaternions = inputs().quaternions;
    const std::vector<Eigen::Quaterniond> waypoints(quaternions.begin(), quaternions.begin() + 8);
    const SquadSpline spline(waypoints);

    double u = 0.0;
    for (auto _ : state)
    {
        Eigen::Matrix3d result = spline.evaluateMatrix(u);
        benchmark::DoNotOptimize(result);

        u += 0.001;
        if (u > 7.0)
        {
            u = 0.0;
        }
    }
}
BENCHMARK(BM_SquadEvaluate)->Unit(benchmark::kNanosecond);

} // namespace

BENCHMARK_MAIN();
