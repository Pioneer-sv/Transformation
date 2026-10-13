# Transformation

[![CI](https://github.com/Pioneer-sv/Transformation/actions/workflows/ci.yml/badge.svg)](https://github.com/Pioneer-sv/Transformation/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-blue.svg)](#依赖)
[![Header-only](https://img.shields.io/badge/header--only-yes-brightgreen.svg)](#集成到自己的项目)
[![Coverage](https://img.shields.io/badge/line%20coverage-100%25-brightgreen.svg)](#覆盖率)

基于 Eigen 的旋转表示转换与姿态轨迹库：**旋转矩阵、欧拉角、轴角、四元数、旋转向量（李代数 so(3)）** 五种表示之间任意互转（20 条转换路径），并提供 **姿态插值（slerp）**、**多点球面样条（squad）** 与 **旋转角 / 测地线距离** 度量。

## 五种旋转表示

| 表示 | 说明 | 数据量 | 优点 | 缺点 |
|------|------|:-----:|------|------|
| 旋转矩阵 | 3×3 正交矩阵 | 9 个 | 组合方便、无歧义 | 参数多、需满足正交约束 |
| 欧拉角 | 绕三轴旋转的角度 | 3 个 | 直观、参数少 | 万向节死锁、序列依赖 |
| 轴角 | 绕单位轴旋转的角度 | 4 个 | 几何意义清晰 | 旋转组合不便 |
| 四元数 | 超复数表示 | 4 个 | 无死锁、插值友好 | 直观性差 |
| 旋转向量 | 李代数 so(3) 元素 | 3 个 | 无冗余、可求导 | 需指数/对数映射 |

旋转向量是唯一**无约束、无冗余**的三参数表示：方向为转轴、模长为旋转角。它是最小二乘优化（姿态图优化、IMU 预积分、手眼标定）与求导/插值的常用参数化方式。

## 特性

- **全组合转换**：5 种表示任意互转（20 条路径），6 种欧拉角序列（大小写均可）
- **李代数映射**：指数映射（Rodrigues 公式，θ → 0 走泰勒级数）与对数映射（最短弧，模长 `[0, π]`）
- **姿态插值**：`slerp` 沿最短弧匀速插值，自动处理四元数双覆盖（q 与 −q）
- **姿态轨迹**：`SquadSpline` 经过全部路点的 C¹ 连续球面样条（squad），端点自然边界
- **角度度量**：`rotationAngle` / `rotationDistance`（双不变测地线距离），小角度下不用 `acos`
- **输入校验**：旋转矩阵必须满足 `RᵀR = I` 且 `det(R) = +1`，非法输入抛 `std::invalid_argument`
- **数值稳健**：θ → 0 与 θ → π 的轴提取不退化；恒等旋转返回单位轴；非单位四元数自动归一化
- **实测数据**：`roundNearZero` 在 512² / 2048² 矩阵上比原双层循环快 **8.2× / 4.5×**（见[性能](#性能)）
- **质量**：行覆盖率 **100%**、分支 99.3%；3.9 万条断言；CI 覆盖 GCC/Clang × Debug/Release × ASan/UBSan × 基准 × 覆盖率

## 目录结构

```
Transformation/
├── CMakeLists.txt                        # 顶层构建：头文件库 target + 选项 + 安装
├── include/
│   └── transformation.h                  # 库本体（header-only，全部 API）
├── tests/
│   └── test_transformation.cpp           # 13 组测试（CTest 注册）
├── examples/
│   └── demo.cpp                          # 转换演示 + 往返自检 + 插值/样条采样
├── benchmarks/
│   └── benchmark_transformation.cpp      # Google Benchmark 性能基准
├── .github/workflows/ci.yml              # CI：构建测试 / 消毒剂 / 基准 / 覆盖率
├── .clang-format
└── LICENSE
```

## 依赖

| 依赖 | 安装命令 | 说明 |
|------|---------|------|
| Eigen3（≥3.3） | `sudo apt install libeigen3-dev` | 线性代数运算 |
| CMake（≥3.22） | `sudo apt install cmake` | 构建系统 |
| C++17 编译器 | `sudo apt install g++` | 库接口要求 C++17 |
| Google Benchmark（可选） | `sudo apt install libbenchmark-dev` | 仅构建性能基准时需要 |

## 构建、测试与运行示例

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j

ctest --test-dir build --output-on-failure   # 运行测试
./build/examples/demo                        # 运行示例
```

示例输出片段：

```
=== 姿态插值：从单位旋转到绕 z 轴 90° ===
  t = 0.500000  与起点的夹角 = 0.785398  旋转向量 =  0.000000 0.000000 0.785398

=== 球面样条 squad：4 个路点之间的平滑轨迹 ===
  u = 1.5  段内进度 0.50  相对路点 1 的转角 0.415529
```

### 构建选项

| 选项 | 默认 | 说明 |
|------|:----:|------|
| `TRANSFORMATION_BUILD_TESTS` | `ON` | 构建测试并注册到 CTest |
| `TRANSFORMATION_BUILD_EXAMPLES` | `ON` | 构建示例程序 |
| `TRANSFORMATION_BUILD_BENCHMARKS` | `OFF` | 构建性能基准（需要 Google Benchmark） |
| `TRANSFORMATION_ENABLE_COVERAGE` | `OFF` | 开启 `--coverage` 覆盖率统计 |
| `TRANSFORMATION_WARNINGS_AS_ERRORS` | `OFF` | 把 `-Wall -Wextra -Wpedantic` 的警告视为错误（CI 中开启） |

## 集成到自己的项目

```cmake
add_subdirectory(third_party/Transformation)          # 或 FetchContent
target_link_libraries(你的目标 PRIVATE transformation::transformation)
```

也可以只把 `include/transformation.h` 拷进工程，编译时加上 Eigen 的头文件路径。

## API

### 转换函数矩阵（20 条路径）

| 从 → 到 | 旋转矩阵 | 欧拉角 | 轴角 | 四元数 | 旋转向量 |
|---------|:-------:|:-----:|:----:|:-----:|:-------:|
| 旋转矩阵 | — | `rotationMatrixToEulerAngle` | `rotationMatrixToAxisAngle` | `rotationMatrixToQuaternion` | `rotationMatrixToRotationVector` |
| 欧拉角 | `eulerAngleToRotationMatrix` | — | `eulerAngleToAxisAngle` | `eulerAngleToQuaternion` | `eulerAngleToRotationVector` |
| 轴角 | `axisAngleToRotationMatrix` | `axisAngleToEulerAngle` | — | `axisAngleToQuaternion` | `axisAngleToRotationVector` |
| 四元数 | `quaternionToRotationMatrix` | `quaternionToEulerAngle` | `quaternionToAxisAngle` | — | `quaternionToRotationVector` |
| 旋转向量 | `rotationVectorToRotationMatrix` | `rotationVectorToEulerAngle` | `rotationVectorToAxisAngle` | `rotationVectorToQuaternion` | — |

涉及欧拉角的函数需要额外传入旋转序列 `order`。类型约定：旋转矩阵 `Eigen::Matrix3d`、欧拉角与旋转向量 `Eigen::Vector3d`、轴角 `Eigen::AngleAxisd`、四元数 `Eigen::Quaterniond`。

### 姿态插值与轨迹

| 接口 | 说明 |
|------|------|
| `slerp(from, to, t)` | 四元数球面线性插值，沿最短弧；允许外插 |
| `interpolate(from, to, t)` | 旋转矩阵 / 轴角的测地线插值（内部走 slerp） |
| `squad(from, fromCtrl, toCtrl, to, t)` | 单段球面四边形插值 |
| `squadControlPoint(prev, current, next)` | 计算 squad 的中间控制四元数 |
| `SquadSpline(waypoints)` | 多点姿态轨迹；`evaluate(u)` / `evaluateMatrix(u)`，u ∈ [0, n−1] |

`SquadSpline` 精确通过每个路点，并在路点处保持角速度连续（C¹）。只有两个路点时退化为 `slerp`。

### 角度度量

| 函数 | 说明 |
|------|------|
| `rotationAngle(R)` | 旋转矩阵对应的旋转角，落在 `[0, π]` |
| `rotationDistance(a, b)` | 两个旋转的测地线距离（双不变度量），矩阵与四元数重载 |

### 辅助函数

| 函数 | 说明 |
|------|------|
| `parseOrder(order)` | 旋转序列转索引顺序，支持大小写混用 |
| `roundNearZero(mat, threshold)` | 将绝对值小于阈值的元素置零（矩阵/向量/表达式均可） |
| `rotationMatrixIsUnitary(R, tol)` | 判断是否为合法旋转矩阵（正交且 `det = +1`，可识别镜像） |

### 错误处理

非法输入统一抛出 `std::invalid_argument`（下标越界为 `std::out_of_range`），库内不打印日志、不做"返回零值"兜底：

| 场景 | 行为 |
|------|------|
| 旋转矩阵非正交或为镜像（`det = −1`） | 抛异常 |
| 欧拉角序列长度不为 3、含重复轴或非法字符 | 抛异常 |
| 四元数模长小于 `kTransformationQuaternionEpsilon` | 抛异常 |
| 轴角对象轴长为 0 或角度为 0 | 视为恒等旋转（返回单位轴） |
| 四元数模长不为 1 | 自动归一化后使用 |
| 路点少于 2 个 | `SquadSpline` 构造抛异常 |

## 使用示例

```cpp
#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include "transformation.h"

int main()
{
    const std::string order = "xyz";
    const Eigen::Vector3d euler(0.3, -0.4, 0.5);

    // 五种表示
    const Eigen::Matrix3d R = eulerAngleToRotationMatrix(euler, order);
    const Eigen::Quaterniond q = eulerAngleToQuaternion(euler, order);
    const Eigen::Vector3d rv = eulerAngleToRotationVector(euler, order); // 李代数 so(3)

    // 指数/对数映射互逆
    std::cout << "旋转角 " << rotationAngle(R) << "，对数映射模长 " << rotationMatrixToRotationVector(R).norm() << "\n";

    // 姿态插值：一半路程
    const Eigen::Quaterniond mid = slerp(Eigen::Quaterniond::Identity(), q, 0.5);
    std::cout << "到中点的夹角 = " << rotationDistance(Eigen::Quaterniond::Identity(), mid) << "\n";

    // 多点姿态轨迹：经过 4 个路点的 C¹ 连续样条
    std::vector<Eigen::Quaterniond> waypoints{Eigen::Quaterniond::Identity(), q,
                                              eulerAngleToQuaternion(euler * 2.0, order),
                                              eulerAngleToQuaternion(-euler, order)};
    const SquadSpline spline(waypoints);
    std::cout << "轨迹中点姿态:\n" << spline.evaluateMatrix(1.5) << "\n";

    return 0;
}
```

## 设计说明

**欧拉角序列语义**：`order` 表示三个角的相乘顺序，即 `R = R_order[0](euler[0]) · R_order[1](euler[1]) · R_order[2](euler[2])`，与 `Eigen::Matrix3d::eulerAngles(i, j, k)` 一致。

**角度范围**：`rotationMatrixToEulerAngle` 沿用 Eigen 的返回范围，且万向锁附近解不唯一——**比较两个欧拉角是否等价时，应比较其对应的旋转矩阵，而不是直接比角度**。

**指数映射**：`R = I + (sinθ/θ)·[v]× + ((1−cosθ)/θ²)·[v]×²`。反对称矩阵直接由旋转向量构造，省去额外归一化；`sinθ/θ` 与 `(1−cosθ)/θ²` 在 `θ < 1e-4` 时改用泰勒级数（`1 − θ²/6`、`0.5 − θ²/24`），否则 `θ = 0` 会得到 `0/0 = NaN`。

**对数映射**：返回最短弧，模长 `[0, π]`。由 `θ = 2·atan2(|vec|, |w|)` 得到角度（不用 `2·acos(w)`，后者在 θ → 0 时精度塌陷），`v = (θ/sin(θ/2))·vec`；`sin(θ/2) → 0` 时退化为 `v ≈ 2·vec`。四元数双覆盖通过强制 `w ≥ 0` 消除。矩阵路径复用轴角提取，继承同样的 θ → 0 / θ → π 处理。

**轴角提取的两种退化**：角度 0 时轴不唯一，统一返回 `(0, UnitX())`；角度 π 时 `R − Rᵀ` 退化，改用对称部分 `R = 2nnᵀ − I` 提取轴。

**插值不做欧拉角线性插值**：欧拉角与旋转之间是非线性映射，逐分量插值既不匀速也不走测地线，靠近万向锁还会剧烈摆动；库内一律转到四元数插值后再转回目标表示。`slerp` 在 `|dot| → 1` 时退化为归一化线性插值。

**squad 轨迹**：控制点取 `s_i = q_i · exp( −( log(q_i⁻¹q_{i+1}) + log(q_i⁻¹q_{i−1}) ) / 4 )`，即两侧角速度的平均，使样条在路点处 C¹ 连续；单段插值为 `squad(q_i, s_i, s_{i+1}, q_{i+1}; t) = slerp( slerp(q_i, q_{i+1}, t), slerp(s_i, s_{i+1}, t), 2t(1−t) )`。端点用自然边界（控制点取路点自身），因此首末端曲率不受约束，但**必定精确通过每个路点**。

**精度与阈值**：`kTransformationZeroThreshold = 1e-10` 抹掉浮点噪声；`kTransformationUnitaryTolerance = 1e-6` 判定矩阵合法性；`kTransformationSeriesThreshold = 1e-4` 切换泰勒级数。用 `acos((tr(AᵀB) − 1) / 2)` 度量旋转误差在误差趋近 0 时导数发散（`1e-13` 会被放大成 `1e-6`），因此测试与 `rotationDistance` 都避免这一形式。

## 性能

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DTRANSFORMATION_BUILD_BENCHMARKS=ON
cmake --build build -j && ./build/benchmarks/benchmark_transformation
```

实测（16 核 x86-64 / GCC 11 / `-O3`，输入取自数组并轮转以规避常量折叠）：

| 基准 | 本库 | 对照实现 | 结论 |
|------|-----:|---------:|------|
| `roundNearZero` 3×3（固定尺寸） | 4.86 ns | 3.94 ns（旧双层循环） | 小矩阵上 `select` 反而慢 ~23% |
| `roundNearZero` 512×512 | 131 µs | 1078 µs（旧双层循环） | **快 8.2×** |
| `roundNearZero` 2048×2048 | 15.6 ms | 69.6 ms（旧双层循环） | **快 4.5×** |
| `eulerAngleToRotationMatrix` | 135 ns | 55.3 ns（手写 AngleAxis 连乘） | 含序列解析与置零 |
| `rotationMatrixToQuaternion` | 29.6 ns | 7.4 ns（`Eigen::Quaterniond(R)`） | 含 `RᵀR`/`det` 校验与归一化 |
| `rotationMatrixToAxisAngle` | 36.7 ns | 43.7 ns（`Eigen::AngleAxisd(R)`） | 含校验仍更快 |
| `slerp` | 58.9 ns | 45.1 ns（`Eigen::Quaternion::slerp`） | 含归一化与双覆盖处理 |
| `rotationDistance` | 51.7 ns | 12.1 ns（`2·acos(|dot|)`） | 慢 4×，但小角度精度远优于 acos |
| `rotationMatrixToRotationVector` | 37.3 ns | — | 对数映射 |
| `SquadSpline::evaluateMatrix` | 212 ns | — | 8 路点样条求值 |

两点如实说明：

1. **向量化只在大矩阵上划算**。3×3 固定尺寸时编译器本来就能把双层循环展开得很干净，`select` 的临时数组反而多花 ~1 ns；但绝对量级都是几纳秒，而 512² 以上有 4–8× 的差距，因此保留 `select` 实现。
2. **校验是有代价的**。`rotationMatrixToQuaternion` 比裸 Eigen 慢 4×，因为多了正交性/行列式校验与置零；换算成绝对值也就 30 ns，对 1 kHz 的控制回路（周期 1 ms）占比不到 0.01%。如果确定输入可信，可以绕过 `rotation*` 入口直接用 Eigen。

## 测试与覆盖率

```bash
ctest --test-dir build --output-on-failure        # 13 组、约 3.9 万条断言

cmake -S . -B build-cov -DCMAKE_BUILD_TYPE=Debug -DTRANSFORMATION_ENABLE_COVERAGE=ON
cmake --build build-cov -j && ctest --test-dir build-cov
gcov -b build-cov/tests/CMakeFiles/transformation_tests.dir/test_transformation.cpp.o
```

当前 `include/transformation.h` 覆盖率：**行 100.00%（227/227）**，分支 99.32%。CI 的 `coverage` 作业用 gcovr 生成 HTML 报告并作为 artifact 上传（`coverage/index.html`）。

| 测试分组 | 内容 |
|----------|------|
| 序列解析 | 6 种序列、大小写混用、长度/重复轴/非法字符异常 |
| 已知值 | 与手算矩阵逐元素对照，验证乘法顺序语义 |
| 往返 | 6 种序列 × 300 个随机旋转 × 20 条转换路径 |
| 边界 | 恒等、恰好 180°、π−1e-12、极小角度、万向锁、超范围角度、零轴/非单位轴、非单位四元数 |
| 旋转向量 | 指数/对数互逆、θ→0 泰勒分支、θ=π、\|v\|>π 最短弧、q 与 −q 一致 |
| 插值 | 端点精确、匀速性、双覆盖、与 Eigen `slerp` 对拍、外插合法 |
| 轨迹 | squad 精确过路点、两路点退化为 slerp、C¹ 连续性（对照逐段 slerp）、路点取反不变 |
| 度量 | 已知值、对称性、双不变性、小角度精度、四元数重载一致 |
| 对拍 | 与 Eigen 的 `eulerAngles` / `AngleAxisd` / `Quaternion` / `slerp` 交叉验证 |
| 异常 | 非正交矩阵、镜像矩阵、非法序列、零四元数、路点不足 |
| 分支 | 小角度一阶近似、内部轴分派工具的防御性分支 |
| 性质 | 输出仍满足正交、`det = +1`、四元数模长为 1、轴角角度落在 `[0, π]` |

## 已知限制

- 只支持 `double` 精度的三维旋转，未提供 `float` 模板化版本
- 万向锁附近欧拉角解不唯一，库不承诺返回与输入完全相同的角度组合（旋转本身正确）
- 欧拉角序列仅支持三条正交坐标轴的 6 种排列，不支持绕任意轴的序列
- squad 端点采用自然边界，首末端曲率不受约束（未做端点速度指定）
- 未涉及 SE(3)（平移 + 旋转的刚体变换）

## 变更记录

### v2.2.0

- **新增**：多点姿态轨迹 `SquadSpline`（squad 球面样条）与 `squad` / `squadControlPoint`；精确过路点、C¹ 连续、两路点退化为 slerp
- **新增**：Google Benchmark 性能基准（`benchmarks/`），量化 `roundNearZero` 向量化收益并给出与 Eigen 基线的对比
- **新增**：覆盖率统计（`TRANSFORMATION_ENABLE_COVERAGE`）与 CI 覆盖率作业（gcovr + HTML artifact），行覆盖率 100%
- **新增**：与 Eigen 内置实现的对拍测试组，以及小角度/防御性分支用例（断言 3.5 万 → 3.9 万）
- **CI**：4 个作业——构建测试矩阵（GCC/Clang × Debug/Release）、ASan+UBSan、性能基准、覆盖率

### v2.1.0

- **新增**：旋转向量（李代数 so(3)）作为第五种表示，新增 8 条转换路径，共 20 条
- **新增**：指数映射与对数映射（最短弧，模长 `[0, π]`）
- **新增**：姿态插值 `slerp` 与矩阵/轴角 `interpolate`
- **新增**：角度度量 `rotationAngle`、`rotationDistance`
- **测试**：断言 2.1 万 → 3.5 万

### v2.0.0

- **修复**：旋转角恰好为 π 时轴提取退化为 `0/0`（旧实现轴误差约 0.3）
- **修复**：恒等旋转返回零轴，违反 `AngleAxisd` 单位轴约定
- **修复**：非正交/镜像矩阵被静默接受并产出错误结果
- **修复**：大写旋转序列在 `eulerAngleTo*` 中抛 `std::out_of_range`
- **改进**：取消"打印 stderr 后返回零值"的兜底策略；`roundNearZero` 改用 Eigen `select`；去掉每次调用构造的哈希表与 `std::function`
- **工程**：标准目录布局、CTest、GitHub Actions、`.clang-format`、MIT 许可证

### 1.0.0

- 首个版本：12 条转换路径、6 种欧拉角序列、往返自检程序

## 许可证

[MIT](LICENSE)
