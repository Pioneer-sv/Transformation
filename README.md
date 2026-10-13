# Transformation

[![CI](https://github.com/Pioneer-sv/Transformation/actions/workflows/ci.yml/badge.svg)](https://github.com/Pioneer-sv/Transformation/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-blue.svg)](#依赖)
[![Header-only](https://img.shields.io/badge/header--only-yes-brightgreen.svg)](#集成到自己的项目)

基于 Eigen 的旋转表示转换库：**旋转矩阵、欧拉角、轴角、四元数、旋转向量（李代数 so(3)）** 五种表示之间任意互转（20 条转换路径全覆盖），并提供 **姿态插值（slerp）** 与 **旋转角 / 测地线距离** 度量。

## 五种旋转表示

| 表示 | 说明 | 数据量 | 优点 | 缺点 |
|------|------|:-----:|------|------|
| 旋转矩阵 | 3×3 正交矩阵 | 9 个 | 组合方便、无歧义 | 参数多、需满足正交约束 |
| 欧拉角 | 绕三轴旋转的角度 | 3 个 | 直观、参数少 | 万向节死锁、序列依赖 |
| 轴角 | 绕单位轴旋转的角度 | 4 个 | 几何意义清晰 | 旋转组合不便 |
| 四元数 | 超复数表示 | 4 个 | 无死锁、插值友好 | 直观性差 |
| 旋转向量 | 李代数 so(3) 元素 | 3 个 | 无冗余、可求导 | 需指数/对数映射 |

旋转向量是唯一**无约束、无冗余**的三参数表示：方向为转轴、模长为旋转角。它是最小二乘优化（姿态图优化、IMU 预积分、手眼标定）与线性插值/求导的常用参数化方式。

## 特性

- **全组合转换**：5 种表示之间任意互转（20 条路径），6 种欧拉角序列（`xyz`/`xzy`/`yxz`/`yzx`/`zxy`/`zyx`，大小写均可）
- **李代数映射**：旋转向量的指数映射（Rodrigues 公式，θ → 0 用泰勒级数）与对数映射（返回最短弧，模长落在 `[0, π]`）
- **姿态插值**：`slerp` 沿最短弧匀速插值，自动处理四元数双覆盖（q 与 −q）；矩阵与轴角插值沿测地线，与四元数结果一致
- **角度度量**：`rotationAngle` / `rotationDistance`（双不变测地线距离），小角度下不使用 `acos` 以免精度塌陷
- **输入校验**：旋转矩阵必须满足 `RᵀR = I` 且 `det(R) = +1`，非法输入抛出 `std::invalid_argument`，不会静默返回错误结果
- **数值稳健**：θ → 0 与 θ → π 的轴角提取不退化；恒等旋转返回单位轴而非零轴；非单位四元数自动归一化
- **按元素向量化**：`roundNearZero` 基于 Eigen `select` 实现
- **无额外依赖**：仅需 Eigen3，header-only
- **3.5 万条断言的测试套件** + GitHub Actions（GCC/Clang × Debug/Release × ASan/UBSan）

## 目录结构

```
Transformation/
├── CMakeLists.txt              # 顶层构建：头文件库 target + 选项 + 安装
├── include/
│   └── transformation.h        # 库本体（header-only，全部 API）
├── tests/
│   ├── CMakeLists.txt
│   └── test_transformation.cpp # 测试套件（CTest 注册）
├── examples/
│   ├── CMakeLists.txt
│   └── demo.cpp                # 用法示例：转换演示 + 12 条路径往返自检 + 插值采样
├── .github/workflows/ci.yml    # 持续集成
├── .clang-format
└── LICENSE
```

## 依赖

| 依赖 | 安装命令 | 说明 |
|------|---------|------|
| Eigen3（≥3.3） | `sudo apt install libeigen3-dev` | 线性代数运算 |
| CMake（≥3.22） | `sudo apt install cmake` | 构建系统 |
| C++17 编译器 | `sudo apt install g++` | 库接口要求 C++17 |

## 构建、测试与运行示例

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j

ctest --test-dir build --output-on-failure   # 运行测试
./build/examples/demo                        # 运行示例
```

示例输出片段（完整输出见 `demo.cpp`）：

```
=== 姿态插值：从单位旋转到绕 z 轴 90° ===
  t = 0.000000  与起点的夹角 = 0.000000  旋转向量 =  0.000000 0.000000 0.000000
  t = 0.250000  与起点的夹角 = 0.392699  旋转向量 =  0.000000 0.000000 0.392699
  t = 0.500000  与起点的夹角 = 0.785398  旋转向量 =  0.000000 0.000000 0.785398
  t = 0.750000  与起点的夹角 = 1.178097  旋转向量 =  0.000000 0.000000 1.178097
  t = 1.000000  与起点的夹角 = 1.570796  旋转向量 =  0.000000 0.000000 1.570796
```

夹角随 `t` 线性增长，即插值结果是匀速的（测地线）。

### 构建选项

| 选项 | 默认 | 说明 |
|------|:----:|------|
| `TRANSFORMATION_BUILD_TESTS` | `ON` | 构建测试并注册到 CTest |
| `TRANSFORMATION_BUILD_EXAMPLES` | `ON` | 构建示例程序 |
| `TRANSFORMATION_WARNINGS_AS_ERRORS` | `OFF` | 把 `-Wall -Wextra -Wpedantic` 的警告视为错误（CI 中开启） |

## 集成到自己的项目

```cmake
# 方式一：作为子目录
add_subdirectory(third_party/Transformation)
target_link_libraries(你的目标 PRIVATE transformation::transformation)

# 方式二：FetchContent
include(FetchContent)
FetchContent_Declare(transformation
        GIT_REPOSITORY https://github.com/Pioneer-sv/Transformation.git
        GIT_TAG main)
FetchContent_MakeAvailable(transformation)
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

### 姿态插值

| 函数 | 说明 |
|------|------|
| `slerp(from, to, t)` | 四元数球面线性插值，沿最短弧；`t = 0` 为 `from`、`t = 1` 为 `to`，允许外插 |
| `interpolate(from, to, t)` | 旋转矩阵或轴角版本的测地线插值（内部走 slerp） |

### 角度度量

| 函数 | 说明 |
|------|------|
| `rotationAngle(R)` | 旋转矩阵对应的旋转角，落在 `[0, π]` |
| `rotationDistance(a, b)` | 两个旋转的测地线距离（双不变度量），支持矩阵与四元数重载 |

### 辅助函数

| 函数 | 说明 |
|------|------|
| `parseOrder(order)` | 旋转序列转索引顺序，支持大小写混用 |
| `roundNearZero(mat, threshold)` | 将绝对值小于阈值的元素置零（矩阵/向量/表达式均可） |
| `rotationMatrixIsUnitary(R, tol)` | 判断是否为合法旋转矩阵（正交且 `det = +1`，可识别镜像） |

### 错误处理

非法输入统一抛出 `std::invalid_argument`，调用方自行决定如何处理；库内不打印日志、不做"返回零值"兜底：

| 场景 | 行为 |
|------|------|
| 旋转矩阵非正交或为镜像（`det = −1`） | 抛异常 |
| 欧拉角序列长度不为 3、含重复轴或非法字符 | 抛异常 |
| 四元数模长小于 `kTransformationQuaternionEpsilon` | 抛异常 |
| 轴角对象轴长为 0 或角度为 0 | 视为恒等旋转（返回单位轴），不抛异常 |
| 四元数模长不为 1 | 自动归一化后使用 |

## 使用示例

```cpp
#include <Eigen/Dense>
#include <iostream>
#include "transformation.h"

int main()
{
    const std::string order = "xyz";
    const Eigen::Vector3d euler(0.3, -0.4, 0.5);

    // 五种表示
    const Eigen::Matrix3d R = eulerAngleToRotationMatrix(euler, order);
    const Eigen::AngleAxisd aa = eulerAngleToAxisAngle(euler, order);
    const Eigen::Quaterniond q = eulerAngleToQuaternion(euler, order);
    const Eigen::Vector3d rv = eulerAngleToRotationVector(euler, order); // 李代数 so(3)

    // 旋转向量的指数/对数映射互逆
    const Eigen::Vector3d logR = rotationMatrixToRotationVector(R);
    std::cout << "旋转角 " << rotationAngle(R) << "，对数映射模长 " << logR.norm() << "\n";

    // 姿态插值：一半路程
    const Eigen::Quaterniond mid = slerp(Eigen::Quaterniond::Identity(), q, 0.5);
    std::cout << "到中点的夹角 = " << rotationDistance(Eigen::Quaterniond::Identity(), mid) << "\n";

    return 0;
}
```

## 设计说明

**欧拉角序列语义**：`order` 表示三个角的相乘顺序，即
`R = R_order[0](euler[0]) · R_order[1](euler[1]) · R_order[2](euler[2])`，
与 `Eigen::Matrix3d::eulerAngles(i, j, k)` 的约定一致。例如 `"zyx"` 对应 `Rz(e0)·Ry(e1)·Rx(e2)`。

**角度范围**：`rotationMatrixToEulerAngle` 沿用 Eigen 的返回范围，并且万向锁附近解不唯一。因此**比较两个欧拉角是否等价时，应比较它们对应的旋转矩阵，而不是直接比角度**。

**指数映射（`rotationVector → 旋转矩阵`）**：使用 Rodrigues 公式 `R = I + (sinθ/θ)·[v]× + ((1−cosθ)/θ²)·[v]×²`。反对称矩阵直接由旋转向量构造，避免额外归一化；`sinθ/θ` 与 `(1−cosθ)/θ²` 在 `θ < 1e-4` 时改用泰勒级数（`1 − θ²/6` 与 `0.5 − θ²/24`）。若直接写成 `sinθ/θ`，`θ = 0` 会得到 `0/0 = NaN`。

**对数映射（`→ 旋转向量`）**：返回最短弧表示，模长落在 `[0, π]`。先由 `|vec(q)| = sin(θ/2)` 与 `w = cos(θ/2)` 得到 `θ = 2·atan2(|vec|, |w|)`，再取 `v = (θ/sin(θ/2))·vec`；`sin(θ/2) → 0` 时退化为 `v ≈ 2·vec`（一阶展开）。四元数的双覆盖通过强制 `w ≥ 0` 消除。矩阵路径复用轴角提取，因此同样继承 θ → 0 / θ → π 的处理。

**轴角提取的两种退化情况**：

- 旋转角为 0：轴不唯一，统一返回 `(0, UnitX())`，保证 `AngleAxisd` 的"单位轴"约定成立；
- 旋转角为 π：`sin(θ) ≈ 0` 使反对称分量 `R − Rᵀ` 退化，此时改用对称部分 `R = 2nnᵀ − I` 提取轴，避免 `0/0`。

**插值的表示选择**：不在欧拉角上做线性插值——欧拉角与旋转之间是非线性映射，逐分量插值既不匀速也不走测地线，靠近万向锁还会剧烈摆动。因此库内一律先转到四元数做 `slerp`，再转回目标表示。`slerp` 中 `|dot| → 1` 时改用归一化线性插值（nlerp），避免 `sinθ → 0` 的除法。

**精度与阈值**：`kTransformationZeroThreshold = 1e-10` 抹掉浮点噪声；`kTransformationUnitaryTolerance = 1e-6` 判定矩阵合法性；`kTransformationSeriesThreshold = 1e-4` 切换泰勒级数。测试中发现：用 `acos((tr(AᵀB) − 1) / 2)` 度量旋转误差在误差趋近 0 时导数发散，会把 `1e-13` 放大成 `1e-6`，因此测试与 `rotationDistance` 都避免这一形式。

## 测试

```bash
ctest --test-dir build --output-on-failure
```

覆盖内容（10 组、约 3.5 万条断言）：

| 分组 | 内容 |
|------|------|
| 序列解析 | 6 种序列、大小写混用、长度/重复轴/非法字符异常 |
| 已知值 | 与手算矩阵逐元素对照，验证乘法顺序语义 |
| 往返 | 6 种序列 × 300 个随机旋转 × 20 条转换路径 |
| 边界 | 恒等、恰好 180°、π−1e-12、极小角度、万向锁、超范围角度、零轴/非单位轴、非单位四元数 |
| 旋转向量 | 指数/对数互逆、θ→0 泰勒分支、θ=π、\|v\|>π 的最短弧、q 与 −q 一致性 |
| 插值 | 端点精确、匀速性、双覆盖、与 Eigen 内置 `slerp` 对拍、矩阵/轴角一致性、外插合法 |
| 度量 | 旋转角已知值、对称性、双不变性（左右同乘不变量）、小角度精度、四元数重载一致 |
| 异常 | 非正交矩阵、镜像矩阵、非法序列、零四元数 |
| 性质 | 输出仍满足正交、`det = +1`、四元数模长为 1、轴角角度落在 `[0, π]` |

## 已知限制

- 只支持 `double` 精度的三维旋转，未提供 `float` 模板化版本
- 万向锁附近欧拉角解不唯一，库不承诺返回与输入完全相同的角度组合（旋转本身正确）
- 欧拉角序列仅支持三条正交坐标轴的 6 种排列，不支持绕任意轴的序列
- 插值只提供两点之间的 `slerp`；未提供多控制点的球面样条（squad）
- 未涉及 SE(3)（平移 + 旋转的刚体变换）

## 变更记录

### v2.1.0

- **新增**：旋转向量（李代数 so(3)）作为第五种表示，新增 8 条转换路径，共 20 条
- **新增**：指数映射 `rotationVectorToRotationMatrix`（Rodrigues + θ→0 泰勒级数）、`rotationVectorToQuaternion`
- **新增**：对数映射 `rotationMatrixToRotationVector`、`quaternionToRotationVector`（最短弧，模长 `[0, π]`）
- **新增**：姿态插值 `slerp`（最短弧、双覆盖处理、近平行退化处理）与矩阵/轴角 `interpolate`
- **新增**：角度度量 `rotationAngle`、`rotationDistance`（矩阵与四元数重载，小角度不使用 `acos`）
- **测试**：新增 3 组用例（旋转向量、插值、度量），断言数 2.1 万 → 3.5 万；插值新增与 Eigen 内置 `slerp` 的对拍

### v2.0.0

- **修复**：旋转角恰好为 π 时轴提取退化为 `0/0`（旧实现轴误差约 0.3），改用对称部分提取
- **修复**：恒等旋转返回零轴，违反 `AngleAxisd` 单位轴约定，现返回 `(0, UnitX())`
- **修复**：非正交/镜像矩阵被静默接受并产出错误结果，现统一抛 `std::invalid_argument`
- **修复**：大写旋转序列在 `eulerAngleTo*` 中抛 `std::out_of_range`（`parseOrder` 却支持大写），现全部大小写无关
- **改进**：取消 `handleException` 打印 stderr 后返回零值的策略，库不再吞掉错误
- **改进**：`roundNearZero` 由双层循环改为 Eigen `select`（按元素向量化）
- **改进**：去掉每次调用都构造的 `unordered_map<char, std::function<...>>`，改为 `switch` 分派
- **工程**：标准目录布局、CTest 测试套件、GitHub Actions（GCC/Clang、Debug/Release、ASan/UBSan）、`.clang-format`、MIT 许可证

### 1.0.0

- 首个版本：12 条转换路径、6 种欧拉角序列、往返自检程序

## 许可证

[MIT](LICENSE)
