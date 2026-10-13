# Transformation

[![CI](https://github.com/Pioneer-sv/Transformation/actions/workflows/ci.yml/badge.svg)](https://github.com/Pioneer-sv/Transformation/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-blue.svg)](#依赖)
[![Header-only](https://img.shields.io/badge/header--only-yes-brightgreen.svg)](#集成到自己的项目)

基于 Eigen 的旋转表示转换库，支持**旋转矩阵、欧拉角、轴角、四元数**四种表示之间的任意相互转换（12 条转换路径全覆盖）。

## 四种旋转表示

| 表示 | 说明 | 数据量 | 优点 | 缺点 |
|------|------|:-----:|------|------|
| 旋转矩阵 | 3×3 正交矩阵 | 9 个 | 组合方便、无歧义 | 参数多、需满足正交约束 |
| 欧拉角 | 绕三轴旋转的角度 | 3 个 | 直观、参数少 | 万向节死锁、序列依赖 |
| 轴角 | 绕单位轴旋转的角度 | 4 个 | 几何意义清晰 | 旋转组合不便 |
| 四元数 | 超复数表示 | 4 个 | 无死锁、插值友好 | 直观性差 |

## 特性

- **全组合转换**：4 种表示之间任意互转，6 种欧拉角序列（`xyz`/`xzy`/`yxz`/`yzx`/`zxy`/`zyx`，大小写均可）
- **输入校验**：旋转矩阵必须满足 `RᵀR = I` 且 `det(R) = +1`，非法输入抛出 `std::invalid_argument`，不会静默返回错误结果
- **数值稳健**：旋转角为 0 或 π 时轴角提取不退化为 `0/0`；恒等旋转返回单位轴而不是零轴；非单位四元数自动归一化
- **按元素向量化**：`roundNearZero` 基于 Eigen `select` 实现，替代手写双层循环
- **无额外依赖**：仅需 Eigen3，header-only，直接 `#include` 即可
- **2.1 万条断言的测试套件** + GitHub Actions 持续集成（GCC/Clang × Debug/Release × ASan/UBSan）

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
│   └── demo.cpp                # 用法示例 + 12 条路径往返自检
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

`build/examples/demo` 会打印同一个旋转的四种表示，并输出 12 条转换路径的往返误差：

```
=== 12 条转换路径往返自检 ===
[PASS] R -> Euler -> R  误差 = 0.000000
...
结果: 12/12 通过
```

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

### 转换函数矩阵

| 从 → 到 | 旋转矩阵 | 欧拉角 | 轴角 | 四元数 |
|---------|:-------:|:-----:|:----:|:-----:|
| 旋转矩阵 | — | `rotationMatrixToEulerAngle(R, order)` | `rotationMatrixToAxisAngle(R)` | `rotationMatrixToQuaternion(R)` |
| 欧拉角 | `eulerAngleToRotationMatrix(euler, order)` | — | `eulerAngleToAxisAngle(euler, order)` | `eulerAngleToQuaternion(euler, order)` |
| 轴角 | `axisAngleToRotationMatrix(aa)` | `axisAngleToEulerAngle(aa, order)` | — | `axisAngleToQuaternion(aa)` |
| 四元数 | `quaternionToRotationMatrix(q)` | `quaternionToEulerAngle(q, order)` | `quaternionToAxisAngle(q)` | — |

类型约定：旋转矩阵 `Eigen::Matrix3d`、欧拉角 `Eigen::Vector3d`、轴角 `Eigen::AngleAxisd`、四元数 `Eigen::Quaterniond`。

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
| 旋转矩阵非正交或为镜像（`det = −1`） | `rotationMatrixTo*` 抛异常 |
| 欧拉角序列长度不为 3、含重复轴或非法字符 | 所有涉及 `order` 的函数抛异常 |
| 四元数模长小于 `kTransformationQuaternionEpsilon` | `quaternionTo*` 抛异常 |
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

    // 欧拉角 → 旋转矩阵 / 轴角 / 四元数
    const Eigen::Matrix3d R = eulerAngleToRotationMatrix(euler, order);
    const Eigen::AngleAxisd aa = eulerAngleToAxisAngle(euler, order);
    const Eigen::Quaterniond q = eulerAngleToQuaternion(euler, order);

    std::cout << "旋转矩阵:\n" << R << "\n";
    std::cout << "轴角: " << aa.angle() << " 绕 " << aa.axis().transpose() << "\n";
    std::cout << "四元数(x y z w): " << q.coeffs().transpose() << "\n";

    // 反向转换：非法输入会抛 std::invalid_argument
    try
    {
        const Eigen::Vector3d back = rotationMatrixToEulerAngle(R, order);
        std::cout << "回到欧拉角: " << back.transpose() << "\n";
    }
    catch (const std::invalid_argument& e)
    {
        std::cerr << "转换失败: " << e.what() << "\n";
    }

    return 0;
}
```

## 设计说明

**欧拉角序列语义**：`order` 表示三个角的相乘顺序，即
`R = R_order[0](euler[0]) · R_order[1](euler[1]) · R_order[2](euler[2])`，
与 `Eigen::Matrix3d::eulerAngles(i, j, k)` 的约定一致。例如 `"zyx"` 对应 `Rz(e0)·Ry(e1)·Rx(e2)`。

**角度范围**：`rotationMatrixToEulerAngle` 沿用 Eigen 的返回范围（部分序列下第二个角落在 `[0, π]`），并且万向锁附近解不唯一。因此**比较两个欧拉角是否等价时，应比较它们对应的旋转矩阵，而不是直接比角度**——测试套件就是这么做的。

**轴角提取的两种退化情况**：

- 旋转角为 0：轴不唯一，统一返回 `(0, UnitX())`，保证 `AngleAxisd` 的"单位轴"约定成立（否则后续插值/组合会出现零轴）；
- 旋转角为 π：`sin(θ) ≈ 0` 使反对称分量 `R − Rᵀ` 退化，此时改用对称部分 `R = 2nnᵀ − I` 提取轴，避免 `0/0`。

**精度与阈值**：`kTransformationZeroThreshold = 1e-10` 用于抹掉浮点噪声（如 `1e-17`、`−0.0`）；`kTransformationUnitaryTolerance = 1e-6` 用于判定矩阵是否为合法旋转。测试中发现：用 `acos((tr(AᵀB) − 1) / 2)` 度量旋转误差在误差趋近 0 时导数发散，会把 `1e-13` 放大成 `1e-6`，因此测试改用矩阵差的 Frobenius 范数。

## 测试

```bash
ctest --test-dir build --output-on-failure
```

覆盖内容（共约 2.1 万条断言）：

| 分组 | 内容 |
|------|------|
| 序列解析 | 6 种序列、大小写混用、长度/重复轴/非法字符异常 |
| 已知值 | 与手算矩阵逐元素对照，验证乘法顺序语义 |
| 往返 | 6 种序列 × 300 个随机旋转 × 12 条转换路径 |
| 边界 | 恒等、恰好 180°、π−1e-12、极小角度、万向锁、超范围角度、零轴/非单位轴、非单位四元数 |
| 异常 | 非正交矩阵、镜像矩阵（det = −1）、非法序列、零四元数 |
| 性质 | 输出仍满足正交、`det = +1`、四元数模长为 1、轴角角度落在 `[0, π]` |

## 已知限制

- 只支持 `double` 精度的三维旋转；未提供 `float` 模板化版本
- 万向锁附近欧拉角解不唯一，库不承诺返回与输入完全相同的角度组合（旋转本身正确）
- 欧拉角序列仅支持三条正交坐标轴的 6 种排列，不支持绕任意轴的序列
- 未提供旋转插值（slerp）与旋转导数/李代数接口

## 变更记录

### v2.0.0

- **修复**：旋转角恰好为 π 时轴提取退化为 `0/0`（旧实现轴误差约 0.3），改用对称部分提取
- **修复**：恒等旋转返回零轴，违反 `AngleAxisd` 单位轴约定，现返回 `(0, UnitX())`
- **修复**：非正交/镜像矩阵被静默接受并产出错误结果，现统一抛 `std::invalid_argument`
- **修复**：大写旋转序列在 `eulerAngleTo*` 中抛 `std::out_of_range`（`parseOrder` 却支持大写），现全部大小写无关
- **改进**：`handleException` 打印到 stderr 后返回零值的策略取消，库不再吞掉错误
- **改进**：`roundNearZero` 由双层循环改为 Eigen `select`（按元素向量化）
- **改进**：去掉每次调用都构造的 `unordered_map<char, std::function<...>>`，改为 `switch` 分派
- **工程**：标准目录布局、CTest 测试套件、GitHub Actions（GCC/Clang、Debug/Release、ASan/UBSan）、`.clang-format`、MIT 许可证

### 1.0.0

- 首个版本：12 条转换路径、6 种欧拉角序列、往返自检程序

## 许可证

[MIT](LICENSE)
