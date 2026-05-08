# Transformation

基于 Eigen 的旋转表示转换库，支持**旋转矩阵、欧拉角、轴角、四元数**四种表示之间的任意相互转换。

## 四种旋转表示

| 表示 | 说明 | 数据量 | 优点 | 缺点 |
|------|------|:-----:|------|------|
| 旋转矩阵 | 3×3 正交矩阵 | 9 个 | 组合方便、无歧义 | 参数多、需满足正交约束 |
| 欧拉角 | 绕三轴旋转的角度 | 3 个 | 直观、参数少 | 万向节死锁、序列依赖 |
| 轴角 | 绕单位轴旋转的角度 | 4 个 | 几何意义清晰 | 旋转组合不便 |
| 四元数 | 超复数表示 | 4 个 | 无死锁、插值友好 | 直观性差 |

## 依赖

| 库 | 安装命令 | 用途 |
|----|---------|------|
| Eigen3 | `sudo apt install libeigen3-dev` | 线性代数运算 |
| CMake (≥3.22) | `sudo apt install cmake` | 构建系统 |

需要 **C++17** 或更高版本。

## 目录结构

```
Transformation/
├── README.md
└── transformation/
    ├── CMakeLists.txt          # CMake 构建配置
    ├── include/
    │   └── transformation.h    # 库头文件（所有 API）
    └── src/
        └── main.cpp            # 测试程序（往返验证）
```

## 构建与运行

```bash
cd transformation
mkdir -p build && cd build
cmake ..
make -j$(nproc)
./bin/transformation
```

期望输出：12 项往返测试全部通过（`[PASS]`），最终打印 `结果: 12/12 通过`，退出码为 0。

## API

### 转换函数矩阵

| 从 → 到 | 旋转矩阵 | 欧拉角 | 轴角 | 四元数 |
|---------|:-------:|:-----:|:----:|:-----:|
| 旋转矩阵 | — | `rotationMatrixToEulerAngle(R, order)` | `rotationMatrixToAxisAngle(R)` | `rotationMatrixToQuaternion(R)` |
| 欧拉角 | `eulerAngleToRotationMatrix(euler, order)` | — | `eulerAngleToAxisAngle(euler, order)` | `eulerAngleToQuaternion(euler, order)` |
| 轴角 | `axisAngleToRotationMatrix(aa)` | `axisAngleToEulerAngle(aa, order)` | — | `axisAngleToQuaternion(aa)` |
| 四元数 | `quaternionToRotationMatrix(q)` | `quaternionToEulerAngle(q, order)` | `quaternionToAxisAngle(q)` | — |

`order` 参数支持 6 种旋转序列（大小写均可）：`"xyz"`, `"xzy"`, `"yxz"`, `"yzx"`, `"zxy"`, `"zyx"`。

### 辅助函数

| 函数 | 说明 |
|------|------|
| `parseOrder(order)` | 将旋转序列字符串转换为 Eigen 所需的索引顺序 |
| `roundNearZero(mat, threshold)` | 将矩阵/向量中绝对值小于阈值的元素置零 |
| `handleException(e)` | 统一异常处理，输出错误信息到 stderr |
| `rotationMatrixIsUnitary(R)` | 检查旋转矩阵是否为正交阵 |

## 使用示例

```cpp
#include <Eigen/Dense>
#include "transformation.h"

int main() 
{
    // 定义旋转：绕 X 轴旋转 45°
    Eigen::Vector3d euler(M_PI / 4, 0, 0);
    std::string order = "xyz";

    // 欧拉角 → 旋转矩阵
    Eigen::Matrix3d R = eulerAngleToRotationMatrix(euler, order);
    std::cout << "Rotation Matrix:\n" << R << std::endl;

    // 旋转矩阵 → 欧拉角
    Eigen::Vector3d euler_back = rotationMatrixToEulerAngle(R, order);
    std::cout << "Euler angles: " << euler_back.transpose() << std::endl;

    // 旋转矩阵 → 轴角
    Eigen::AngleAxisd aa = rotationMatrixToAxisAngle(R);
    std::cout << "Angle: " << aa.angle()
              << " Axis: " << aa.axis().transpose() << std::endl;

    return 0;
}
```

## 已知限制

- 处理 `roundNearZero` 使用了双层 for 循环遍历元素，大数据量时可用 Eigen 的 `unaryExpr` 配合 SIMD 优化
- 异常处理仅打印到 stderr 并返回零值，调用方无法区分成功与失败
- 欧拉角转换存在万向节死锁的固有问题（取决于旋转序列和角度）
