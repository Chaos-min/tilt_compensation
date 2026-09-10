# TLS 激光雷达倾角补偿

使用BMN静态倾角传感器读数，将一站激光雷达点云旋转到水平坐标系。

算法把仪器的俯仰/横滚调平，测站原点不变、水平度盘方向（yaw）不变。重力看不出航向，所以两站之间该配准还是要配准。

## 坐标约定

- 雷达：右手系，`+Z` 向上。
- 传感器：右手系，安装时 `+Z` 向下。
- 安装映射：传感器 `+X` 对应雷达 `-X`，传感器 `+Y` 对应雷达 `+Y`。
- 抬高传感器某测量轴的正端时，该轴读数为正。

## 使用

```cpp
#include "tilt_compensation.h"

#include <vector>

const std::vector<tilt_compensation::Point3f> points{{1.0F, 2.0F, 3.0F}};
const tilt_compensation::TiltAngles reading{2.0103, -0.5132};
const tilt_compensation::TiltAngles zero_bias{0.1020, -0.0310};

std::vector<tilt_compensation::Point3f> in_place = points;
tilt_compensation::compensateInPlace(in_place, reading, zero_bias);

const std::vector<tilt_compensation::Point3f> corrected =
    tilt_compensation::compensate(points, reading, zero_bias);
```

一站数据应使用同一组 `reading`。原地接口节省一份点云内存；副本接口保留输入数据。

## 算法

先计算 `tx = reading.x - zero_bias.x`、`ty = reading.y - zero_bias.y`，将角度从度转换为弧度后，再重建雷达坐标中的世界向上方向：

```text
up = [-sin(tx), sin(ty), sqrt(1 - sin²(tx) - sin²(ty))]
```

算法使用最小旋转将 `up` 对齐雷达 `+Z`，因此只补偿倾斜，不改变偏航角。

## 异常与限制

- 角度单位为度；非法或不符合物理约束的角度会抛出 `std::invalid_argument`。
- 含 `NaN/Inf` 的点保持不变。
- 静态倾角传感器应在设备稳定后读取，振动和线性加速度会污染结果。
- 改变传感器安装方向时必须同步更新轴映射。



## 构建与测试

```powershell
cmake -S . -B build
cmake --build build --config Debug
ctest --test-dir build -C Debug --output-on-failure
```

