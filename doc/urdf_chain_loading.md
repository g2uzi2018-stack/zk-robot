# URDF 运动链加载：第一步

本次新增的是 `loadUrdfChain()`，不是已经完成的正解模型。它把本地文件转换成
后续 FK / Jacobian 可用的数据，不连接 CAN，不发目标位置，不实现 IK、MoveL、
时间参数化、碰撞或动力学计算。

## 入口函数的上下游

文件：`src/motion/kinematics/urdf_chain.hpp` / `.cpp`。

```cpp
#include "motion/kinematics/urdf_chain.hpp"

// 在程序初始化阶段调用。右臂换成 R_WRIST_R_S；不硬编码几何尺寸。
const auto chain = robot::motion::loadUrdfChain(
    "T170C-LX-AGX-URDF-1.0/urdf/T170C-V1.1-URDF-1.0_mujoco.urdf",
    "WAIST_Y_S", "L_WRIST_R_S", 7);
```

上游是初始化代码：提供文件路径、基准连杆、末端连杆、预期活动关节数。
这里的 base/tip 是机械结构的边界，不是 CartesianLinePath 的运动起点和终点。

内部先读文件并解析 XML，验证连杆树，再从 tip 沿父关节回溯到 base，反转结果，
最后为活动关节分配 q 下标并检查数量。XML 文档会释放，返回的数据不依赖它的生命周期。

返回的 `UrdfChain` 包含：

- `base_link` / `tip_link`：此链采用的参考坐标系和末端坐标系。
- `joints`：base 到 tip 的所有关节，包括 fixed；每项保存名称、父子连杆、类型、
  joint/origin、局部单位轴和可选 q_index。
- `joint_names`：仅活动关节，顺序就是以后传入 q 数组的顺序。

例如两活动轴和一个固定工具偏移的链，`joints.size()` 是 3，`joint_names.size()`
是 2；fixed 的 q_index 为空。固定工具偏移不能因为不计入自由度而丢掉。

下游是之后才要实现的具体 `KinematicModel<N>`：初始化时保存这份数据，运行时
`forwardKinematics(q)` 和 `jacobian(q)` 只使用缓存数据、不读文件。
此前讨论的 `UrdfKinematicModel<7>::fromUrdf(...)` 可以将本函数作为内部加载步骤；
本次没有创建一个缺少正解实现的假模型。`UrdfChain` 本身没有 FK / IK 方法。

## 坐标与兼容性边界

`origin` 来自 joint/origin，不是网格或惯性原点；URDF RPY 按
`Rz(yaw) * Ry(pitch) * Rx(roll)` 构造旋转。axis 表达在关节局部坐标系。
缺失 origin 使用单位变换，缺失活动关节 axis 使用 URDF 默认 X 轴；显式空值、
非有限数值和零轴拒绝，合法非单位轴归一化。位置/移动关节单位 m，旋转单位 rad。

这是用 libxml2 解析 XML 的**限定运动学子集加载器**，不是完整 URDF 标准验证器。
支持树状模型里 base 为 tip 祖先的串联链；选中的链支持 fixed、revolute、continuous、
prismatic，拒绝 mimic、planar、floating 和未知类型。未选中的支链不参与几何计算。

全模型拓扑必须是单根、连通、无环的树。重复名称、多个父关节、悬空引用、找不到
连杆或自由度不匹配都会抛 runtime_error，不返回半成品。只读取本地 UTF-8 / ASCII
文件，大小上限 8 MiB；不扩展 xacro，不处理 DTD/自定义实体/命名空间元素，不联网。

不加载网格、不验证 visual/inertial/limit 的完整语义、不使用 URDF limit 作为运行
安全限制。加载成功只表示此运动链的数据可提取，不表示机械臂可安全执行运动。
传入 FK 的 q 必须已经由上游按 joint_names 排序并转换成模型关节坐标；不能直接
拿电机 counts、另一台机械臂的电机顺序或未处理的方向/零偏来用。

## 构建与测试

XML 依赖只属于可选 `robot_urdf_chain` 静态库，不改变现有 `robot_motion` 头文件库
的依赖。默认不编译新增模块；需要 C++17、Eigen 和 libxml2 开发文件。
Debian/Ubuntu 的新增依赖包是 `libxml2-dev`（原项目依赖仍需要已安装）。

```bash
cmake -S . -B build -DBUILD_TESTING=ON -DBUILD_URDF_CHAIN_LOADER=ON
cmake --build build --target motion_urdf_chain_test -j2
ctest --test-dir build -R '^motion_urdf_chain' --output-on-failure
```

`motion_urdf_chain` 使用临时 XML，覆盖关节顺序、固定工具变换、完整 RPY、各受支持
类型、默认值和多种错误输入；`motion_urdf_chain_t170c` 额外读取仓库的实际 URDF，
验证左右臂各七个活动关节的顺序。两项均不打开机器人设备。

下一步再实现单个关节在 q 下的变换，随后沿 joints 串起来实现 FK。本步骤不修改
Joint、Motor、Controller 或 Executor 的运行行为。
