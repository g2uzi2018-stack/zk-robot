# TIAGo CAN 配置

此目录包含 TIAGo 机器人的 CAN 配置文件。

每个 YAML 文件描述一条物理 CAN 总线，包含以下内容：

- SocketCAN 接口名称
- 关节名称
- CAN 节点 ID
- 关节单位
- 关节限位
- 编码器参数

超时、重试、状态转换和通信策略等软件运行时行为不在此处配置，
这些行为由 CAN 驱动程序的实现定义。

## CAN 总线布局

| 文件 | 接口 | 关节 |
|---|---|---|
| `left_shoulder.yaml` | `vcan0` | 左臂关节 1-2 |
| `left_elbow.yaml` | `vcan1` | 左臂关节 3-4 |
| `left_wrist.yaml` | `vcan2` | 左臂关节 5-7 |
| `right_shoulder.yaml` | `vcan3` | 右臂关节 1-2 |
| `right_elbow.yaml` | `vcan4` | 右臂关节 3-4 |
| `right_wrist.yaml` | `vcan5` | 右臂关节 5-7 |
| `left_gripper.yaml` | `vcan6` | 左夹爪关节 |
| `right_gripper.yaml` | `vcan7` | 右夹爪关节 |

## 配置结构

示例：

```yaml
schema_version: 1

interface: vcan0

joints:
  - name: arm_left_1_joint
    node_id: 1
    unit: radian

    limits:
      min_position: -1.0
      max_position: 1.0
      max_velocity: 0.2

    encoder:
      counts_per_motor_revolution: 4096
      gear_ratio: 1.0
      direction: 1
      zero_offset: 0.0
