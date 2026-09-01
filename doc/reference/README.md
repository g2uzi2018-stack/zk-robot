# 外骨骼官方参考资料

本目录只保存用于审查和追溯的外部资料，不作为项目运行时 Python 模块导入。

- `remote_manipulator_data_reader.py`：用户提供的官方读取器原始快照。
- 它用于确认 legacy 帧长度、字段偏移、little-endian 编码和
  `signed_raw * (2π / 16384)` 弧度换算。
- 它没有定义人体关节名称、每通道方向、零位偏置或 3D 运动学，因此不能替代
  `doc/exoskeleton_development.md` 中的现场映射记录。

当前实时项目代码使用 `src/input/exoskeleton/` 和
`tools/exoskeleton_joint_monitor.py` 的独立解析实现，不依赖本目录文件。
