#include "ti5/config/config_loader.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{
    void expect(const bool condition, const std::string &message)
    {
        if (!condition)
        {
            throw std::runtime_error(message);
        }
    }

    template <typename Callable>
    void expectThrowsContaining(Callable &&callable,
                                const std::string &expected_text,
                                const std::string &message)
    {
        try
        {
            callable();
        }
        catch (const std::exception &error)
        {
            expect(std::string(error.what()).find(expected_text) != std::string::npos,
                   message + ": actual error was '" + error.what() + "'");
            return;
        }
        throw std::runtime_error(message + ": no exception was thrown");
    }

    void writeText(const std::filesystem::path &path, const std::string &text)
    {
        std::ofstream output(path);
        if (!output)
        {
            throw std::runtime_error("cannot create test YAML file " + path.string());
        }
        output << text;
    }

    std::string replaceOnce(std::string text,
                            const std::string &from,
                            const std::string &to)
    {
        const auto position = text.find(from);
        if (position == std::string::npos)
        {
            throw std::runtime_error("test fixture does not contain '" + from + "'");
        }
        text.replace(position, from.size(), to);
        return text;
    }

    class TemporaryDirectory final
    {
    public:
        TemporaryDirectory()
        {
            const auto suffix = std::chrono::steady_clock::now().time_since_epoch().count();
            path_ = std::filesystem::temp_directory_path() /
                    ("zk_robot_ti5_config_test_" + std::to_string(suffix));
            std::filesystem::create_directories(path_);
        }

        ~TemporaryDirectory()
        {
            std::error_code error;
            std::filesystem::remove_all(path_, error);
        }

        const std::filesystem::path &path() const
        {
            return path_;
        }

    private:
        std::filesystem::path path_;
    };

    const std::string kMinimalRobotYaml = R"yaml(
schema_version: 1
robot:
  vendor: TI5
  model: T170C
  body_motor_count: 22
can_buses:
  - name: first
    protocol: ti5_joint
    required: true
    expected_node_ids: [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22]
encoder_defaults:
  type: dual
  position_reference: output
  counts_per_output_revolution: 262144
  gear_ratio: 101.0
joints:
  - name: joint_1
    physical_name: JOINT_1
    bus: first
    motor: {node_id: 1, unit: radian}
  - name: joint_2
    physical_name: JOINT_2
    bus: first
    motor: {node_id: 2, unit: radian}
  - name: joint_3
    physical_name: JOINT_3
    bus: first
    motor: {node_id: 3, unit: radian}
  - name: joint_4
    physical_name: JOINT_4
    bus: first
    motor: {node_id: 4, unit: radian}
  - name: joint_5
    physical_name: JOINT_5
    bus: first
    motor: {node_id: 5, unit: radian}
  - name: joint_6
    physical_name: JOINT_6
    bus: first
    motor: {node_id: 6, unit: radian}
  - name: joint_7
    physical_name: JOINT_7
    bus: first
    motor: {node_id: 7, unit: radian}
  - name: joint_8
    physical_name: JOINT_8
    bus: first
    motor: {node_id: 8, unit: radian}
  - name: joint_9
    physical_name: JOINT_9
    bus: first
    motor: {node_id: 9, unit: radian}
  - name: joint_10
    physical_name: JOINT_10
    bus: first
    motor: {node_id: 10, unit: radian}
  - name: joint_11
    physical_name: JOINT_11
    bus: first
    motor: {node_id: 11, unit: radian}
  - name: joint_12
    physical_name: JOINT_12
    bus: first
    motor: {node_id: 12, unit: radian}
  - name: joint_13
    physical_name: JOINT_13
    bus: first
    motor: {node_id: 13, unit: radian}
  - name: joint_14
    physical_name: JOINT_14
    bus: first
    motor: {node_id: 14, unit: radian}
  - name: joint_15
    physical_name: JOINT_15
    bus: first
    motor: {node_id: 15, unit: radian}
  - name: joint_16
    physical_name: JOINT_16
    bus: first
    motor: {node_id: 16, unit: radian}
  - name: joint_17
    physical_name: JOINT_17
    bus: first
    motor: {node_id: 17, unit: radian}
  - name: joint_18
    physical_name: JOINT_18
    bus: first
    motor: {node_id: 18, unit: radian}
  - name: joint_19
    physical_name: JOINT_19
    bus: first
    motor: {node_id: 19, unit: radian}
  - name: joint_20
    physical_name: JOINT_20
    bus: first
    motor: {node_id: 20, unit: radian}
  - name: joint_21
    physical_name: JOINT_21
    bus: first
    motor: {node_id: 21, unit: radian}
  - name: joint_22
    physical_name: JOINT_22
    bus: first
    motor: {node_id: 22, unit: radian}
)yaml";

    const std::string kMinimalCanYaml = R"yaml(
schema_version: 1
socketcan:
  bitrate: 1000000
  interface_regex: "^can[0-9]+$"
  require_interface_up: true
  validate_bitrate: true
  manage_linux_link: false
  restart_ms: 100
  reconfigure_wait_ms: 100
  startup_wait_ms: 100
  body_adapter:
    selector: sysfs_parent
    value: test-parent
    expected_channels: 1
discovery:
  enabled: true
  response_timeout_ms: 50
  confirmations_required: 3
  max_attempts: 5
  allow_partial_bus: false
  require_unique_bus_match: true
)yaml";
}

int main()
{
    try
    {
        const std::filesystem::path source_dir = TI5_SOURCE_DIR;
        const auto shipped_robot = robot::ti5::loadRobotConfig(source_dir / "config/ti5/t170c/robot.yaml");
        expect(shipped_robot.vendor == "TI5", "shipped robot vendor mismatch");
        expect(shipped_robot.model == "T170C", "shipped robot model mismatch");
        expect(shipped_robot.body_motor_count == 22, "shipped body motor count mismatch");
        expect(shipped_robot.can_buses.size() == 4, "shipped logical bus count mismatch");

        const auto shipped_can = robot::ti5::loadCanConfig(source_dir / "config/ti5/t170c/can.yaml");
        expect(shipped_can.socketcan.bitrate == 1000000, "shipped SocketCAN bitrate mismatch");
        expect(shipped_can.socketcan.manage_linux_link, "shipped SocketCAN link management mismatch");
        expect(shipped_can.socketcan.body_adapter.selector == "id_path",
               "shipped body adapter selector mismatch");
        expect(shipped_can.socketcan.body_adapter.expected_channels == 4,
               "shipped body adapter channel count mismatch");
        expect(shipped_can.discovery.enabled, "shipped Discovery enabled mismatch");
        expect(shipped_can.discovery.response_timeout == std::chrono::milliseconds{50},
               "shipped Discovery timeout mismatch");
        expect(shipped_can.discovery.confirmations_required == 3, "shipped confirmation count mismatch");
        expect(shipped_can.discovery.max_attempts == 5, "shipped attempt count mismatch");

        TemporaryDirectory temporary_directory;
        const auto robot_path = temporary_directory.path() / "robot.yaml";
        const auto can_path = temporary_directory.path() / "can.yaml";
        writeText(robot_path, kMinimalRobotYaml);
        writeText(can_path, kMinimalCanYaml);

        const auto minimal_robot = robot::ti5::loadRobotConfig(robot_path);
        expect(minimal_robot.can_buses.front().expected_node_ids.size() == 22,
               "minimal robot node IDs were not loaded");
        expect(minimal_robot.joints.size() == 22 &&
                   minimal_robot.encoder_defaults.counts_per_output_revolution == 262144 &&
                   minimal_robot.encoder_defaults.gear_ratio == 101.0,
               "physical joints and encoder defaults were not loaded");
        const auto minimal_can = robot::ti5::loadCanConfig(can_path);
        expect(minimal_can.socketcan.restart_ms == std::chrono::milliseconds{100},
               "minimal restart_ms was not loaded");

        writeText(robot_path, replaceOnce(kMinimalRobotYaml, "schema_version: 1", "schema_version: 2"));
        expectThrowsContaining([&] { robot::ti5::loadRobotConfig(robot_path); },
                               "schema_version",
                               "unsupported robot schema version must be rejected");

        writeText(robot_path, replaceOnce(kMinimalRobotYaml, "model: T170C", "model: T170B"));
        expectThrowsContaining([&] { robot::ti5::loadRobotConfig(robot_path); },
                               "T170C",
                               "non-T170C model must be rejected");

        writeText(robot_path, replaceOnce(kMinimalRobotYaml, "expected_node_ids: [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22]", "expected_node_ids: [1, 1]"));
        expectThrowsContaining([&] { robot::ti5::loadRobotConfig(robot_path); },
                               "重复",
                               "duplicate node IDs in one logical bus must be rejected");

        writeText(robot_path, replaceOnce(kMinimalRobotYaml, "expected_node_ids: [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22]", "expected_node_ids: [0, 2]"));
        expectThrowsContaining([&] { robot::ti5::loadRobotConfig(robot_path); },
                               "1..2047",
                               "invalid node IDs must be rejected");

        writeText(robot_path, replaceOnce(kMinimalRobotYaml, "body_motor_count: 22", "body_motor_count: 21"));
        expectThrowsContaining([&] { robot::ti5::loadRobotConfig(robot_path); },
                               "22",
                               "body motor count mismatch must be rejected");

        const auto duplicate_bus_yaml = replaceOnce(
            kMinimalRobotYaml,
            "    expected_node_ids: [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22]",
            "    expected_node_ids: [1]\n  - name: first\n    protocol: ti5_joint\n    required: true\n    expected_node_ids: [2]");
        writeText(robot_path, duplicate_bus_yaml);
        expectThrowsContaining([&] { robot::ti5::loadRobotConfig(robot_path); },
                               "名称重复",
                               "duplicate logical bus names must be rejected");


        const auto duplicate_node_yaml = replaceOnce(
            kMinimalRobotYaml,
            "motor: {node_id: 22, unit: radian}",
            "motor: {node_id: 2, unit: radian}");
        writeText(robot_path, duplicate_node_yaml);
        expectThrowsContaining([&] { robot::ti5::loadRobotConfig(robot_path); },
                               "ID 2",
                               "ID 2 must have one physical joint");

        writeText(robot_path, replaceOnce(kMinimalRobotYaml, "required: true", "required: [true]"));
        expectThrowsContaining([&] { robot::ti5::loadRobotConfig(robot_path); },
                               "类型错误",
                               "wrong YAML field type must be rejected");

        writeText(can_path, replaceOnce(kMinimalCanYaml, "schema_version: 1", "schema_version: 2"));
        expectThrowsContaining([&] { robot::ti5::loadCanConfig(can_path); },
                               "schema_version",
                               "unsupported CAN schema version must be rejected");

        writeText(can_path, replaceOnce(kMinimalCanYaml, "response_timeout_ms: 50", "response_timeout_ms: 0"));
        expectThrowsContaining([&] { robot::ti5::loadCanConfig(can_path); },
                               "正数",
                               "zero Discovery timeout must be rejected");

        writeText(can_path, replaceOnce(kMinimalCanYaml, "max_attempts: 5", "max_attempts: 2"));
        expectThrowsContaining([&] { robot::ti5::loadCanConfig(can_path); },
                               "confirmations_required",
                               "insufficient Discovery attempts must be rejected");

        writeText(can_path, replaceOnce(kMinimalCanYaml, "confirmations_required: 3", "confirmations_required: [3]"));
        expectThrowsContaining([&] { robot::ti5::loadCanConfig(can_path); },
                               "类型错误",
                               "wrong Discovery field type must be rejected");

        std::cout << "TI5 configuration loader tests passed\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "TI5 configuration loader test failed: " << error.what() << '\n';
        return 1;
    }
}
