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
  body_motor_count: 2
can_buses:
  - name: first
    protocol: ti5_joint
    required: true
    expected_node_ids: [1, 2]
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
        expect(minimal_robot.can_buses.front().expected_node_ids.size() == 2,
               "minimal robot node IDs were not loaded");
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

        writeText(robot_path, replaceOnce(kMinimalRobotYaml, "expected_node_ids: [1, 2]", "expected_node_ids: [1, 1]"));
        expectThrowsContaining([&] { robot::ti5::loadRobotConfig(robot_path); },
                               "重复",
                               "duplicate node IDs in one logical bus must be rejected");

        writeText(robot_path, replaceOnce(kMinimalRobotYaml, "expected_node_ids: [1, 2]", "expected_node_ids: [0, 2]"));
        expectThrowsContaining([&] { robot::ti5::loadRobotConfig(robot_path); },
                               "1..2047",
                               "invalid node IDs must be rejected");

        writeText(robot_path, replaceOnce(kMinimalRobotYaml, "body_motor_count: 2", "body_motor_count: 3"));
        expectThrowsContaining([&] { robot::ti5::loadRobotConfig(robot_path); },
                               "不一致",
                               "body motor count mismatch must be rejected");

        const auto duplicate_bus_yaml = replaceOnce(
            kMinimalRobotYaml,
            "    expected_node_ids: [1, 2]",
            "    expected_node_ids: [1]\n  - name: first\n    protocol: ti5_joint\n    required: true\n    expected_node_ids: [2]");
        writeText(robot_path, duplicate_bus_yaml);
        expectThrowsContaining([&] { robot::ti5::loadRobotConfig(robot_path); },
                               "名称重复",
                               "duplicate logical bus names must be rejected");

        const auto cross_bus_duplicate_yaml = replaceOnce(
            kMinimalRobotYaml,
            "    expected_node_ids: [1, 2]",
            "    expected_node_ids: [1]\n  - name: second\n    protocol: ti5_joint\n    required: true\n    expected_node_ids: [1]");
        writeText(robot_path, cross_bus_duplicate_yaml);
        expect(robot::ti5::loadRobotConfig(robot_path).can_buses.size() == 2,
               "node IDs on separate logical buses should be independently scoped");

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
