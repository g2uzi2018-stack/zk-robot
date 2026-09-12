#include "ti5/arm/arm.hpp"
#include "ti5/can/can_discovery.hpp"
#include "ti5/config/config_loader.hpp"
#include "ti5/controller/arm_controller.hpp"
#include "ti5/controller/head_controller.hpp"
#include "ti5/controller/waist_controller.hpp"
#include "ti5/head/head.hpp"
#include "ti5/waist/waist.hpp"
#include "ti5/joint/joint_config_builder.hpp"
#include "can/can_interface_manager.hpp"
#include "ti5/controller/hand_controller.hpp"
#include <cstdlib>
#include <cstdint>
#include <type_traits>

#include <array>
#include <algorithm>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <poll.h>
#include <stdexcept>
#include <chrono>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

#ifndef TI5_SOURCE_DIR
#define TI5_SOURCE_DIR "."
#endif

namespace
{
using namespace robot::ti5;
constexpr double kStepRad = 0.05;
int hand_step_raw = 50;
std::uint16_t hand_initial_raw = 30000;
std::uint8_t hand_speed_raw = 10;
bool commission_hands = false;
constexpr int kPollMilliseconds = 10;
constexpr auto kControlPeriod = std::chrono::milliseconds(10);
constexpr auto kDisplayPeriod = std::chrono::milliseconds(100);

class RawTerminal final
{
public:
    RawTerminal()
    {
        if (::tcgetattr(STDIN_FILENO, &old_) == 0)
        {
            auto raw = old_;
            raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
            raw.c_cc[VMIN] = 0;
            raw.c_cc[VTIME] = 1;
            ::tcsetattr(STDIN_FILENO, TCSANOW, &raw);
            active_ = true;
        }
    }

    ~RawTerminal()
    {
        if (active_) ::tcsetattr(STDIN_FILENO, TCSANOW, &old_);
    }

    RawTerminal(const RawTerminal &) = delete;
    RawTerminal &operator=(const RawTerminal &) = delete;

private:
    termios old_{};
    bool active_{false};
};

struct JointDirection
{
    const char *up;
    const char *down;
};

constexpr std::array<JointDirection, 3> kHeadDirections{{
    {"机器人左方", "机器人右方"},
    {"机器人后方", "机器人前方"},
    {"机器人右方", "机器人左方"},
}};

// 腰部的各轴正负方向尚未完成逐轴实机标定；CLI 只显示坐标增减，
// 不把未验证的现场方向写成确定结论。
constexpr std::array<JointDirection, 5> kWaistDirections{{
    {"增加 yaw 角", "减少 yaw 角"},
    {"增加 fold_p3 角", "减少 fold_p3 角"},
    {"增加 fold_p2 角", "减少 fold_p2 角"},
    {"增加 fold_p1 角", "减少 fold_p1 角"},
    {"增加 fold_r 角", "减少 fold_r 角"},
}};

constexpr std::array<JointDirection, 7> kLeftArmDirections{{
    {"机器人后方", "机器人前方"},
    {"机器人上方", "机器人下方"},
    {"机器人身体内侧", "机器人身体外侧"},
    {"机器人后方", "机器人前方"},
    {"机器人身体内侧", "机器人身体外侧"},
    {"机器人后方", "机器人前方"},
    {"机器人身体外侧", "机器人身体内侧"},
}};

constexpr std::array<JointDirection, 7> kRightArmDirections{{
    {"机器人前方", "机器人后方"},
    {"机器人上方", "机器人下方"},
    {"机器人身体外侧", "机器人身体内侧"},
    {"机器人前方", "机器人后方"},
    {"机器人身体外侧", "机器人身体内侧"},
    {"机器人前方", "机器人后方"},
    {"机器人身体内侧", "机器人身体外侧"},
}};

const std::string &interfaceFor(const DiscoveryResult &result,
                                const std::string &bus_name)
{
    for (const auto &bus : result.logical_buses)
    {
        if (bus.bus_name == bus_name && bus.complete && bus.interface_name)
            return *bus.interface_name;
    }
    throw std::runtime_error("未发现完整 CAN 总线：" + bus_name);
}

const LogicalCanBus &logicalBusFor(const Ti5RobotConfig &config,
                                   const std::string &bus_name)
{
    for (const auto &bus : config.can_buses)
    {
        if (bus.name == bus_name) return bus;
    }
    throw std::runtime_error("缺少逻辑 CAN 总线配置：" + bus_name);
}

CanBusOptions makeBusOptions(const LogicalCanBus &bus,
                             const CanConfig &config)
{
    return CanBusOptions{
        bus.expected_node_ids,
        config.receive.use_can_filters,
        config.receive.receive_error_frames,
        config.control.send_failure_threshold};
}

void prepareHeadForControl(Head &head)
{
    try
    {
        head.prepare();
    }
    catch (const std::exception &error)
    {
        if (std::string(error.what()).find("mixed mode") == std::string::npos)
            throw;
        head.stop();
        head.prepare();
    }
}

void prepareArmForControl(Arm &arm)
{
    try
    {
        arm.prepare();
    }
    catch (const std::exception &error)
    {
        if (std::string(error.what()).find("mixed mode") == std::string::npos)
            throw;
        arm.stop();
        arm.prepare();
    }
}

void startWaistForControl(Waist &waist, WaistController &controller)
{
    if (controller.state() == WaistController::ControlState::Failed)
        controller.reset();
    if (controller.state() == WaistController::ControlState::Running)
        return;

    // 腰部没有 STOP 恢复路径。若 Waist 仍处于已建立的 mode=8，
    // 只恢复 Controller；其他状态则重新执行只读准备和当前位置启动。
    if (waist.controlState() != WaistControlState::PositionControlActive)
    {
        waist.prepare();
        waist.startPositionControlAtCurrentPosition();
    }
    controller.start();
}

void drawHead(const HeadController &controller, std::size_t selected)
{
    static constexpr const char *names[] = {
        "neck_yaw", "neck_pitch", "neck_roll"};
    const auto state = controller.currentState();
    const auto target = controller.targetPositions();
    std::cout << "\033[2J\033[HTI5 Head Control\n\n"
              << "1. neck_yaw\n2. neck_pitch\n3. neck_roll\n\n"
              << "当前关节: " << names[selected]
              << "\n目标角度: " << std::fixed << std::setprecision(4)
              << target[selected] << " rad\n实际角度: ";
    if (state && state->joints[selected].position_rad)
        std::cout << *state->joints[selected].position_rad << " rad\n";
    else
        std::cout << "等待反馈\n";
    std::cout << "\n↑：" << kHeadDirections[selected].up
              << "\n↓：" << kHeadDirections[selected].down
              << "\n↑/↓ 调整 0.01 rad，q 返回关节选择\n" << std::flush;
}

template <std::size_t N>
void drawArm(const ArmController &controller,
             std::size_t selected,
             const std::array<std::string, N> &names,
             const std::array<JointDirection, N> &directions,
             const char *title)
{
    const auto state = controller.currentState();
    const auto target = controller.targetPositions();
    std::cout << "\033[2J\033[H" << title << " Control\n\n";
    for (std::size_t index = 0; index < N; ++index)
        std::cout << (index == selected ? "> " : "  ") << index + 1
                  << ". " << names[index] << "\n";
    std::cout << "\n当前关节: " << names[selected]
              << "\n目标角度: " << std::fixed << std::setprecision(4)
              << target[selected] << " rad\n实际角度: ";
    if (state && state->joints[selected].position_rad)
        std::cout << *state->joints[selected].position_rad << " rad\n";
    else
        std::cout << "等待反馈\n";
    std::cout << "\n↑：" << directions[selected].up
              << "\n↓：" << directions[selected].down
              << "\n数字键选择关节，↑/↓ 调整 0.01 rad，q 返回关节选择并保持当前目标\n"
              << std::flush;
}

template <std::size_t N>
void drawWaist(const WaistController &controller,
               std::size_t selected,
               const std::array<std::string, N> &names,
               const std::array<JointDirection, N> &directions)
{
    const auto state = controller.currentState();
    const auto target = controller.targetPositions();
    std::cout << "\033[2J\033[HTI5 Waist Control\n\n";
    for (std::size_t index = 0; index < N; ++index)
        std::cout << (index == selected ? "> " : "  ") << index + 1
                  << ". " << names[index] << "\n";
    std::cout << "\n当前关节: " << names[selected]
              << "\n目标角度: " << std::fixed << std::setprecision(4)
              << target[selected] << " rad\n实际角度: ";
    if (state && state->joints[selected].position_rad)
        std::cout << *state->joints[selected].position_rad << " rad\n";
    else
        std::cout << "等待反馈\n";
    std::cout << "\n↑：" << directions[selected].up
              << "\n↓：" << directions[selected].down
              << "\n数字键选择关节，↑/↓ 调整 0.05 rad，q 返回关节选择并保持 mode 8\n"
              << std::flush;
}

template <typename Values>
void adjustTarget(Values &target, std::size_t selected, int direction)
{
    if constexpr (std::is_integral_v<typename Values::value_type>)
        target[selected] = static_cast<std::uint16_t>(std::clamp(
            static_cast<int>(target[selected]) + direction * hand_step_raw, 0, 65535));
    else
        target[selected] += direction * kStepRad;
}

template <typename Values>
bool applyArrow(unsigned char key, std::size_t selected, Values &target)
{
    unsigned char sequence[2]{};
    if (::read(STDIN_FILENO, sequence, sizeof(sequence)) !=
        static_cast<ssize_t>(sizeof(sequence)))
        return false;
    if (key != 27 || sequence[0] != '[') return false;
    if (sequence[1] == 'A') adjustTarget(target, selected, 1);
    else if (sequence[1] == 'B') adjustTarget(target, selected, -1);
    else return false;
    return true;
}

template <typename Controller, typename Values, typename Tick, typename Draw>
void controlLoop(Controller &controller,
                 Values initial_target,
                 std::size_t joint_count,
                 std::size_t initial_selected,
                 Tick tick,
                 Draw draw)
{
    auto target = initial_target;
    std::size_t selected = initial_selected;
    bool back = false;
    auto next_display = std::chrono::steady_clock::now();
    auto next_control = std::chrono::steady_clock::now();
    while (!back)
    {
        const auto now = std::chrono::steady_clock::now();
        if (now >= next_control)
        {
            tick();
            next_control += kControlPeriod;
            if (next_control <= std::chrono::steady_clock::now())
                next_control = std::chrono::steady_clock::now() + kControlPeriod;
        }
        if (now >= next_display)
        {
            draw(controller, selected);
            next_display = now + kDisplayPeriod;
        }
        pollfd descriptor{STDIN_FILENO, POLLIN, 0};
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
            next_control - std::chrono::steady_clock::now());
        const int timeout = std::max(
            1, std::min(kPollMilliseconds,
                        static_cast<int>(remaining.count()) + 1));
        if (::poll(&descriptor, 1, timeout) <= 0) continue;
        unsigned char key = 0;
        if (::read(STDIN_FILENO, &key, 1) != 1) continue;
        if (key == 'q' || key == 'Q') back = true;
        else if (key >= '1' && key <= static_cast<unsigned char>('0' + joint_count))
            selected = static_cast<std::size_t>(key - '1');
        else if (key == 27 && applyArrow(key, selected, target))
            controller.setTarget(target);
        else if (key == '+' || key == '-')
        {
            adjustTarget(target, selected, key == '+' ? 1 : -1);
            controller.setTarget(target);
        }
    }
}

void updateRunningArm(ArmController &controller)
{
    if (controller.state() == ArmController::ControlState::Running)
        controller.update();
}

void updateRunningWaist(WaistController &controller)
{
    if (controller.state() == WaistController::ControlState::Running)
        controller.update();
}

template <typename Tick>
unsigned char waitForMenuKey(Tick tick)
{
    auto next_control = std::chrono::steady_clock::now();
    for (;;)
    {
        const auto now = std::chrono::steady_clock::now();
        if (now >= next_control)
        {
            tick();
            next_control += kControlPeriod;
            if (next_control <= std::chrono::steady_clock::now())
                next_control = std::chrono::steady_clock::now() + kControlPeriod;
        }
        pollfd descriptor{STDIN_FILENO, POLLIN, 0};
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
            next_control - std::chrono::steady_clock::now());
        const int timeout = std::max(
            1, std::min(kPollMilliseconds,
                        static_cast<int>(remaining.count()) + 1));
        if (::poll(&descriptor, 1, timeout) <= 0) continue;
        unsigned char key = 0;
        if (::read(STDIN_FILENO, &key, 1) == 1) return key;
    }
}

template <std::size_t N>
void jointMenu(const char *title,
               const std::array<std::string, N> &names)
{
    std::cout << "\033[2J\033[H" << title << " 关节选择\n\n";
    for (std::size_t index = 0; index < N; ++index)
        std::cout << index + 1 << ". " << names[index] << "\n";
    std::cout << "\n按数字进入控制，q 返回部件菜单\n" << std::flush;
}

class HandSession
{
public:
    HandSession(HandSide side, const hand::HandConfig &config)
    {
        auto selected = side == HandSide::Left ? config.left : config.right;
        if (!selected.protocol_verified || !selected.control_enabled)
        {
            const char *confirmation = std::getenv("ZK_ROBOT_CONFIRM_UNVERIFIED_HAND_TEST");
            if (!commission_hands || !confirmation || std::string(confirmation) != "YES")
                throw std::runtime_error("手部控制未开放；调试需 --commission 并设置 ZK_ROBOT_CONFIRM_UNVERIFIED_HAND_TEST=YES");
            selected.protocol_verified = true;
            selected.control_enabled = true;
        }
        if (selected.interface_name.empty())
            throw std::runtime_error("hands.yaml 未指定手部 CAN 接口");
        robot::can::CanInterfaceManager manager;
        const auto &transport = config.transport;
        const robot::can::CanInterfaceSettings settings{
            transport.bitrate, transport.restart_ms, transport.reconfigure_wait,
            transport.startup_wait, transport.validate_bitrate};
        const auto ready = transport.manage_linux_link
            ? manager.prepare(selected.interface_name, settings)
            : manager.inspect(selected.interface_name);
        if (!ready.up || (transport.validate_bitrate &&
            (!ready.bitrate || *ready.bitrate != transport.bitrate)))
            throw std::runtime_error(selected.interface_name + " 未启动或波特率不符");
        hand_ = std::make_unique<Hand>(side, selected, selected.interface_name,
                                      config.discovery.response_timeout);
        controller_ = std::make_unique<HandController>(*hand_);
        speeds_.fill(hand_speed_raw);
    }

    ~HandSession() { pause(); }
    void start()
    {
        if (controller_->state() == HandController::ControlState::Failed)
            controller_->reset();
        controller_->start(speeds_);
        if (controller_->currentState())
            saved_target_ = controller_->targetPositions();
        else if (!initialized_)
            saved_target_.fill(hand_initial_raw);
        controller_->setTarget(saved_target_, speeds_);
        initialized_ = true;
    }
    void pause()
    {
        if (controller_ && controller_->state() == HandController::ControlState::Running)
            controller_->pause();
    }
    void setTarget(const Hand::PositionValues &values)
    {
        controller_->setTarget(values, speeds_);
        saved_target_ = values;
    }
    void update() { controller_->update(); }
    const auto &targetPositions() const { return controller_->targetPositions(); }
    const auto &currentState() const { return controller_->currentState(); }

private:
    std::unique_ptr<Hand> hand_;
    std::unique_ptr<HandController> controller_;
    Hand::PositionValues saved_target_{};
    Hand::SpeedValues speeds_{};
    bool initialized_{false};
};

const std::array<std::string, 6> hand_names{{
    "channel_0", "channel_1", "channel_2", "channel_3", "channel_4", "channel_5"}};

void drawHand(const HandSession &session, std::size_t selected, const char *title)
{
    std::cout << "\033[2J\033[H" << title << " Control\n\n";
    for (std::size_t i = 0; i < hand_names.size(); ++i)
        std::cout << (i == selected ? "> " : "  ") << i + 1 << ". " << hand_names[i] << "\n";
    std::cout << "\n当前关节: " << hand_names[selected]
              << "\n目标位置: " << session.targetPositions()[selected] << " raw\n实际位置: ";
    if (session.currentState())
        std::cout << session.currentState()->positions_raw[selected] << " raw\n";
    else
        std::cout << "无反馈（首次使用 initial-raw）\n";
    std::cout << "\n↑ 增加 / ↓ 减少，每步 " << hand_step_raw
              << " raw，速度 " << static_cast<int>(hand_speed_raw)
              << "\n数字键选择关节，+/- 也可调整，q 返回关节选择并暂停刷新\n" << std::flush;
}

int parseRaw(const std::string &value, const std::string &name, int minimum, int maximum)
{
    std::size_t used = 0;
    const int result = std::stoi(value, &used);
    if (used != value.size() || result < minimum || result > maximum)
        throw std::invalid_argument(name + " 数值超出范围");
    return result;
}

} // namespace

int main(int argc, char **argv)
{
    try
    {
        bool dry_run = false;
        std::vector<std::string> candidates;
        for (int i = 1; i < argc; ++i)
        {
            const std::string option = argv[i];
            if (option == "--help" || option == "-h")
            {
                std::cout << "用法: ti5_joint_cli [can0 can1 ...] [--commission] [--dry-run]\n"
                          << "  --hand-step-raw 1..200 (默认 50；兼容 --delta-raw)\n"
                          << "  --initial-raw 0..65535 (无反馈初值，默认 30000)\n"
                          << "  --speed-raw 1..20 (默认 5)\n"
                          << "主菜单: 1 头部 / 2 左臂 / 3 右臂 / 4 左手 / 5 右手 / 6 腰部\n"
                          << "数字键选关节，↑/↓ 或 +/- 调整，q 逐级返回/退出。\n"
                          << "头部/双臂/腰部每步 0.05 rad；手部使用 raw 单位。\n"
                          << "--dry-run 仅检查配置，不打开 CAN。\n"
                          << "未开放手部配置时，--commission 还需 ZK_ROBOT_CONFIRM_UNVERIFIED_HAND_TEST=YES。\n";
                return 0;
            }
            if (option == "--dry-run") { dry_run = true; continue; }
            if (option == "--commission") { commission_hands = true; continue; }
            if (option == "--hand-step-raw" || option == "--delta-raw" ||
                option == "--initial-raw" || option == "--speed-raw")
            {
                if (++i >= argc) throw std::invalid_argument(option + " 缺少数值");
                if (option == "--initial-raw") hand_initial_raw = parseRaw(argv[i], option, 0, 65535);
                else if (option == "--speed-raw") hand_speed_raw = parseRaw(argv[i], option, 1, 20);
                else hand_step_raw = parseRaw(argv[i], option, 1, 200);
                continue;
            }
            if (option.rfind("--", 0) == 0) throw std::invalid_argument("未知参数: " + option);
            candidates.push_back(option);
        }
        const auto root = std::filesystem::path(TI5_SOURCE_DIR) /
                          "config/ti5/t170c";
        const auto robot_config = robot::ti5::loadRobotConfig(root / "robot.yaml");
        const auto can_config = robot::ti5::loadCanConfig(root / "can.yaml");
        const auto safety_config = robot::ti5::loadJointSafetyConfig(root / "safety.yaml");
        if (dry_run)
        {
            const auto config = hand::loadHandConfig(root / "hands.yaml");
            std::cout << "配置检查通过；左手=" << config.left.interface_name
                      << "，右手=" << config.right.interface_name
                      << "，手部步长=" << hand_step_raw << " raw；未打开 CAN\n";
            return 0;
        }
        std::vector<LogicalCanBus> requested;
        for (const auto &bus : robot_config.can_buses)
            if (bus.name == "waist_fold" || bus.name == "head" ||
                bus.name == "left_arm" || bus.name == "right_arm")
                requested.push_back(bus);
        if (requested.size() != 4)
            throw std::runtime_error(
                "robot.yaml 缺少 waist_fold/head/left_arm/right_arm 总线");
        if (candidates.empty()) candidates = {"can0", "can1", "can2", "can3"};
        const auto discovery = CanDiscovery{}.discover(
            requested, makeDiscoveryOptions(can_config), candidates);
        if (!discovery.success)
            throw std::runtime_error("腰部、头部和双臂 CAN 总线发现失败");
        const auto configs = makeJointConfigs(robot_config, safety_config);
        const auto &waist_bus_config = logicalBusFor(robot_config, "waist_fold");
        const auto &head_bus_config = logicalBusFor(robot_config, "head");
        const auto &left_bus_config = logicalBusFor(robot_config, "left_arm");
        const auto &right_bus_config = logicalBusFor(robot_config, "right_arm");
        auto waist_bus = std::make_unique<CanBus>(
            interfaceFor(discovery, "waist_fold"),
            makeBusOptions(waist_bus_config, can_config));
        auto head_bus = std::make_unique<CanBus>(
            interfaceFor(discovery, "head"),
            makeBusOptions(head_bus_config, can_config));
        auto left_bus = std::make_unique<CanBus>(
            interfaceFor(discovery, "left_arm"),
            makeBusOptions(left_bus_config, can_config));
        auto right_bus = std::make_unique<CanBus>(
            interfaceFor(discovery, "right_arm"),
            makeBusOptions(right_bus_config, can_config));
        Waist waist(std::move(waist_bus), configs);
        Head head(std::move(head_bus), configs);
        Arm left_arm(ArmSide::Left, std::move(left_bus), configs);
        Arm right_arm(ArmSide::Right, std::move(right_bus), configs);
        WaistController waist_controller(waist);
        HeadController head_controller(head);
        ArmController left_controller(left_arm);
        ArmController right_controller(right_arm);
        std::unique_ptr<HandSession> left_hand, right_hand;
        RawTerminal terminal;
        auto pumpControllers = [&left_controller, &right_controller,
                                &waist_controller]()
        {
            updateRunningArm(left_controller);
            updateRunningArm(right_controller);
            updateRunningWaist(waist_controller);
        };

        const std::array<std::string, 3> head_names{{"neck_yaw", "neck_pitch", "neck_roll"}};
        const std::array<std::string, 7> left_names{{"left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw", "left_elbow_yaw", "left_wrist_pitch", "left_wrist_yaw", "left_wrist_roll"}};
        const std::array<std::string, 7> right_names{{"right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw", "right_elbow_yaw", "right_wrist_pitch", "right_wrist_yaw", "right_wrist_roll"}};
        const std::array<std::string, 5> waist_names{{"waist_yaw", "fold_p3", "fold_p2", "fold_p1", "fold_r"}};
        for (;;)
        {
            std::cout << "\033[2J\033[HTI5 Joint CLI\n\n"
                      << "1. 头部\n2. 左臂\n3. 右臂\n4. 左手\n5. 右手\n6. 腰部\n\n"
                      << "按 1/2/3/4/5/6 选择部件，q 退出程序\n" << std::flush;
            const unsigned char key = waitForMenuKey(pumpControllers);
            if (key == 'q' || key == 'Q') break;
            if (key < '1' || key > '6') continue;
            if (key == '4' || key == '5')
            {
                auto &session = key == '4' ? left_hand : right_hand;
                const char *title = key == '4' ? "左手" : "右手";
                try
                {
                    for (;;)
                    {
                        jointMenu(title, hand_names);
                        const auto selected_key = waitForMenuKey(pumpControllers);
                        if (selected_key == 'q' || selected_key == 'Q') break;
                        if (selected_key < '1' || selected_key > '6') continue;
                        if (!session)
                            session = std::make_unique<HandSession>(
                                key == '4' ? HandSide::Left : HandSide::Right,
                                hand::loadHandConfig(root / "hands.yaml"));
                        session->start();
                        auto next_hand = std::chrono::steady_clock::now();
                        controlLoop(*session, session->targetPositions(), 6, selected_key - '1',
                            [&]() {
                                pumpControllers();
                                const auto now = std::chrono::steady_clock::now();
                                if (now >= next_hand) {
                                    session->update();
                                    next_hand = std::chrono::steady_clock::now() + std::chrono::milliseconds(50);
                                    pumpControllers();
                                }
                            },
                            [title](const auto &s, std::size_t n) { drawHand(s, n, title); });
                        session->pause();
                    }
                }
                catch (const std::exception &error)
                {
                    if (session) session->pause();
                    std::cout << "\n手部控制失败: " << error.what()
                              << "\n按任意键返回部件菜单\n" << std::flush;
                    waitForMenuKey(pumpControllers);
                }
                continue;
            }
            if (key == '1')
            {
                for (;;)
                {
                    jointMenu("头部", head_names);
                    const unsigned char key = waitForMenuKey(pumpControllers);
                    if (key == 'q' || key == 'Q') break;
                    if (key >= '1' && key <= '3')
                    {
                        prepareHeadForControl(head);
                        head.startPositionControlAtCurrentPosition();
                        head_controller.start();
                        controlLoop(head_controller, head_controller.targetPositions(), 3, key - '1',
                                    [&head_controller, &pumpControllers]()
                                    {
                                        head_controller.update();
                                        pumpControllers();
                                    },
                                    [](const auto &controller, std::size_t selected) { drawHead(controller, selected); });
                        head_controller.stopAndConfirm();
                    }
                }
            }
            else if (key == '2')
            {
                for (;;)
                {
                    jointMenu("左臂", left_names);
                    const unsigned char key = waitForMenuKey(pumpControllers);
                    if (key == 'q' || key == 'Q') break;
                    if (key >= '1' && key <= '7')
                    {
                        if (left_controller.state() == ArmController::ControlState::Failed)
                            left_controller.reset();
                        if (left_controller.state() != ArmController::ControlState::Running)
                        {
                            prepareArmForControl(left_arm);
                            left_arm.startPositionControlAtCurrentPosition();
                            left_controller.start();
                        }
                        controlLoop(left_controller, left_controller.targetPositions(), 7, key - '1',
                                    pumpControllers,
                                    [&left_names](const auto &controller, std::size_t selected) { drawArm(controller, selected, left_names, kLeftArmDirections, "Left Arm"); });
                    }
                }
            }
            else if (key == '3')
            {
                for (;;)
                {
                    jointMenu("右臂", right_names);
                    const unsigned char key = waitForMenuKey(pumpControllers);
                    if (key == 'q' || key == 'Q') break;
                    if (key >= '1' && key <= '7')
                    {
                        if (right_controller.state() == ArmController::ControlState::Failed)
                            right_controller.reset();
                        if (right_controller.state() != ArmController::ControlState::Running)
                        {
                            prepareArmForControl(right_arm);
                            right_arm.startPositionControlAtCurrentPosition();
                            right_controller.start();
                        }
                        controlLoop(right_controller, right_controller.targetPositions(), 7, key - '1',
                                    pumpControllers,
                                    [&right_names](const auto &controller, std::size_t selected) { drawArm(controller, selected, right_names, kRightArmDirections, "Right Arm"); });
                    }
                }
            }
            else
            {
                for (;;)
                {
                    jointMenu("腰部", waist_names);
                    const unsigned char key = waitForMenuKey(pumpControllers);
                    if (key == 'q' || key == 'Q') break;
                    if (key >= '1' && key <= '5')
                    {
                        startWaistForControl(waist, waist_controller);
                        controlLoop(
                            waist_controller,
                            waist_controller.targetPositions(),
                            5,
                            key - '1',
                            pumpControllers,
                            [&waist_names](const auto &controller,
                                           std::size_t selected)
                            {
                                drawWaist(controller, selected,
                                          waist_names, kWaistDirections);
                            });
                    }
                }
            }
        }

        // 头部退出时请求并确认 STOP；腰部不提供 STOP 路径，
        // 双臂和腰部保留最后一个位置目标，让 mode 8 继续承担软件保持。
        auto stopOnExit = [](auto &component, const char *name)
        {
            try
            {
                component.stop();
            }
            catch (const std::exception &error)
            {
                std::cerr << name << " STOP 确认失败: " << error.what() << "\n";
            }
        };
        stopOnExit(head, "头部");
        // 双臂退出时保留最后一个 0x44 目标，让驱动器继续在位置模式保持姿态。
        // 这里不能发送 0x02 STOP，否则 mode=0 后手臂可能因重力下垂。
        std::cout << "\033[2J\033[H已退出 TI5 Joint CLI\n";
    }
    catch (const std::exception &error)
    {
        std::cerr << "错误: " << error.what() << "\n";
        return 1;
    }
    return 0;
}
