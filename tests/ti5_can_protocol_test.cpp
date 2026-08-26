#include "ti5/can/can_protocol.hpp"
#include "ti5/can/encoder_conversion.hpp"

#include <cmath>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
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
void expectThrows(Callable &&callable, const std::string &message)
{
    bool threw = false;
    try
    {
        callable();
    }
    catch (const std::exception &)
    {
        threw = true;
    }
    expect(threw, message);
}

bool almostEqual(const double left, const double right)
{
    return std::abs(left - right) < 1e-12;
}

robot::can::CanFrame int32Response(const std::uint16_t node_id,
                                   const std::uint8_t command,
                                   const std::uint32_t value)
{
    robot::can::CanFrame frame{};
    frame.id = node_id;
    frame.data_length = 5;
    frame.data = {
        command,
        static_cast<std::uint8_t>(value & 0xFFU),
        static_cast<std::uint8_t>((value >> 8U) & 0xFFU),
        static_cast<std::uint8_t>((value >> 16U) & 0xFFU),
        static_cast<std::uint8_t>((value >> 24U) & 0xFFU),
        0,
        0,
        0};
    return frame;
}

} // namespace

int main()
{
    try
    {
        using namespace robot::ti5;

        constexpr std::uint16_t node_id = 23;
        constexpr std::uint32_t counts_per_revolution = 262144;
        constexpr double pi = 3.1415926535897932384626433832795;

        expect(almostEqual(
                   positionCountsToRadians(65536, counts_per_revolution),
                   pi / 2.0),
               "65536 count must convert to pi/2");
        expect(almostEqual(
                   positionCountsToRadians(-65536, counts_per_revolution),
                   -pi / 2.0),
               "negative count conversion failed");
        expect(radiansToPositionCounts(
                   pi / 2.0,
                   counts_per_revolution) == 65536,
               "pi/2 must convert to 65536 count");
        expect(radiansToPositionCounts(
                   -pi / 2.0,
                   counts_per_revolution) == -65536,
               "negative radian conversion failed");
        expect(almostEqual(
                   speedRawToOutputRadiansPerSecond(101, 101.0),
                   2.0 * pi / 100.0),
               "speed conversion failed");

        const auto stop_request = encodeStopModeRequest(node_id);
        expect(stop_request.id == node_id &&
                   stop_request.data_length == 1 &&
                   stop_request.data[0] == 0x02,
               "0x02 STOP-mode request encoding failed");
        expect(static_cast<std::uint32_t>(DriverRunMode::Stop) == 0U &&
                   static_cast<std::uint32_t>(
                       DriverRunMode::ProfilePosition) == 8U,
               "confirmed driver run-mode values changed");

        const auto position_query = encodePositionQuery(node_id);
        expect(position_query.id == node_id &&
                   position_query.data_length == 1 &&
                   position_query.data[0] == 0x08,
               "0x08 query encoding failed");

        const auto position_response = int32Response(
            node_id,
            0x08,
            static_cast<std::uint32_t>(-65536));
        expect(isPositionQueryResponse(position_response, node_id),
               "valid 0x08 response rejected");
        expect(decodePositionCounts(position_response) == -65536,
               "0x08 signed position decode failed");

        const auto run_mode_query = encodeDriverStatusQuery(
            node_id,
            DriverStatusField::RunMode);
        expect(run_mode_query.data_length == 1 &&
                   run_mode_query.data[0] == 0x03,
               "0x03 run-mode query encoding failed");
        const auto run_mode_response = int32Response(node_id, 0x03, 8U);
        expect(isDriverStatusQueryResponse(
                   run_mode_response,
                   node_id,
                   DriverStatusField::RunMode),
               "valid run-mode response rejected");
        expect(decodeDriverStatusFeedback(
                   run_mode_response,
                   DriverStatusField::RunMode).value == 8U,
               "run-mode response decode failed");

        constexpr std::uint32_t fault_bits = 0x00090042U;
        const auto fault_response = int32Response(
            node_id,
            0x0A,
            fault_bits);
        expect(decodeDriverStatusFeedback(
                   fault_response,
                   DriverStatusField::FaultBits).value == fault_bits,
               "fault bitmap must preserve every bit");
        expect(!isDriverStatusQueryResponse(
                   fault_response,
                   node_id,
                   DriverStatusField::RunMode),
               "fault response must not be accepted as run mode");

        const auto position_limit = int32Response(
            node_id,
            0x1B,
            static_cast<std::uint32_t>(-61895));
        expect(isPositionLimitQueryResponse(
                   position_limit,
                   node_id,
                   PositionLimitKind::Minimum) &&
                   decodePositionLimitCounts(
                       position_limit,
                       PositionLimitKind::Minimum) == -61895,
               "driver position-limit decode failed");

        const auto command = encodePositionCsp(node_id, -65536);
        expect(command.data_length == 5 && command.data[0] == 0x44 &&
                   command.data[1] == 0x00 && command.data[2] == 0x00 &&
                   command.data[3] == 0xFF && command.data[4] == 0xFF,
               "0x44 Position CSP encoding failed");

        robot::can::CanFrame csp{};
        csp.id = node_id;
        csp.data_length = 8;
        csp.data = {0x34, 0x12, 0xFE, 0xFF,
                    0x00, 0x00, 0xFF, 0xFF};
        const auto decoded_csp = decodeCspFeedback(csp);
        expect(decoded_csp.current_milliamps == 0x1234 &&
                   decoded_csp.speed_raw == -2 &&
                   decoded_csp.position_counts == -65536,
               "CSP feedback decode failed");

        auto malformed = position_response;
        malformed.data_length = 6;
        expect(!isPositionQueryResponse(malformed, node_id),
               "non-standard 0x08 DLC must be rejected");
        expectThrows([&] { decodePositionCounts(malformed); },
                     "malformed position response must throw");
        expectThrows([&] { encodePositionQuery(0x800); },
                      "non-standard node ID must throw");
        expectThrows([&] { encodeStopModeRequest(0x800); },
                     "STOP request must reject a non-standard node ID");
        expectThrows([&] { encodePositionQuery(0); },
                     "zero is not a valid TI5 node ID");
        expectThrows(
            [&] {
                radiansToPositionCounts(
                    std::numeric_limits<double>::infinity(),
                    counts_per_revolution);
            },
            "non-finite position must throw");

        std::cout << "TI5 CAN protocol tests passed\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "TI5 CAN protocol test failed: "
                  << error.what() << '\n';
        return 1;
    }
}
