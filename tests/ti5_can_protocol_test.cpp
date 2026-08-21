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
    void expect(bool condition, const std::string &message)
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

    bool almostEqual(double lhs, double rhs)
    {
        return std::abs(lhs - rhs) < 1e-12;
    }
}

int main()
{
    try
    {
        using robot::can::CanFrame;
        using namespace robot::ti5;

        constexpr std::uint32_t counts_per_revolution = 262144;
        constexpr std::uint16_t node_id = 23;
        constexpr double pi = 3.1415926535897932384626433832795;

        // T170C 输出端位置换算：不再除以 gear_ratio。
        expect(almostEqual(positionCountsToRadians(0, counts_per_revolution), 0.0),
               "0 count must convert to 0 rad");
        expect(almostEqual(positionCountsToRadians(65536, counts_per_revolution), pi / 2.0),
               "65536 count must convert to pi/2");
        expect(almostEqual(positionCountsToRadians(-65536, counts_per_revolution), -pi / 2.0),
               "-65536 count must convert to -pi/2");
        expect(radiansToPositionCounts(0.0, counts_per_revolution) == 0,
               "0 rad must convert to 0 count");
        expect(radiansToPositionCounts(pi / 2.0, counts_per_revolution) == 65536,
               "pi/2 must convert to 65536 count");
        expect(radiansToPositionCounts(-pi / 2.0, counts_per_revolution) == -65536,
               "-pi/2 must convert to -65536 count");

        const auto query = encodePositionQuery(node_id);
        expect(query.id == node_id, "0x08 query CAN ID mismatch");
        expect(query.data_length == 1, "0x08 query DLC mismatch");
        expect(query.data[0] == 0x08, "0x08 query command mismatch");

        CanFrame positive_response{};
        positive_response.id = node_id;
        positive_response.data_length = 5;
        positive_response.data = {0x08, 0x78, 0x56, 0x34, 0x12, 0, 0, 0};
        expect(isPositionQueryResponse(positive_response, node_id),
               "valid positive 0x08 response rejected");
        expect(decodePositionCounts(positive_response) == 0x12345678,
               "positive little-endian position decode failed");

        CanFrame negative_response{};
        negative_response.id = node_id;
        negative_response.data_length = 5;
        negative_response.data = {0x08, 0x00, 0x00, 0xFF, 0xFF, 0, 0, 0};
        expect(isPositionQueryResponse(negative_response, node_id),
               "valid negative 0x08 response rejected");
        expect(decodePositionCounts(negative_response) == -65536,
               "negative little-endian position decode failed");

        // 严格检查 11-bit CAN ID、DLC、命令字和期望节点 ID。
        CanFrame malformed = positive_response;
        malformed.data_length = 4;
        expect(!isPositionQueryResponse(malformed, node_id), "DLC=4 must be rejected");
        expectThrows([&] { decodePositionCounts(malformed); },
                     "decode must reject DLC=4");

        malformed = positive_response;
        malformed.data_length = 9;
        expect(!isPositionQueryResponse(malformed, node_id), "DLC=9 must be rejected");
        expectThrows([&] { decodePositionCounts(malformed); },
                     "decode must reject DLC=9");

        malformed = positive_response;
        malformed.data[0] = 0x09;
        expect(!isPositionQueryResponse(malformed, node_id),
               "wrong 0x08 response command must be rejected");
        expectThrows([&] { decodePositionCounts(malformed); },
                     "decode must reject wrong response command");

        expect(!isPositionQueryResponse(positive_response, node_id + 1),
               "wrong expected node ID must be rejected");

        malformed = positive_response;
        malformed.id = 0x800;
        expect(!isPositionQueryResponse(malformed, node_id),
               "non-standard response CAN ID must be rejected");
        expectThrows([&] { decodePositionCounts(malformed); },
                     "decode must reject non-standard response CAN ID");
        expectThrows([&] { encodePositionQuery(0x800); },
                     "encoder must reject non-standard node ID");

        // A 级确认的 CSP 查询、反馈和 Position CSP 编码。
        const auto csp_query = encodeCspQuery(node_id);
        expect(csp_query.id == node_id && csp_query.data_length == 1 && csp_query.data[0] == 0x41,
               "0x41 CSP query encoding failed");

        const auto position_csp = encodePositionCsp(node_id, -65536);
        expect(position_csp.id == node_id && position_csp.data_length == 5,
               "0x44 Position CSP header encoding failed");
        expect(position_csp.data[0] == 0x44 && position_csp.data[1] == 0x00 &&
                   position_csp.data[2] == 0x00 && position_csp.data[3] == 0xFF &&
                   position_csp.data[4] == 0xFF,
               "0x44 Position CSP little-endian encoding failed");

        CanFrame csp_feedback_frame{};
        csp_feedback_frame.id = node_id;
        csp_feedback_frame.data_length = 8;
        csp_feedback_frame.data = {0x34, 0x12, 0xFE, 0xFF,
                                   0x00, 0x00, 0xFF, 0xFF};
        expect(isCspFeedback(csp_feedback_frame, node_id),
               "valid CSP feedback rejected");
        const auto csp_feedback = decodeCspFeedback(csp_feedback_frame);
        expect(csp_feedback.node_id == node_id &&
                   csp_feedback.current_milliamps == 0x1234 &&
                   csp_feedback.speed_raw == -2 &&
                   csp_feedback.position_counts == -65536,
               "CSP feedback decode failed");

        malformed = csp_feedback_frame;
        malformed.data_length = 7;
        expect(!isCspFeedback(malformed, node_id), "CSP DLC=7 must be rejected");
        expectThrows([&] { decodeCspFeedback(malformed); },
                     "CSP decode must reject DLC=7");

        expectThrows([&] { positionCountsToRadians(0, 0); },
                     "zero counts per output revolution must be rejected");
        expectThrows([&] { radiansToPositionCounts(std::numeric_limits<double>::infinity(), counts_per_revolution); },
                     "non-finite radians must be rejected");

        std::cout << "TI5 CAN protocol tests passed\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "TI5 CAN protocol test failed: " << error.what() << '\n';
        return 1;
    }
}
