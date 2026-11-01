#include <array>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "bridge_controller.hpp"
#include "motor_protocol.hpp"
#include "pc_protocol.hpp"

// Plain check instead of assert(): it must stay active in every build type.
#define CHECK(condition)                                                         \
    do {                                                                         \
        if (!(condition)) {                                                      \
            std::printf("%s:%d: FAILED: %s\n", __FILE__, __LINE__, #condition); \
            std::exit(1);                                                        \
        }                                                                        \
    } while (false)

namespace {

using bridge::CommandResult;
using bridge::Controller;
using bridge::Output;
using bridge::State;
using bridge::StopReason;
using motor_protocol::CanFrame;
using motor_protocol::Feedback;

std::vector<pc_protocol::ParseResult> feed(pc_protocol::StreamParser& parser,
                                           const std::vector<std::uint8_t>& bytes) {
    std::vector<pc_protocol::ParseResult> results;
    for (const std::uint8_t byte : bytes) {
        const auto result = parser.push(byte);
        if (result.frame || result.error) {
            results.push_back(result);
        }
    }
    return results;
}

std::vector<std::uint8_t> toVector(const pc_protocol::EncodedFrame& encoded) {
    return {encoded.bytes.begin(), encoded.bytes.begin() + static_cast<long>(encoded.size)};
}

void testCrcMatchesKnownVector() {
    const std::array<std::uint8_t, 9> check{'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    CHECK(pc_protocol::crc16(check) == 0x29B1);
}

void testParserRoundTripsPayload() {
    const std::array<std::uint8_t, 4> payload{1, 2, 3, 4};
    pc_protocol::StreamParser parser;
    const auto results =
        feed(parser, toVector(pc_protocol::encode(pc_protocol::MessageType::setVelocity, payload)));
    CHECK(results.size() == 1);
    CHECK(results[0].frame.has_value());
    CHECK(results[0].frame->type == 0x03);
    CHECK(results[0].frame->payloadSize == 4);
    CHECK(results[0].frame->payload[3] == 4);
}

void testParserReportsCorruptedCrcThenRecovers() {
    const std::array<std::uint8_t, 2> payload{9, 9};
    auto bytes = toVector(pc_protocol::encode(pc_protocol::MessageType::setVelocity, payload));
    const auto valid = bytes;
    bytes[4] ^= 0xFF;
    bytes.insert(bytes.end(), valid.begin(), valid.end());
    pc_protocol::StreamParser parser;
    const auto results = feed(parser, bytes);
    CHECK(results.size() == 2);
    CHECK(results[0].error == pc_protocol::ErrorCode::crcMismatch);
    CHECK(results[1].frame.has_value());
}

void testParserRejectsOversizedLengthAndSkipsGarbage() {
    pc_protocol::StreamParser parser;
    auto results = feed(parser, {0x00, 0xAA, 0xAA, 0x55, 0x01, 0xFF});
    CHECK(results.size() == 1);
    CHECK(results[0].error == pc_protocol::ErrorCode::invalidLength);
    results = feed(parser, toVector(pc_protocol::encode(pc_protocol::MessageType::enable, {})));
    CHECK(results.size() == 1);
    CHECK(results[0].frame->type == 0x01);
}

void testVelocityCommandIsBigEndianTwosComplement() {
    const CanFrame frame = motor_protocol::makeVelocityCommand(100, -100);
    CHECK(frame.id == 0x32);
    const std::array<std::uint8_t, 8> expected{0x00, 0x64, 0xFF, 0x9C, 0, 0, 0, 0};
    CHECK(frame.data == expected);
}

void testSetModeOnlyAddressesTwoMotors() {
    const CanFrame frame = motor_protocol::makeSetMode(motor_protocol::MODE_VELOCITY);
    CHECK(frame.id == 0x105);
    const std::array<std::uint8_t, 8> expected{2, 2, 0, 0, 0, 0, 0, 0};
    CHECK(frame.data == expected);
}

void testFeedbackParsing() {
    const CanFrame frame{.id = 0x98, .data = {0xFF, 0x9C, 0x01, 0x00, 0x7F, 0xFF, 0x0A, 0x02}};
    const auto feedback = motor_protocol::parseFeedback(frame);
    CHECK(feedback.has_value());
    CHECK(feedback->motorId == 2);
    CHECK(feedback->velocityRpm == -100);
    CHECK(feedback->currentRaw == 256);
    CHECK(feedback->angleRaw == 0x7FFF);
    CHECK(feedback->fault == 0x0A);
    CHECK(feedback->mode == 2);
}

void testFeedbackParsingIgnoresOtherIds() {
    for (const std::uint16_t id : {0x32, 0x96, 0x99, 0x201, 0x265}) {
        CHECK(!motor_protocol::parseFeedback(CanFrame{.id = id, .data = {}}).has_value());
    }
}

Feedback healthyFeedback(std::uint8_t motorId) {
    return {.motorId = motorId, .velocityRpm = 0, .currentRaw = 0, .angleRaw = 0, .fault = 0,
            .mode = motor_protocol::MODE_VELOCITY};
}

void feedBoth(Controller& controller, std::uint32_t nowMs) {
    controller.onFeedback(healthyFeedback(1), nowMs);
    controller.onFeedback(healthyFeedback(2), nowMs);
}

Controller makeActiveController(std::uint32_t nowMs) {
    Controller controller;
    Output output;
    CHECK(controller.enable(nowMs, output) == CommandResult::accepted);
    feedBoth(controller, nowMs);
    (void)controller.tick(nowMs);
    CHECK(controller.state() == State::active);
    return controller;
}

bool containsMode(const Output& output, std::uint8_t mode) {
    for (std::size_t index = 0; index < output.count; ++index) {
        if (output.frames[index].id == motor_protocol::SET_MODE_ID &&
            output.frames[index].data[0] == mode) {
            return true;
        }
    }
    return false;
}

void testDisabledKeepsRepeatingSafeCommandsFromFirstTick() {
    Controller controller;
    const Output first = controller.tick(1);
    CHECK(first.count == 3);
    const std::array<std::uint8_t, 8> zero{};
    CHECK(first.frames[0].id == 0x32);
    CHECK(first.frames[0].data == zero);
    CHECK(first.frames[1].id == 0x105);
    CHECK(first.frames[1].data[0] == motor_protocol::MODE_DISABLE);
    CHECK(first.frames[2].id == 0x106);
    CHECK(first.frames[2].data[0] == 10);
    CHECK(controller.tick(100).count == 0);
    CHECK(controller.tick(101).count == 3);
}

void testKeepsDisablingAfterStoppingFinishes() {
    Controller controller = makeActiveController(0);
    CHECK(controller.disable(1) == CommandResult::accepted);
    CHECK(containsMode(controller.tick(501), motor_protocol::MODE_DISABLE));
    CHECK(controller.tick(600).count == 0);
    CHECK(containsMode(controller.tick(601), motor_protocol::MODE_DISABLE));
}

void testArmingKeepsSendingZeroVelocity() {
    Controller controller;
    Output output;
    CHECK(controller.enable(0, output) == CommandResult::accepted);
    const Output periodic = controller.tick(20);
    CHECK(periodic.count == 1);
    const std::array<std::uint8_t, 8> zero{};
    CHECK(periodic.frames[0].id == 0x32);
    CHECK(periodic.frames[0].data == zero);
}

void testModeLossStops() {
    Controller controller = makeActiveController(0);
    Feedback openLoop = healthyFeedback(1);
    openLoop.mode = 0;
    controller.onFeedback(openLoop, 1);
    (void)controller.tick(1);
    CHECK(controller.state() == State::stopping);
    CHECK(controller.stopReason() == StopReason::modeLost);
}

void testCanTransmitFailureStops() {
    Controller controller = makeActiveController(0);
    controller.onCanTransmitFailure(1);
    CHECK(controller.state() == State::stopping);
    CHECK(controller.stopReason() == StopReason::canTransmitFailed);
}

void testCanTransmitFailureWhileDisabledKeepsReason() {
    Controller controller;
    controller.onCanTransmitFailure(1);
    CHECK(controller.state() == State::disabled);
    CHECK(controller.stopReason() == StopReason::none);
}

void testEnableSendsModesInOrder() {
    Controller controller;
    Output output;
    CHECK(controller.enable(0, output) == CommandResult::accepted);
    CHECK(output.count == 3);
    CHECK(output.frames[0].id == 0x32);
    CHECK(output.frames[1].data[0] == motor_protocol::MODE_ENABLE);
    CHECK(output.frames[2].data[0] == motor_protocol::MODE_VELOCITY);
    CHECK(controller.state() == State::arming);
}

void testStaleVelocityModeDoesNotArm() {
    Controller controller;
    feedBoth(controller, 0);
    Output output;
    CHECK(controller.enable(10, output) == CommandResult::accepted);
    (void)controller.tick(11);
    CHECK(controller.state() == State::arming);
}

void testArmingRequiresBothMotors() {
    Controller controller;
    Output output;
    CHECK(controller.enable(0, output) == CommandResult::accepted);
    controller.onFeedback(healthyFeedback(1), 1);
    (void)controller.tick(2);
    CHECK(controller.state() == State::arming);
}

void testArmingTimesOutAndDisables() {
    Controller controller;
    Output output;
    CHECK(controller.enable(0, output) == CommandResult::accepted);
    const Output timedOut = controller.tick(500);
    CHECK(controller.state() == State::disabled);
    CHECK(controller.stopReason() == StopReason::armingTimeout);
    CHECK(containsMode(timedOut, motor_protocol::MODE_DISABLE));
}

void testRejectsVelocityWhenNotActive() {
    Controller controller;
    CHECK(controller.setVelocity(10, 10, 0) == CommandResult::invalidState);
}

void testRejectsOutOfRangeWithoutFeedingTimeout() {
    Controller controller = makeActiveController(0);
    CHECK(controller.setVelocity(bridge::MAX_VELOCITY_RPM + 1, 0, 100) == CommandResult::velocityOutOfRange);
    CHECK(controller.setVelocity(0, -bridge::MAX_VELOCITY_RPM - 1, 100) == CommandResult::velocityOutOfRange);
    feedBoth(controller, 199);
    (void)controller.tick(200);
    CHECK(controller.state() == State::stopping);
    CHECK(controller.stopReason() == StopReason::commandTimeout);
}

void testActiveSendsLatestVelocityPeriodically() {
    Controller controller = makeActiveController(0);
    CHECK(controller.setVelocity(25, -25, 1) == CommandResult::accepted);
    feedBoth(controller, 20);
    const Output first = controller.tick(20);
    CHECK(first.count == 1);
    CHECK(first.frames[0].id == 0x32);
    CHECK(first.frames[0].data[1] == 25);
    CHECK(controller.tick(21).count == 0);
    feedBoth(controller, 40);
    CHECK(controller.tick(40).count == 1);
}

void testCommandTimeoutZeroesThenDisables() {
    Controller controller = makeActiveController(0);
    CHECK(controller.setVelocity(20, 20, 0) == CommandResult::accepted);
    feedBoth(controller, 199);
    (void)controller.tick(199);
    CHECK(controller.state() == State::active);
    feedBoth(controller, 200);
    const Output stopping = controller.tick(200);
    CHECK(controller.state() == State::stopping);
    CHECK(stopping.count == 1);
    const std::array<std::uint8_t, 8> zero{};
    CHECK(stopping.frames[0].data == zero);
    const Output done = controller.tick(700);
    CHECK(controller.state() == State::disabled);
    CHECK(containsMode(done, motor_protocol::MODE_DISABLE));
    CHECK(controller.stopReason() == StopReason::commandTimeout);
}

void testFeedbackLossStops() {
    Controller controller = makeActiveController(0);
    CHECK(controller.setVelocity(10, 10, 99) == CommandResult::accepted);
    controller.onFeedback(healthyFeedback(1), 99);
    (void)controller.tick(100);
    CHECK(controller.state() == State::stopping);
    CHECK(controller.stopReason() == StopReason::feedbackLost);
}

void testMotorFaultStopsAndBlocksReenable() {
    Controller controller = makeActiveController(0);
    Feedback faulted = healthyFeedback(2);
    faulted.fault = 0x0A;
    controller.onFeedback(faulted, 1);
    (void)controller.tick(1);
    CHECK(controller.state() == State::stopping);
    CHECK(controller.stopReason() == StopReason::motorFault);
    (void)controller.tick(501);
    CHECK(controller.state() == State::disabled);
    Output output;
    CHECK(controller.enable(502, output) == CommandResult::invalidState);
    feedBoth(controller, 503);
    CHECK(controller.enable(504, output) == CommandResult::accepted);
}

void testDisableRequestStopsAndDoesNotAutoResume() {
    Controller controller = makeActiveController(0);
    CHECK(controller.setVelocity(20, 20, 1) == CommandResult::accepted);
    CHECK(controller.disable(2) == CommandResult::accepted);
    CHECK(controller.state() == State::stopping);
    CHECK(controller.setVelocity(20, 20, 3) == CommandResult::invalidState);
    (void)controller.tick(502);
    CHECK(controller.state() == State::disabled);
    feedBoth(controller, 503);
    (void)controller.tick(503);
    CHECK(controller.state() == State::disabled);
}

void testTimeoutWorksAcrossMillisecondWrap() {
    const std::uint32_t start = 0xFFFFFFF0U;
    Controller controller = makeActiveController(start);
    const std::uint32_t later = start + 200U;
    feedBoth(controller, later);
    (void)controller.tick(later);
    CHECK(controller.state() == State::stopping);
}

}  // namespace

int main() {
    testCrcMatchesKnownVector();
    testParserRoundTripsPayload();
    testParserReportsCorruptedCrcThenRecovers();
    testParserRejectsOversizedLengthAndSkipsGarbage();
    testVelocityCommandIsBigEndianTwosComplement();
    testSetModeOnlyAddressesTwoMotors();
    testFeedbackParsing();
    testFeedbackParsingIgnoresOtherIds();
    testDisabledKeepsRepeatingSafeCommandsFromFirstTick();
    testKeepsDisablingAfterStoppingFinishes();
    testArmingKeepsSendingZeroVelocity();
    testModeLossStops();
    testCanTransmitFailureStops();
    testCanTransmitFailureWhileDisabledKeepsReason();
    testEnableSendsModesInOrder();
    testStaleVelocityModeDoesNotArm();
    testArmingRequiresBothMotors();
    testArmingTimesOutAndDisables();
    testRejectsVelocityWhenNotActive();
    testRejectsOutOfRangeWithoutFeedingTimeout();
    testActiveSendsLatestVelocityPeriodically();
    testCommandTimeoutZeroesThenDisables();
    testFeedbackLossStops();
    testMotorFaultStopsAndBlocksReenable();
    testDisableRequestStopsAndDoesNotAutoResume();
    testTimeoutWorksAcrossMillisecondWrap();
    std::puts("all tests passed");
    return 0;
}
