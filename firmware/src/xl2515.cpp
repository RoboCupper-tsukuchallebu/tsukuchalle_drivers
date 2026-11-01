#include "xl2515.hpp"

#include <array>

#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"

namespace xl2515 {
namespace {

spi_inst_t* const SPI_PORT = spi1;
constexpr uint SCLK_PIN = 10;
constexpr uint MOSI_PIN = 11;
constexpr uint MISO_PIN = 12;
constexpr uint CHIP_SELECT_PIN = 9;
constexpr uint SPI_BAUD_HZ = 10'000'000;

constexpr std::uint8_t COMMAND_RESET = 0xC0;
constexpr std::uint8_t COMMAND_WRITE = 0x02;
constexpr std::uint8_t COMMAND_READ = 0x03;
constexpr std::uint8_t COMMAND_BIT_MODIFY = 0x05;
constexpr std::uint8_t COMMAND_READ_RX_BUFFER_0 = 0x90;
constexpr std::uint8_t COMMAND_READ_RX_BUFFER_1 = 0x94;
constexpr std::uint8_t COMMAND_LOAD_TX_BUFFER_0 = 0x40;
constexpr std::uint8_t COMMAND_REQUEST_TO_SEND_0 = 0x81;
constexpr std::uint8_t COMMAND_READ_STATUS = 0xA0;

constexpr std::uint8_t REGISTER_CANSTAT = 0x0E;
constexpr std::uint8_t REGISTER_CANCTRL = 0x0F;
constexpr std::uint8_t REGISTER_CNF3 = 0x28;
constexpr std::uint8_t REGISTER_CNF2 = 0x29;
constexpr std::uint8_t REGISTER_CNF1 = 0x2A;
constexpr std::uint8_t REGISTER_CANINTE = 0x2B;
constexpr std::uint8_t REGISTER_TXB0CTRL = 0x30;
constexpr std::uint8_t REGISTER_RXB0CTRL = 0x60;
constexpr std::uint8_t REGISTER_RXB1CTRL = 0x70;

// 16 MHz, 500 kbit/s: 16 time quanta of 125 ns, sample point 75 %.
constexpr std::uint8_t CNF1_500K_16MHZ = 0x00;
constexpr std::uint8_t CNF2_500K_16MHZ = 0x9E;
constexpr std::uint8_t CNF3_500K_16MHZ = 0x03;

constexpr std::uint8_t MODE_NORMAL = 0x00;
constexpr std::uint8_t MODE_MASK = 0xE0;
constexpr std::uint8_t RECEIVE_ANY_WITH_ROLLOVER = 0x64;
constexpr std::uint8_t RECEIVE_ANY = 0x60;
constexpr std::uint8_t TXREQ_BIT = 0x08;
constexpr std::uint8_t RX_BUFFER_0_FULL = 0x01;
constexpr std::uint8_t RX_BUFFER_1_FULL = 0x02;
constexpr std::uint8_t ENABLE_RX_INTERRUPT_FLAGS = 0x03;
constexpr std::uint8_t EXTENDED_FRAME_BIT = 0x08;
constexpr std::size_t CAN_DATA_SIZE = 8;
constexpr std::uint32_t RESET_SETTLE_MS = 5;

class ChipSelectGuard {
public:
    ChipSelectGuard() { gpio_put(CHIP_SELECT_PIN, false); }
    // The wait guarantees the minimum CS-high time between back-to-back commands.
    ~ChipSelectGuard() {
        gpio_put(CHIP_SELECT_PIN, true);
        busy_wait_us_32(1);
    }
    ChipSelectGuard(const ChipSelectGuard&) = delete;
    ChipSelectGuard& operator=(const ChipSelectGuard&) = delete;
};

void writeRegister(std::uint8_t address, std::uint8_t value) {
    const std::array<std::uint8_t, 3> message{COMMAND_WRITE, address, value};
    ChipSelectGuard guard;
    spi_write_blocking(SPI_PORT, message.data(), message.size());
}

std::uint8_t readRegister(std::uint8_t address) {
    const std::array<std::uint8_t, 2> message{COMMAND_READ, address};
    std::uint8_t value = 0;
    ChipSelectGuard guard;
    spi_write_blocking(SPI_PORT, message.data(), message.size());
    spi_read_blocking(SPI_PORT, 0, &value, 1);
    return value;
}

void modifyRegister(std::uint8_t address, std::uint8_t mask, std::uint8_t value) {
    const std::array<std::uint8_t, 4> message{COMMAND_BIT_MODIFY, address, mask, value};
    ChipSelectGuard guard;
    spi_write_blocking(SPI_PORT, message.data(), message.size());
}

std::uint8_t readStatus() {
    std::uint8_t status = 0;
    ChipSelectGuard guard;
    spi_write_blocking(SPI_PORT, &COMMAND_READ_STATUS, 1);
    spi_read_blocking(SPI_PORT, 0, &status, 1);
    return status;
}

// Reading through this command also clears the matching RXnIF flag when CS rises.
std::optional<motor_protocol::CanFrame> readRxBuffer(std::uint8_t command) {
    // SIDH, SIDL, EID8, EID0, DLC, then eight data bytes.
    std::array<std::uint8_t, 5 + CAN_DATA_SIZE> raw{};
    {
        ChipSelectGuard guard;
        spi_write_blocking(SPI_PORT, &command, 1);
        spi_read_blocking(SPI_PORT, 0, raw.data(), raw.size());
    }
    if ((raw[1] & EXTENDED_FRAME_BIT) != 0) {
        return std::nullopt;
    }
    motor_protocol::CanFrame frame;
    frame.id = static_cast<std::uint16_t>((static_cast<std::uint16_t>(raw[0]) << 3) | (raw[1] >> 5));
    for (std::size_t index = 0; index < CAN_DATA_SIZE; ++index) {
        frame.data[index] = raw[5 + index];
    }
    return frame;
}

}  // namespace

bool initialize() {
    spi_init(SPI_PORT, SPI_BAUD_HZ);
    gpio_set_function(SCLK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(MOSI_PIN, GPIO_FUNC_SPI);
    gpio_set_function(MISO_PIN, GPIO_FUNC_SPI);
    gpio_init(CHIP_SELECT_PIN);
    gpio_put(CHIP_SELECT_PIN, true);
    gpio_set_dir(CHIP_SELECT_PIN, GPIO_OUT);

    {
        ChipSelectGuard guard;
        spi_write_blocking(SPI_PORT, &COMMAND_RESET, 1);
    }
    sleep_ms(RESET_SETTLE_MS);

    writeRegister(REGISTER_CNF1, CNF1_500K_16MHZ);
    writeRegister(REGISTER_CNF2, CNF2_500K_16MHZ);
    writeRegister(REGISTER_CNF3, CNF3_500K_16MHZ);
    writeRegister(REGISTER_RXB0CTRL, RECEIVE_ANY_WITH_ROLLOVER);
    writeRegister(REGISTER_RXB1CTRL, RECEIVE_ANY);
    // RXnIF flags are polled through READ STATUS; the INT pin is not used.
    writeRegister(REGISTER_CANINTE, ENABLE_RX_INTERRUPT_FLAGS);
    writeRegister(REGISTER_CANCTRL, MODE_NORMAL);
    sleep_ms(RESET_SETTLE_MS);

    return (readRegister(REGISTER_CANSTAT) & MODE_MASK) == MODE_NORMAL &&
           readRegister(REGISTER_CNF2) == CNF2_500K_16MHZ;
}

bool isTransmitBusy() {
    return (readRegister(REGISTER_TXB0CTRL) & TXREQ_BIT) != 0;
}

void abortTransmit() {
    modifyRegister(REGISTER_TXB0CTRL, TXREQ_BIT, 0);
}

void transmit(const motor_protocol::CanFrame& frame) {
    std::array<std::uint8_t, 6 + CAN_DATA_SIZE> message{};
    message[0] = COMMAND_LOAD_TX_BUFFER_0;
    message[1] = static_cast<std::uint8_t>(frame.id >> 3);
    message[2] = static_cast<std::uint8_t>((frame.id & 0x07) << 5);
    message[3] = 0;
    message[4] = 0;
    message[5] = CAN_DATA_SIZE;
    for (std::size_t index = 0; index < CAN_DATA_SIZE; ++index) {
        message[6 + index] = frame.data[index];
    }
    {
        ChipSelectGuard guard;
        spi_write_blocking(SPI_PORT, message.data(), message.size());
    }
    ChipSelectGuard guard;
    spi_write_blocking(SPI_PORT, &COMMAND_REQUEST_TO_SEND_0, 1);
}

std::optional<motor_protocol::CanFrame> receive() {
    const std::uint8_t status = readStatus();
    if ((status & RX_BUFFER_0_FULL) != 0) {
        return readRxBuffer(COMMAND_READ_RX_BUFFER_0);
    }
    if ((status & RX_BUFFER_1_FULL) != 0) {
        return readRxBuffer(COMMAND_READ_RX_BUFFER_1);
    }
    return std::nullopt;
}

}  // namespace xl2515
