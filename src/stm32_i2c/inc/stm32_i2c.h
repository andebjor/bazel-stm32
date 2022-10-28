// Thin wrapper of I2C functionality over the STM primitives.

#pragma once

#include <cstdint>
#include <optional>

#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_i2c.h"

// The debug build assert for static_vector is not embedded-compatible, so
// force release version.
#ifndef NDEBUG
 #define NDEBUG
 #include "fixed_capacity_vector"
 #undef NDEBUG
#else
 #include "fixed_capacity_vector"
#endif

namespace {

#if 1
void delay(uint32_t ms) {
    ms *= 33600;
    while(ms--) {
        asm("nop");
    }
}
#endif

}


namespace stm32_i2c {

template<size_t NBuf = 1024>
class stm32_i2c {
    public:
        /// @brief Constructor
        ///
        /// @param scl_pin The clock pin to use
        /// @param sda_pin The data pin to use
        /// @param clockspeed Clock speed to use
        stm32_i2c(uint8_t scl_pin, uint8_t sda_pin, uint32_t clockspeed = 400000) noexcept;

        /// @brief No-op: Has to be there as the original lib is not RAII
        auto constexpr begin() const noexcept {}

        /// @brief Initiate a transmission towards a device
        ///
        /// @param address Slave device address
        auto constexpr beginTransmission(uint8_t address) noexcept -> void;

        /// @brief Send one byte of data
        ///
        /// @note Requires a transmission to be initiated
        ///
        /// @param data Payload to send
        ///
        /// @return 0, always
        auto write(uint8_t data) noexcept -> size_t;

        /// @brief TODO
        ///
        /// @param keep_connection TODO
        ///
        /// @return TODO
        auto endTransmission(bool keepConnection=false) const noexcept -> uint8_t;

        /// @brief Request data from a slave device
        ///
        /// @param address What slave to request data from
        /// @param n_bytes How many bytes to request
        ///
        /// @return TODO
        auto requestFrom(uint8_t address, uint8_t n_bytes) noexcept -> uint8_t;

        /// @brief TODO
        ///
        /// @return TODO
        auto read() noexcept -> int;

    private:
        /// @brief Tarpit function called on usage error intented to aid debugging
        [[noreturn]] auto usage_error() const noexcept { while (1) {} }

        /// @brief Tarpit function called on write error intented to aid debugging
        [[noreturn]] auto write_error() const noexcept { while (1) {} }

        /// @brief Tarpit function called on read error intented to aid debugging
        [[noreturn]] auto read_error() const noexcept { while (1) {} }

        /// @brief Initiate a transfer
        auto generate_start() const noexcept -> void;

        /// @brief Send what we have in th transmit buffer
        auto transmit_send_buffer() const noexcept -> void;

        /// @brief Read the specified number of bytes from the device
        ///
        /// The received data is appended to `read_buffer_`.
        ///
        /// @note The device must have beed setup to transmit at least this
        ///     much data.
        ///
        /// @param n_bytes How many bytes to read.
        auto read_bytes(size_t n_bytes) noexcept -> void;

        /// @brief End a receive session
        auto generate_stop() const noexcept -> void;

        uint8_t scl_pin_;
        uint8_t sda_pin_;

        /// @brief The I2C device to use on th STM32 unit
        ///
        /// Cannot be const because ST API is not const-correct, or actually
        /// changes the device struct.
        I2C_TypeDef* i2c_device_{I2C1}; // TODO: Make selectable

        /// @brief Buffer for storing received bytes until the client pops them
        std::experimental::fixed_capacity_vector<uint8_t, NBuf> read_buffer_;

        /// @brief Iterator into the read buffer when the user pops bytes
        typename std::experimental::fixed_capacity_vector<uint8_t, NBuf>::const_iterator read_index_;

        /// @brief Buffer for storing received bytes until the client pops them
        std::experimental::fixed_capacity_vector<uint8_t, NBuf> transmit_buffer_;

        /// @brief Possible modes of the library
        enum Mode: uint8_t {
            Idle,  ///< No active session
            Transmitting,  ///< In a transmission session
            Receiving,  ///< In a receiving session
        };

        /// @brief Keep track of what mode the transmission is in
        Mode transfer_mode_{Idle};

        /// @brief Address to interact with
        std::optional<uint8_t> slave_address_{std::nullopt};
};

using I2C_interface = stm32_i2c<1024>;

template<size_t NBuf>
stm32_i2c<NBuf>::stm32_i2c(uint8_t scl_pin, uint8_t sda_pin, uint32_t clockspeed) noexcept: scl_pin_{scl_pin}, sda_pin_{sda_pin}
{
    // Enable clock buses
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE); // TODO: Make selectable
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); // TODO: Make selectable

    // Configure GPIO for I2C
    GPIO_InitTypeDef gpio;
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = scl_pin | sda_pin;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_50MHz; // TODO: Make selectable?
    gpio.GPIO_OType = GPIO_OType_OD;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1); // TODO: All three args...
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1); // TODO: All three args...
    GPIO_Init(GPIOB, &gpio); // TODO: Make selectable

    // Configure the I2C
    I2C_InitTypeDef i2c;
    I2C_StructInit(&i2c);
//     i2c.I2C_ClockSpeed = clockspeed;
    (void)clockspeed;
    i2c.I2C_ClockSpeed = 10000;
    i2c.I2C_Mode = I2C_Mode_I2C;
    i2c.I2C_DutyCycle = I2C_DutyCycle_2;
    i2c.I2C_OwnAddress1 = 0x33;
    i2c.I2C_Ack = I2C_Ack_Enable;
//     i2c.I2C_Ack = I2C_Ack_Disable;
    i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(i2c_device_, &i2c);

    // Start I2C
    I2C_Cmd(i2c_device_, ENABLE);
    I2C_AcknowledgeConfig(i2c_device_, ENABLE);
}

template<size_t NBuf>
auto constexpr stm32_i2c<NBuf>::beginTransmission(uint8_t address) noexcept -> void
{
    transfer_mode_ = Transmitting;
    transmit_buffer_.clear();
    slave_address_ = address;
}

template<size_t NBuf>
auto stm32_i2c<NBuf>::write(uint8_t data) noexcept -> size_t
{
    if (transfer_mode_ != Transmitting)
    {
        usage_error();
    }

    if (transmit_buffer_.full())
    {
        write_error();
    }
    transmit_buffer_.push_back(data);

    return 1;
}

template<size_t NBuf>
auto stm32_i2c<NBuf>::endTransmission(bool keep_connection) const noexcept -> uint8_t
{
    if (transfer_mode_ != Transmitting)
    {
        usage_error();
    }

    if (transmit_buffer_.empty())
    {
        usage_error();
    }

    generate_start();
    transmit_send_buffer();

    if (!keep_connection || true)
    {
        generate_stop();
    }

    // This seems to be what the Arduino lib does.
    return 0;
}

template<size_t NBuf>
auto stm32_i2c<NBuf>::requestFrom(uint8_t address, uint8_t n_bytes) noexcept -> uint8_t
{
    if (n_bytes > read_buffer_.capacity())
    {
        read_error();
    }

    transfer_mode_ = Receiving;
    read_buffer_.clear();
    slave_address_ = address;

    generate_start();
    read_bytes(n_bytes);
    read_index_ = read_buffer_.begin();

//     generate_stop();

    return 255u;
}

template<size_t NBuf>
auto stm32_i2c<NBuf>::read() noexcept -> int
{
    auto val = *read_index_;
    read_index_++;
    return val;
}

template<size_t NBuf>
auto stm32_i2c<NBuf>::generate_start() const noexcept -> void
{
    delay(20);
    I2C_GenerateSTART(i2c_device_, ENABLE);

    // Waiting for flag
    while(!I2C_CheckEvent(i2c_device_, I2C_EVENT_MASTER_MODE_SELECT))
    {
    }

    // Send address to slave
    const auto address_7bit = static_cast<uint8_t>(*slave_address_ << 1u);
    const auto direction = (
        transfer_mode_ == Transmitting?
        I2C_Direction_Transmitter:
        I2C_Direction_Receiver
    );
    I2C_Send7bitAddress(i2c_device_, address_7bit, direction);

    // And check the transmitting
    const auto expected_event = (
        transfer_mode_ == Transmitting?
        I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED:
        I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED
    );
    while(!I2C_CheckEvent(i2c_device_, expected_event))
    {
    }
}

template<size_t NBuf>
auto stm32_i2c<NBuf>::transmit_send_buffer() const noexcept -> void
{
    for (const auto& data: transmit_buffer_)
    {
        I2C_SendData(i2c_device_, data);

        // wait for the data transmitted flag
        while (!I2C_CheckEvent(i2c_device_, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
        {
        }
    }
}

template<size_t NBuf>
auto stm32_i2c<NBuf>::read_bytes(size_t n_bytes) noexcept -> void
{
//     n_bytes += 2;
    while (n_bytes-- > 0)
    {
        if (n_bytes == 0)
        {
            generate_stop();
        }
        while (!I2C_CheckEvent(i2c_device_, I2C_EVENT_MASTER_BYTE_RECEIVED))
        {
        }
        read_buffer_.push_back(I2C_ReceiveData(i2c_device_));
    }

#if 0
    for (size_t i=0; i<11; ++i)
    {
        GPIO_ResetBits(GPIOB, scl_pin_);
        size_t mus = 1000;
        while(mus--) {
            asm("nop");
        }
        GPIO_SetBits(GPIOB, scl_pin_);
        mus = 1000;
        while(mus--) {
            asm("nop");
        }
    }
#endif
}

template<size_t NBuf>
auto stm32_i2c<NBuf>::generate_stop() const noexcept -> void
{
    I2C_GenerateSTOP(i2c_device_, ENABLE);
}

}  // namespace stm32_i2c
