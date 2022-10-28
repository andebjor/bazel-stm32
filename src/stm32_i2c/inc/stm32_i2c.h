// Thin wrapper of I2C functionality over the STM primitives.

#pragma once

#include <cstdint>

#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_i2c.h"
#include "fixed_capacity_vector"
#include "stm32f4xx_i2c.h"

namespace stm32_i2c {

template<size_t NBuf = 1024>
class stm32_i2c {
    public:
        stm32_i2c(uint8_t sda_pin, uint8_t scl_pin, uint32_t clockspeed = 400000);

        /// @brief No-op: Has to be there as the original lib is not RAII
        auto begin() {}

        auto beginTransmission(uint8_t address) -> void;
        auto write(uint8_t data) -> size_t;
        auto endTransmission(bool keepConnection=false) -> uint8_t;

        auto requestFrom(uint8_t address, uint8_t nBytes) -> uint8_t;
        auto read() -> int;

    private:
        const I2C_TypeDef* i2c_device_ = I2C1; // TODO: Make selectable

        std::experimental::fixed_capacity_vector<uint8_t, NBuf> read_buffer_;
};

using I2C_interface = stm32_i2c<1024>;

template<size_t NBuf>
stm32_i2c<NBuf>::stm32_i2c(uint8_t sda_pin, uint8_t scl_pin, uint32_t clockspeed)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    GPIO_InitTypeDef gpio;
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = sda_pin | scl_pin;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_OType = GPIO_OType_OD;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &gpio);
    I2C_InitTypeDef i2c;
    I2C_StructInit(&i2c);
    i2c.I2C_ClockSpeed = clockspeed;
    i2c.I2C_Mode = I2C_Mode_I2C;
    i2c.I2C_DutyCycle = I2C_DutyCycle_2;
    i2c.I2C_OwnAddress1 = 0x33;
    i2c.I2C_Ack = I2C_Ack_Enable;
    i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C1, &i2c);
    I2C_Cmd(I2C1, ENABLE);
    I2C_AcknowledgeConfig(I2C1, ENABLE);
}

template<size_t NBuf>
auto stm32_i2c<NBuf>::beginTransmission(uint8_t address) -> void
{
    while (1) {}
}

template<size_t NBuf>
auto stm32_i2c<NBuf>::write(uint8_t data) -> size_t
{
    while (1) {}
}

template<size_t NBuf>
auto stm32_i2c<NBuf>::endTransmission(bool keepConnection) -> uint8_t
{
    while (1) {}
}

template<size_t NBuf>
auto stm32_i2c<NBuf>::requestFrom(uint8_t address, uint8_t nBytes) -> uint8_t
{
    while (1) {}
}

template<size_t NBuf>
auto stm32_i2c<NBuf>::read() -> int
{
    while (1) {}
}

}  // namespace stm32_i2c
