#include <cstdint>

#include <stm32f4xx_gpio.h>
#include "stm32_i2c.h"
#include "SparkFunLSM9DS1.h"

int16_t mag_x;

namespace {

#if 0
void delay(uint32_t ms) {
    ms *= 33600;
    while(ms--) {
        asm("nop");
    }
}
#endif

/// @brief MEMS unit is power by pin PD6
auto power_mems() -> void {
    constexpr auto mems_gpio_clock = RCC_AHB1Periph_GPIOD;
    auto* mems_gpio_bank = GPIOD;
    constexpr auto mems_power_pin = GPIO_Pin_6;

    RCC_AHB1PeriphClockCmd(mems_gpio_clock, ENABLE);

    GPIO_InitTypeDef gpio;
    GPIO_StructInit(&gpio);
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_Pin = mems_power_pin;
    GPIO_Init(mems_gpio_bank, &gpio);

//     GPIO_ResetBits(mems_gpio_bank, mems_power_pin);
//     delay(3000);  // TODO: Do we need this? What delay do we need?
    GPIO_SetBits(mems_gpio_bank, mems_power_pin);
}

auto init_mems(LSM9DS1& external_mems, stm32_i2c::I2C_interface& i2c) -> void {
    constexpr uint8_t agAddress = LSM9DS1_AG_ADDR(1);
    constexpr uint8_t mAddress = LSM9DS1_M_ADDR(1);

    external_mems.begin(agAddress, mAddress, i2c);
}

auto init(LSM9DS1& external_mems, stm32_i2c::I2C_interface& i2c) -> void {
    power_mems();

    init_mems(external_mems, i2c);
}

auto mems_read(LSM9DS1& external_mems) -> void {
//     external_mems.readAccel();
    auto mag_avaliable = external_mems.magAvailable();
    if (mag_avaliable)
    {
        mag_x = external_mems.readMag(X_AXIS);
    }
}

void loop(LSM9DS1& external_mems) {
    static uint32_t counter = 0;

    ++counter;

    if (counter % 2 == 0)
    {
        mems_read(external_mems);
    }

    delay(250);
}

} // anonymous namespace

[[noreturn]] int main() {
    auto i2c = stm32_i2c::I2C_interface{GPIO_Pin_6, GPIO_Pin_7};
    auto external_mems = LSM9DS1{};

    init(external_mems, i2c);

    do {
        loop(external_mems);
    } while (1);
}
