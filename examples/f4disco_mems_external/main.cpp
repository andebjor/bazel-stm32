#include <cstdint>

#include <stm32f4xx_gpio.h>
#include "stm32_i2c.h"
#include "SparkFunLSM9DS1.h"

namespace {

void init(LSM9DS1& external_mems, stm32_i2c::I2C_interface& i2c) {
    constexpr uint8_t agAddress = LSM9DS1_AG_ADDR(1);
    constexpr uint8_t mAddress = LSM9DS1_M_ADDR(1);

    external_mems.begin(agAddress, mAddress, i2c);
}

auto mems_read(LSM9DS1& external_mems) -> void {
    external_mems.readAccel();
}

void delay(uint32_t ms) {
    ms *= 33600;
    while(ms--) {
        asm("nop");
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
