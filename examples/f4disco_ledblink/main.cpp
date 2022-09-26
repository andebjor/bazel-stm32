#include <cstdint>

#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>

#include <cpp_test.h>

namespace {

const uint16_t LEDS = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
const uint16_t LED[4] = {GPIO_Pin_12, GPIO_Pin_13, GPIO_Pin_14, GPIO_Pin_15};

void init() {
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    GPIO_InitTypeDef gpio;
    GPIO_StructInit(&gpio);
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_Pin = LEDS;
    GPIO_Init(GPIOD, &gpio);

    GPIO_SetBits(GPIOD, LEDS);
}

void delay(uint32_t ms) {
    ms *= 33600;
    while(ms--) {
        __NOP();
    }
}

void loop() {
    static uint32_t counter = 0;

    ++counter;

    GPIO_ResetBits(GPIOD, LEDS);
/*     GPIO_SetBits(GPIOD, LED[counter % 4]); */

    if (counter % 2 == 0)
    {
        GPIO_SetBits(GPIOD, LED[cpp_led_on()]);
    }

    delay(250);
}

} // anonymous namespace

[[noreturn]] int main() {
    init();

    do {
        loop();
    } while (1);
}
