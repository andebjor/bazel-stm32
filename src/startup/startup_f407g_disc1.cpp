#include <algorithm>
#include <cstdint>

#include "system_stm32f4xx.h"

// Symbols defined by the linker
extern void (*__preinit_array_start []) (void);
extern void (*__preinit_array_end []) (void);
extern void (*__init_array_start []) (void);
extern void (*__init_array_end []) (void);

extern std::uint32_t __StackTop;
extern "C" [[noreturn]] auto Reset_Handler() -> void;

extern "C" int main(void);

// Generator for weak (default) interrupt handlers
// A unique handler is defined for each interrupt so that it's easy to debug
// deadlocks if an unexpected interrupt is caught.
#define DEFINE_DEFAULT_ISR(name) \
    extern "C" \
    __attribute__((interrupt)) \
    __attribute__((weak)) \
    __attribute__((noreturn)) \
    void name() { \
        while(true); \
    }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wattributes"
DEFINE_DEFAULT_ISR(defaultISR)
DEFINE_DEFAULT_ISR(NMI_Handler)
DEFINE_DEFAULT_ISR(HardFault_Handler)
DEFINE_DEFAULT_ISR(MemManage_Handler)
DEFINE_DEFAULT_ISR(BusFault_Handler)
DEFINE_DEFAULT_ISR(UsageFault_Handler)
DEFINE_DEFAULT_ISR(SVCall_Handler)
DEFINE_DEFAULT_ISR(Debug_Monitor_Handler)
DEFINE_DEFAULT_ISR(PendSV_Handler)
DEFINE_DEFAULT_ISR(SysTick_Handler)
DEFINE_DEFAULT_ISR(USART1_IRQHandler)
DEFINE_DEFAULT_ISR(WWDG_Handler)
DEFINE_DEFAULT_ISR(PVD_Handler)
DEFINE_DEFAULT_ISR(TAMP_STAMP_Handler)
DEFINE_DEFAULT_ISR(RTC_WKUP_Handler)
DEFINE_DEFAULT_ISR(FLASH_Handler)
DEFINE_DEFAULT_ISR(RCC_Handler)
DEFINE_DEFAULT_ISR(EXTI0_Handler)
DEFINE_DEFAULT_ISR(EXTI1_Handler)
DEFINE_DEFAULT_ISR(EXTI2_Handler)
DEFINE_DEFAULT_ISR(EXTI3_Handler)
DEFINE_DEFAULT_ISR(EXTI4_Handler)
DEFINE_DEFAULT_ISR(DMA1_Stream0_Handler)
DEFINE_DEFAULT_ISR(DMA1_Stream1_Handler)
DEFINE_DEFAULT_ISR(DMA1_Stream2_Handler)
DEFINE_DEFAULT_ISR(DMA1_Stream3_Handler)
DEFINE_DEFAULT_ISR(DMA1_Stream4_Handler)
DEFINE_DEFAULT_ISR(DMA1_Stream5_Handler)
DEFINE_DEFAULT_ISR(DMA1_Stream6_Handler)
DEFINE_DEFAULT_ISR(ADC_Handler)
DEFINE_DEFAULT_ISR(CAN1_TX_Handler)
DEFINE_DEFAULT_ISR(CAN1_RX0_Handler)
DEFINE_DEFAULT_ISR(CAN1_RX1_Handler)
DEFINE_DEFAULT_ISR(CAN1_SCE_Handler)
DEFINE_DEFAULT_ISR(EXTI9_5_Handler)
DEFINE_DEFAULT_ISR(TIM1_BRK_TIM9_Handler)
DEFINE_DEFAULT_ISR(TIM1_UP_TIM10_Handler)
DEFINE_DEFAULT_ISR(TIM1_TRG_COM_TIM11_Handler)
DEFINE_DEFAULT_ISR(TIM1_CC_Handler)
DEFINE_DEFAULT_ISR(TIM2_Handler)
DEFINE_DEFAULT_ISR(TIM3_Handler)
DEFINE_DEFAULT_ISR(TIM4_Handler)
DEFINE_DEFAULT_ISR(I2C1_EV_Handler)
DEFINE_DEFAULT_ISR(I2C1_ER_Handler)
DEFINE_DEFAULT_ISR(I2C2_EV_Handler)
DEFINE_DEFAULT_ISR(I2C2_ER_Handler)
DEFINE_DEFAULT_ISR(SPI1_Handler)
DEFINE_DEFAULT_ISR(SPI2_Handler)
DEFINE_DEFAULT_ISR(USART1_Handler)
DEFINE_DEFAULT_ISR(USART2_Handler)
DEFINE_DEFAULT_ISR(USART3_Handler)
DEFINE_DEFAULT_ISR(EXTI15_10_Handler)
DEFINE_DEFAULT_ISR(RTC_Alarm_Handler)
DEFINE_DEFAULT_ISR(OTG_FS_WKUP_Handler)
DEFINE_DEFAULT_ISR(TIM8_BRK_TIM12_Handler)
DEFINE_DEFAULT_ISR(TIM8_UP_TIM13_Handler)
DEFINE_DEFAULT_ISR(TIM8_TRG_COM_TIM14_Handler)
DEFINE_DEFAULT_ISR(TIM8_CC_Handler)
DEFINE_DEFAULT_ISR(DMA1_Stream7_Handler)
DEFINE_DEFAULT_ISR(FSMC_Handler)
DEFINE_DEFAULT_ISR(SDIO_Handler)
DEFINE_DEFAULT_ISR(TIM5_Handler)
DEFINE_DEFAULT_ISR(SPI3_Handler)
DEFINE_DEFAULT_ISR(UART4_Handler)
DEFINE_DEFAULT_ISR(UART5_Handler)
DEFINE_DEFAULT_ISR(TIM6_DAC_Handler)
DEFINE_DEFAULT_ISR(TIM7_Handler)
DEFINE_DEFAULT_ISR(DMA2_Stream0_Handler)
DEFINE_DEFAULT_ISR(DMA2_Stream1_Handler)
DEFINE_DEFAULT_ISR(DMA2_Stream2_Handler)
DEFINE_DEFAULT_ISR(DMA2_Stream3_Handler)
DEFINE_DEFAULT_ISR(DMA2_Stream4_Handler)
DEFINE_DEFAULT_ISR(ETH_Handler)
DEFINE_DEFAULT_ISR(ETH_WKUP_Handler)
DEFINE_DEFAULT_ISR(CAN2_TX_Handler)
DEFINE_DEFAULT_ISR(CAN2_RX0_Handler)
DEFINE_DEFAULT_ISR(CAN2_RX1_Handler)
DEFINE_DEFAULT_ISR(CAN2_SCE_Handler)
DEFINE_DEFAULT_ISR(OTG_FS_Handler)
DEFINE_DEFAULT_ISR(DMA2_Stream5_Handler)
DEFINE_DEFAULT_ISR(DMA2_Stream6_Handler)
DEFINE_DEFAULT_ISR(DMA2_Stream7_Handler)
DEFINE_DEFAULT_ISR(USART6_Handler)
DEFINE_DEFAULT_ISR(I2C3_EV_Handler)
DEFINE_DEFAULT_ISR(I2C3_ER_Handler)
DEFINE_DEFAULT_ISR(OTG_HS_EP1_OUT_Handler)
DEFINE_DEFAULT_ISR(OTG_HS_EP1_IN_Handler)
DEFINE_DEFAULT_ISR(OTG_HS_WKUP_Handler)
DEFINE_DEFAULT_ISR(OTG_HS_Handler)
DEFINE_DEFAULT_ISR(DCMI_Handler)
DEFINE_DEFAULT_ISR(CRYP_Handler)
DEFINE_DEFAULT_ISR(HASH_RNG_Handler)
DEFINE_DEFAULT_ISR(FPU_Handler)
#pragma GCC diagnostic pop

// This must be compiletime evaluated and burnt into flash. Normally it would
// be marked `constexpr`, but that disallows `reinterpret_cast`. One might
// think that `std::bit_cast` would be used, but it can't as it is not
// `constexpr` when arguments are of pointer type.
// @see Table 61 in the reference manual for STM32F407
const void* g_pfnVectors[]
__attribute__((section(".isr_vector"))) {
    // Stack Ptr initialization
    static_cast<void*>(&__StackTop),
    // Entry point
    reinterpret_cast<void*>(&Reset_Handler),
    // Exceptions
    reinterpret_cast<void*>(&NMI_Handler),
    reinterpret_cast<void*>(&HardFault_Handler),
    reinterpret_cast<void*>(&MemManage_Handler),
    reinterpret_cast<void*>(&BusFault_Handler),
    reinterpret_cast<void*>(&UsageFault_Handler),
    nullptr, /* Reserved */
    nullptr, /* Reserved */
    nullptr, /* Reserved */
    nullptr, /* Reserved */
    reinterpret_cast<void*>(&SVCall_Handler),
    reinterpret_cast<void*>(&Debug_Monitor_Handler),
    nullptr, /* Reserved */
    reinterpret_cast<void*>(&PendSV_Handler),
    reinterpret_cast<void*>(&SysTick_Handler),
    reinterpret_cast<void*>(&WWDG_Handler),
    reinterpret_cast<void*>(&PVD_Handler),
    reinterpret_cast<void*>(&TAMP_STAMP_Handler),
    reinterpret_cast<void*>(&RTC_WKUP_Handler),
    reinterpret_cast<void*>(&FLASH_Handler),
    reinterpret_cast<void*>(&RCC_Handler),
    reinterpret_cast<void*>(&EXTI0_Handler),
    reinterpret_cast<void*>(&EXTI1_Handler),
    reinterpret_cast<void*>(&EXTI2_Handler),
    reinterpret_cast<void*>(&EXTI3_Handler),
    reinterpret_cast<void*>(&EXTI4_Handler),
    reinterpret_cast<void*>(&DMA1_Stream0_Handler),
    reinterpret_cast<void*>(&DMA1_Stream1_Handler),
    reinterpret_cast<void*>(&DMA1_Stream2_Handler),
    reinterpret_cast<void*>(&DMA1_Stream3_Handler),
    reinterpret_cast<void*>(&DMA1_Stream4_Handler),
    reinterpret_cast<void*>(&DMA1_Stream5_Handler),
    reinterpret_cast<void*>(&DMA1_Stream6_Handler),
    reinterpret_cast<void*>(&ADC_Handler),
    reinterpret_cast<void*>(&CAN1_TX_Handler),
    reinterpret_cast<void*>(&CAN1_RX0_Handler),
    reinterpret_cast<void*>(&CAN1_RX1_Handler),
    reinterpret_cast<void*>(&CAN1_SCE_Handler),
    reinterpret_cast<void*>(&EXTI9_5_Handler),
    reinterpret_cast<void*>(&TIM1_BRK_TIM9_Handler),
    reinterpret_cast<void*>(&TIM1_UP_TIM10_Handler),
    reinterpret_cast<void*>(&TIM1_TRG_COM_TIM11_Handler),
    reinterpret_cast<void*>(&TIM1_CC_Handler),
    reinterpret_cast<void*>(&TIM2_Handler),
    reinterpret_cast<void*>(&TIM3_Handler),
    reinterpret_cast<void*>(&TIM4_Handler),
    reinterpret_cast<void*>(&I2C1_EV_Handler),
    reinterpret_cast<void*>(&I2C1_ER_Handler),
    reinterpret_cast<void*>(&I2C2_EV_Handler),
    reinterpret_cast<void*>(&I2C2_ER_Handler),
    reinterpret_cast<void*>(&SPI1_Handler),
    reinterpret_cast<void*>(&SPI2_Handler),
    reinterpret_cast<void*>(&USART1_Handler),
    reinterpret_cast<void*>(&USART2_Handler),
    reinterpret_cast<void*>(&USART3_Handler),
    reinterpret_cast<void*>(&EXTI15_10_Handler),
    reinterpret_cast<void*>(&RTC_Alarm_Handler),
    reinterpret_cast<void*>(&OTG_FS_WKUP_Handler),
    reinterpret_cast<void*>(&TIM8_BRK_TIM12_Handler),
    reinterpret_cast<void*>(&TIM8_UP_TIM13_Handler),
    reinterpret_cast<void*>(&TIM8_TRG_COM_TIM14_Handler),
    reinterpret_cast<void*>(&TIM8_CC_Handler),
    reinterpret_cast<void*>(&DMA1_Stream7_Handler),
    reinterpret_cast<void*>(&FSMC_Handler),
    reinterpret_cast<void*>(&SDIO_Handler),
    reinterpret_cast<void*>(&TIM5_Handler),
    reinterpret_cast<void*>(&SPI3_Handler),
    reinterpret_cast<void*>(&UART4_Handler),
    reinterpret_cast<void*>(&UART5_Handler),
    reinterpret_cast<void*>(&TIM6_DAC_Handler),
    reinterpret_cast<void*>(&TIM7_Handler),
    reinterpret_cast<void*>(&DMA2_Stream0_Handler),
    reinterpret_cast<void*>(&DMA2_Stream1_Handler),
    reinterpret_cast<void*>(&DMA2_Stream2_Handler),
    reinterpret_cast<void*>(&DMA2_Stream3_Handler),
    reinterpret_cast<void*>(&DMA2_Stream4_Handler),
    reinterpret_cast<void*>(&ETH_Handler),
    reinterpret_cast<void*>(&ETH_WKUP_Handler),
    reinterpret_cast<void*>(&CAN2_TX_Handler),
    reinterpret_cast<void*>(&CAN2_RX0_Handler),
    reinterpret_cast<void*>(&CAN2_RX1_Handler),
    reinterpret_cast<void*>(&CAN2_SCE_Handler),
    reinterpret_cast<void*>(&OTG_FS_Handler),
    reinterpret_cast<void*>(&DMA2_Stream5_Handler),
    reinterpret_cast<void*>(&DMA2_Stream6_Handler),
    reinterpret_cast<void*>(&DMA2_Stream7_Handler),
    reinterpret_cast<void*>(&USART6_Handler),
    reinterpret_cast<void*>(&I2C3_EV_Handler),
    reinterpret_cast<void*>(&I2C3_ER_Handler),
    reinterpret_cast<void*>(&OTG_HS_EP1_OUT_Handler),
    reinterpret_cast<void*>(&OTG_HS_EP1_IN_Handler),
    reinterpret_cast<void*>(&OTG_HS_WKUP_Handler),
    reinterpret_cast<void*>(&OTG_HS_Handler),
    reinterpret_cast<void*>(&DCMI_Handler),
    reinterpret_cast<void*>(&CRYP_Handler),
    reinterpret_cast<void*>(&HASH_RNG_Handler),
    reinterpret_cast<void*>(&FPU_Handler),
    // External Interrupts
};


extern "C" [[noreturn]] __attribute__((weak)) void exit(int /*ec*/)
{
    while (true) {}
}

extern "C" void __init()
{
    using call_fp_t = void(*)(void);

    for (const auto fp: std::ranges::subrange(static_cast<call_fp_t *>(__preinit_array_start),
                                         static_cast<call_fp_t *>(__preinit_array_end))) {
        fp();
    }
    for (const auto fp: std::ranges::subrange(static_cast<call_fp_t *>(__init_array_start),
                                         static_cast<call_fp_t *>(__init_array_end))) {
        fp();
    }
}

extern "C" [[noreturn]] auto Reset_Handler() -> void {
    // Initialize data section
    extern std::uint8_t __data_start__;
    extern std::uint8_t __data_end__;
    extern std::uint8_t __etext;
    std::size_t size = static_cast<size_t>(&__data_end__- &__data_start__);
    std::copy(&__etext, &__etext + size, &__data_start__);

    // Initialize bss section
    extern std::uint8_t __bss_start__;
    extern std::uint8_t __bss_end__;
    std::fill(&__bss_start__, &__bss_end__, '\0');

    // Setup clock
    SystemInit();

    // Initialize static objects by calling their constructors
    __init();

    // Jump to main
    int ec = main();

    // We don't expect main to return; jump to the abort handler
    // An alternative solution would be to reset and re-run the program. But
    // that might hide some bugs.
    exit(ec);
}

// extern "C" void _close(void) {}
// extern "C" void _lseek(void) {}
// extern "C" void _read(void) {}
// extern "C" void _write(void) {}
