#include "stm32f767xx.h"
#include "PLL.h"

void init_PLL()
{
    RCC->CR |= RCC_CR_HSEBYP; // Clock control register, Enable bypass, the clock comes from the SWD
    RCC->CR |= RCC_CR_HSEON;  // Clock control register, Enable High Speed External Clock
    while(!(RCC->CR & RCC_CR_HSERDY)); // Clock control register, wait until the HSE is ready

    RCC->APB1ENR |= RCC_APB1ENR_PWREN; // Enable the peripheral clock

    FLASH->ACR |= FLASH_ACR_LATENCY_7WS; // Flash latency has to be set

    // SYSCLK prescaler
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;  // ABH prescaler
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2; // APB1 prescaler
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV1; // APB2 prescaler

    RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV1;

    // PLL Source Mux Prescaler
    #define PLL_M 4
    #define PLL_N 96
    #define PLL_P 0 // means prescaler 2
    RCC->PLLCFGR = (PLL_M << 0) | (PLL_N << 6) | (PLL_P << 16) | RCC_PLLCFGR_PLLSRC_HSE;

    // Enable the PLL
    RCC->CR |= RCC_CR_PLLON;
    while(!(RCC->CR & RCC_CR_PLLRDY)); // check the ready flag of the PLL

    // System Clock Mux
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while((RCC->CFGR & RCC_CFGR_SWS) != (RCC_CFGR_SWS_PLL));
}
