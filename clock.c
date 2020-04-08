/****************************************************************************************************
 *                                     Clock File Version 1.00                                      *
 *--------------------------------------------------------------------------------------------------*
 *                                                                                                  *
 ***************************************************************************************************/

#include <stdio.h>
#include <stdbool.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include "clock.h"
#include "systick.h"

#define RCC_I2C123CLKSOURCE_D2PCLK1      (0x00000000U)
#define RCC_I2C123CLKSOURCE_PLL3         RCC_D2CCIP2R_I2C123SEL_0
#define RCC_I2C123CLKSOURCE_HSI          RCC_D2CCIP2R_I2C123SEL_1
#define RCC_I2C123CLKSOURCE_CSI         (RCC_D2CCIP2R_I2C123SEL_0 | RCC_D2CCIP2R_I2C123SEL_1)

#define RCC_D2CCIP2R_IIC123SEL_MASK 0x03
#define RCC_D2CCIP2R_IIC123SEL_SHIFT 13

void rcc_set_iic123_clksel(uint8_t clksel);

/****************************************************************************************************
 * This routine sets the clock source for the IIC123 modules.                                       *
 ***************************************************************************************************/
void rcc_set_iic123_clksel(uint8_t clksel)
{
	RCC_D2CCIP2R &= ~(RCC_D2CCIP2R_IIC123SEL_MASK << RCC_D2CCIP2R_IIC123SEL_SHIFT);
	RCC_D2CCIP2R |= clksel << RCC_D2CCIP2R_IIC123SEL_SHIFT;
}

/****************************************************************************************************
 * This routine initalizes the system clock to run at 480mHz and initializes the systick system.    *
 ***************************************************************************************************/
void CLOCKInit(void)
{
    // define configuration for 25mHz HSE clock to run at 480mHz
    static const struct rcc_pll_config pll_config = 
    {
        .sysclock_source  = RCC_PLL,
        .pll_source       = RCC_PLLCKSELR_PLLSRC_HSE,
        .hse_frequency    = 25000000UL, /*set to 25mHz*/
        .pll1 = 
        {
            .divm = 5U,     // 5MHz PLL1 base clock pre-multiplier, cancels post div2.
            .divn = 192U,   // 192 X 5 ==> 960MHz for PLL1 VCO
            .divp = 2U,     // PLL1P post-divider gives 480MHz output.
            .divq = 12U,    // PLL1Q post-divider gives 80MHz output for FDCAN.
                            // PLL1R are disabled for now.
        },
        .pll2 = {},
        .pll3 = {},
        // Set CPU to PLL1P output at 480MHz.
        .core_pre  = RCC_D1CFGR_D1CPRE_BYP,
        .hpre = RCC_D1CFGR_D1HPRE_DIV2,   // Constrain HCLK below 240MHz by dividing core clock by 2.
        .ppre1 = RCC_D2CFGR_D2PPRE_DIV2,  // Constrain APB1 below 120MHz by dividing HCLK3 by 2.
        .ppre2 = RCC_D2CFGR_D2PPRE_DIV2,  // Constrain APB2 below 120MHz by dividing HCLK3 by 2.
        .ppre3 = RCC_D1CFGR_D1PPRE_DIV2,  // Constrain APB3 below 120MHz by dividing HCLK3 by 2.
        .ppre4 = RCC_D3CFGR_D3PPRE_DIV2,  // Constrain APB4 below 120MHz by dividing HCLK3 by 2.
        .flash_waitstates = 4,
        .voltage_scale = PWR_VOS_SCALE_0, // Highest setting, should support 480MHz operation.
    };
    rcc_clock_setup_pll(&pll_config);   // configure the PLL according to the above structure

    // Optional: Setup SPI buses to use the HSI clock @ 64MHz.
    rcc_set_spi123_clksel(RCC_D2CCIP1R_SPI123SEL_PERCK);  // PERCLK is defaulted to HSI.
    rcc_set_spi45_clksel(RCC_D2CCIP1R_SPI45SEL_HSI);

    // Setup IIC bus to use clock APB1
    rcc_set_iic123_clksel(RCC_I2C123CLKSOURCE_D2PCLK1);
    
    SYSTICKInit();  // init the systick subsystem
}

