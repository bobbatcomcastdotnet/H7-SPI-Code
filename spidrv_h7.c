/****************************************************************************************************
 *                                SPI Driver for STM32H7 - Version 1.00                             *
 *--------------------------------------------------------------------------------------------------*
 *                                                                                                  *
 ***************************************************************************************************/

/****************************************************************************************************
 * This module implements the low level drivers required by the FAT file system implemeted using    *
 * SPI.                                                                                             *
 ***************************************************************************************************/ 

#region Include files
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include "typedefs.h"
#include "ff.h"
#include "diskio.h"
#include "clock.h"
#include "spidrv_h7.h"
#endregion

#define CT_MMC		0x01		/* MMC ver 3 */
#define CT_SD1		0x02		/* SD ver 1 */
#define CT_SD2		0x04		/* SD ver 2 */
#define CT_SDC		(CT_SD1|CT_SD2)	/* SD */
#define CT_BLOCK	0x08		/* Block addressing */

SPI_MODE spiMode = MODE1;


static volatile DSTATUS Stat = STA_NOINIT;	// Physical drive status
static volatile UINT Timer1, Timer2;	    // 1kHz decrement timer stopped at zero (disk_timerproc())
static uint16_t spiStat = (STA_NOINIT<<13) | (STA_NODISK<<13);
uint16_t spiSpeed = SPI_SPEED_SLOW;
bool spi1IsInitialized = false;

 /******************************************************************************************************* 
 * This routine initializes the specified SPI port for use.                                             *
 * Wiring:                                                                                              *
 *           PA4 - SPI1_NSS  - AF5                                                                      *
 *           PA5 - SPI1_SCLK - AF5                                                                      *
 *           PA6 - SPI1_MISO - AF5                                                                      *
 *           PA7 - SPI1_MOSI - AF5                                                                      *
 *******************************************************************************************************/ 
void spiSDInit(void)
{
    if (!spi1IsInitialized)
    {
        rcc_periph_clock_enable(RCC_GPIOA); // enable port A clock
        rcc_periph_clock_enable(RCC_SPI1);  // enable SPI1 clock

        gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO4 | GPIO5 | GPIO6 | GPIO7);
        gpio_set_af(GPIOA, GPIO_AF5, GPIO4 | GPIO5 | GPIO6 | GPIO7);
        gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO5 | GPIO7);

        gpio_set(GPIOA, GPIO4);    // turn off *CS
        gpio_set(GPIOA, GPIO6);    // turn on MISO pullup

        // Set CFG1 with calculated divider and the following defaults.
        // Dafaults:
        //   - DMA Disabled
        //   - CRC Disabled
        //   - FIFO is 1-bytes
        //   - Datasize 8-bits
        // setup baud rate divisor
        
        SPI_CFG1(SPI1) = (SPI_CFG1_MBR_MCLK_DIV_256 << SPI_CFG1_MBR_SHIFT) | (SPI_CFG1_DSIZE(8) | (SPI_CFG1_FTHLV(1) << SPI_CFG1_FTHLV_SHIFT));

        // Set CFG2 with the following defaults.
        // Dafaults:
        //   - Master Mode, MSB first, Full-Duples
        //   - Software slave select control (use GPIO and user callback).
        //   - No inter-data delays.

        SPI_CFG2(SPI1)  = SPI_CFG2_MASTER | SPI_CFG2_SSOE | SPI_CFG2_SSM | SPI_CFG2_AFCNTR | (SPI_CFG2_COMM_FULLDUPLEX << SPI_CFG2_COMM_SHIFT);

        if ((spiMode == MODE2) || (spiMode == MODE3))
        { 
            SPI_CFG2(SPI1) |= SPI_CFG2_CPOL;
        }

        if ((spiMode == MODE1) || (spiMode == MODE3))
        { 
            SPI_CFG2(SPI1) |= SPI_CFG2_CPHA;
        }

        SPI_CFG2(SPI1) &= ~SPI_CFG2_LSBFRST;

        // clear the MODF bit
        SPI2S_IFCR(SPI1) |= SPI2S_IFCR_MODFC;

        SPI2S_CR1(SPI1) |= SPI2S_CR1_SSI;

        // enable SPI1
        spi_enable(SPI1);
        // update stat to remove the NOINIT flag
        spiStat &= !(STA_NOINIT<<13);
        spi1IsInitialized = true;
    }
}
/********************************************************************************************************
 * This routine initializes the SPI port for fast mode.                                                 *
 * Parameters: <none>                                                                                   *
 * Calling this routine results in SPI2 being setup as follows:                                         *
 *      SPI2 clock source is APB1 (42mHz). Using div by 2 results in a clk of 21 mHz.                   *
 *      SPI2 setup: Master mode, Module enabled, Clock data on falling edge, clk=1 on idle.             *
 *                  2-line unidirectional, rx/tx 8 data bits, no CRC, Full duplex, MSB first            *
 *******************************************************************************************************/ 
void spiSDInitFast(void)
{
//  uint32_t cr;

//    cr = SPI_CR1_BAUDRATE_FPCLK_DIV_2 | SPI_CR1_MSTR | SPI_CR1_SPE | SPI_CR1_CPHA | SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE;
//    SPI_CR2(SPI2) |= SPI_CR2_SSOE;
//    SPI_CR1(SPI2) = cr;

    SPI_CFG1(SPI1) |= (SPI_CFG1_MBR_MCLK_DIV_2 << SPI_CFG1_MBR_SHIFT);
    spiSpeed = SPI_SPEED_FAST;
}
/********************************************************************************************************
 * This routine initializes the SPI port for slow mode.                                                 *
 * Parameters: <none>                                                                                   *
 * Calling this routine results in SPI1 being setup as follows:                                         *
 *      SPI2 clock source is APB1 (50mHz). Using div by 128 results in a clk of 390.625 kHz.            *
 *      SPI2 setup: Master mode, Module enabled, Clock data on falling edge, clk=1 on idle.             *
 *                  2-line unidirectional, rx/tx 8 data bits, no CRC, Full duplex, MSB first            *
 *******************************************************************************************************/ 
void spiSDInitSlow(void)
{
//  uint32_t cr;

//    cr = SPI_CR1_BAUDRATE_FPCLK_DIV_128 | SPI_CR1_MSTR | SPI_CR1_SPE | SPI_CR1_CPHA | SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE;
//    SPI_CR2(SPI2) |= SPI_CR2_SSOE;
//    SPI_CR1(SPI2) = cr;

    SPI_CFG1(SPI1) |= (SPI_CFG1_MBR_MCLK_DIV_256 << SPI_CFG1_MBR_SHIFT);
    spiSpeed = SPI_SPEED_SLOW;
}
/****************************************************************************************************
 * Activates the CS signal (*CS = 0).                                                               *
 ***************************************************************************************************/
void spiSDSelect(void)
{
//    gpio_clear(GPIOA, GPIO4);    // turn on *CS (active lo)
    SPI2S_CR1(SPI1) &= ~(SPI2S_CR1_SSI);
}
/****************************************************************************************************
 * Deactivates the CS signal (*CS = 1).                                                             *
 ***************************************************************************************************/
void spiSDDeselect(void)
{
//    gpio_set(GPIOA, GPIO4);    // turn off *CS (active lo)
    SPI2S_CR1(SPI1) |= SPI2S_CR1_SSI;
}
 /******************************************************************************************************* 
 * This routine returns the current status for the specified SPI port.                                  *
 * Parameters:                                                                                          *
 *      spiNbr:  1 = SPI1, 2 = SPI2, 3 = SPI3                                                           *
 *                                                                                                      *
 * Returns: bit mapped status                                                                           *
 *      The valuse returned by this routine is specific to the SD card use of this spi port. That means *
 *      bits 15 to 13 are mapped as follows:                                                            *
 *                                                                                                      *
 * Bit 15 STA_PROTECT: the SD card is write protected (e.g. wp)                                         *
 * Bit 14 STA_NODISK: there is no SD card plugged in (e.g. no card detect)                              *
 * Bit 13 STA_NOINIT: the SPI port has not been initialized                                             *
 * Bits 12-09: reserved (zero)                                                                          *
 *                                                                                                      *
 * Bit 8 FRE: Frame format error                                                                        *
 *      0: No frame format error                                                                        *
 *      1: A frame format error occurred                                                                *
 *      This flag is set by hardware and cleared by software when the SPIx_SR register is read.         *
 *      Note: This flag is used when the SPI operates in TI slave mode or I2S slave mode (refer to      *
 *      Section 28.3.10).                                                                               *
 * Bit 7 BSY: Busy flag                                                                                 *
 *      0: SPI (or I2S) not busy                                                                        *
 *      1: SPI (or I2S) is busy in communication or Tx buffer is not empty                              *
 *      This flag is set and cleared by hardware.                                                       *
 *      Note: BSY flag must be used with caution: refer to Section 28.3.7: Status flags and             *
 *      Section 28.3.8: Disabling the SPI.                                                              *
 * Bit 6 OVR: Overrun flag                                                                              *
 *      0: No overrun occurred                                                                          *
 *      1: Overrun occurred                                                                             *
 *      This flag is set by hardware and reset by a software sequence. Refer to Section 28.4.8 on       *
 *      page 917 for the software sequence.                                                             *
 * Bit 5 MODF: Mode fault                                                                               *
 *      0: No mode fault occurred                                                                       *
 *      1: Mode fault occurred                                                                          *
 *      This flag is set by hardware and reset by a software sequence. Refer to Section 28.3.10 on      *
 *      page 900 for the software sequence.                                                             *
 *      Note: This bit is not used in I2S mode                                                          *
 * Bit 4 CRCERR: CRC error flag                                                                         *
 *      0: CRC value received matches the SPI_RXCRCR value                                              *
 *      1: CRC value received does not match the SPI_RXCRCR value                                       *
 *      This flag is set by hardware and cleared by software writing 0.                                 *
 *      Note: This bit is not used in I2S mode.                                                         *
 * Bit 3 UDR: Underrun flag                                                                             *
 *      0: No underrun occurred                                                                         *
 *      1: Underrun occurred                                                                            *
 *      This flag is set by hardware and reset by a software sequence. Refer to Section 28.4.8 on       *
 *      page 917 for the software sequence.                                                             *
 *      Note: This bit is not used in SPI mode.                                                         *
 * Bit 2 CHSIDE: Channel side                                                                           *
 *      0: Channel Left has to be transmitted or has been received                                      *
 *      1: Channel Right has to be transmitted or has been received                                     *
 *      Note: This bit is not used for SPI mode and is meaningless in PCM mode.                         *
 * Bit 1 TXE: Transmit buffer empty                                                                     *
 *      0: Tx buffer not empty                                                                          *
 *      1: Tx buffer empty                                                                              *
 * Bit 0 RXNE: Receive buffer not empty                                                                 *
 *      0: Rx buffer empty                                                                              *
 *      1: Rx buffer not empty                                                                          *
 *                                                                                                      *
 *******************************************************************************************************/ 
uint16_t spiSDStatus(void)
{
  uint16_t st = 0;
  uint16_t reg16;
	
    reg16 = gpio_port_read(GPIOB);
    if (reg16 & (1<<11))                    // is PB11 (CD*) low?
        { spiStat &= !(STA_NODISK<<13); }   // no: clear the NODISK status
    else
        { spiStat |= (STA_NODISK<<13); }    // yes: set the NODISK status
	st = (spiStat & 0xFE00) | (SPI2S_SR(SPI1) & 0x01FF);
    return (st);
}
 /******************************************************************************************************* 
 * Reads a byte from an open SPI port.                                                                  *
 * Assumes the port has already been selected (CS* is low).                                             *
 *******************************************************************************************************/ 
static byte spiSDReadByte(void)
{
//    spi_send(SPI2, 0xFF);     // clock the data in
//    return (spi_read(SPI2));  // read the data

	while( !(SPI2S_SR(SPI1) & SPI2S_SR_RWNE) );
    uint32_t dat = spi_read(SPI1);
    return (dat);
}
 /******************************************************************************************************* 
 * Reads a block of bytes from an open SPI port.                                                        *
 * Assumes the port has already been selected (CS* is low).                                             *
 *******************************************************************************************************/ 
byte spiSDReadBytes(uint8_t *data, uint16_t numBytes)
{
	uint16_t i = 0;
	for(i=0; i<numBytes; i++)
	{
		*(data+i) = spiSDReadByte();
	}
	return 0;
}
 /******************************************************************************************************* 
 * Sends a block of bytes to an open SPI port.                                                          *
 * Assumes the port has already been selected (CS* is low).                                             *
 *******************************************************************************************************/ 
void spiSDSendBytes(const uint8_t *data, uint16_t numBytes)
{
	uint16_t i = 0;
	for(i=0; i<numBytes; i++)
	{
//        spi_send(SPI2, *(data+i));
//		spi_xfer(SPI2, *(data+i));
        spi_write(SPI1, *(data+i));
	}
}

