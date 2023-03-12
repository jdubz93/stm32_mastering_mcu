/*
 * Find the base address of SPI1:
 * Control Register 1
 * control register 2
 * status register
 * data register
 * CRC polynomial register
 * RX CRC register
 * TX CRC register
 * I2S configuration register
 * I2S prescaler register
 */

#define SPI1_BASE_ADDR 0x40013000U

/* Offsets */
#define SPI1_CR1_OFFSET     0x00U // Control Register 1
#define SPI1_CR2_OFFSET     0x04U // Control Register 2
#define SPI1_SR_OFFSET      0x08U // Status Register
#define SPI1_DR_OFFSET      0x0CU // Data Register
#define SPI1_CRCPR_OFFSET   0x10U // CRC Polynomial Register
#define SPI1_RXCRCR_OFFSET  0x14U // RX CRC Register
#define SPI1_TXCRCR_OFFSET  0x18U // TX CRC Register
#define SPI1_I2SCFGR_OFFSET 0x1CU // I2S Configuration Register
#define SPI1_I2SPR_OFFSET   0x20U // I2S Prescaler Register

/* Addresses */
#define SPI1_CR1_ADDR     (SPI1_BASE_ADDR + SPI1_CR1_OFFSET)
#define SPI1_CR2_ADDR     (SPI1_BASE_ADDR + SPI1_CR2_OFFSET)
#define SPI1_SR_ADDR      (SPI1_BASE_ADDR + SPI1_SR_OFFSET)
#define SPI1_DR_ADDR      (SPI1_BASE_ADDR + SPI1_DR_OFFSET)
#define SPI1_CRCPR_ADDR   (SPI1_BASE_ADDR + SPI1_CRCPR_OFFSET)
#define SPI1_RXCRCR_ADDR  (SPI1_BASE_ADDR + SPI1_RXCRCR_OFFSET)
#define SPI1_TXCRCR_ADDR  (SPI1_BASE_ADDR + SPI1_TXCRCR_OFFSET)
#define SPI1_I2SCFGR_ADDR (SPI1_BASE_ADDR + SPI1_I2SCFGR_OFFSET)
#define SPI1_I2SPR_ADDR   (SPI1_BASE_ADDR + SPI1_I2SPR_OFFSET)

