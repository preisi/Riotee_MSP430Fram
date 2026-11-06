#include <msp430.h>
#include <stdbool.h>
#include <stdint.h>

#define FRAM_END (FRAM_START + FRAM_LENGTH)

/* Interrupt vector table and signatures: [IVT_START, IVT_END) */
#define IVT_START 0xFF80UL
#define IVT_END 0x10000UL

#define CODEDATA_END 0x6000UL

/* Maximum number of bytes per transfer */
#define TRANSFER_SIZE_MAX 0xFFFFUL

/* The command consists of a 20-bit address, three reserved bits (must be zero) and the R/W bit */
#define CMD_ADDR_Msk 0x0FFFFFUL
#define CMD_RSVD_Msk 0x700000UL
#define CMD_RW_Pos 23
#define CMD_RW_Msk (1UL << CMD_RW_Pos)
#define CMD_RW_READ (0UL << CMD_RW_Pos)
#define CMD_RW_WRITE (1UL << CMD_RW_Pos)

/* C2C.CS is connected to P1.4 */
#define CS_PIN BIT4
/* C2C.GPIO (ready signal to the controller) is connected to PJ.2 */
#define READY_PIN BIT2

static void spi_init() {
  /* Put to reset */
  UCA0CTLW0 |= UCSWRST;

  /* Configure for 3-pin SPI slave, keep in reset until the pins are configured */
  UCA0CTLW0 = UCSWRST | UCCKPH | UCMSB | UCSYNC | UCMODE_0;

  /* UCA0CLK on P1.5 */
  P1SEL0 &= ~BIT5;
  P1SEL1 |= BIT5;

  /* UCA0MOSI on P2.0 and UCA0MISO on P2.1 */
  P2SEL0 &= ~(BIT0 | BIT1);
  P2SEL1 |= (BIT0 | BIT1);

  /* Release reset */
  UCA0CTLW0 &= ~UCSWRST;
}

/* Written by DMA0, hence volatile */
static volatile uint8_t dma_cmd_buf[3];

static void dma_init(void) {
  /* DMA0 reads the first three command bytes into memory */
  DMA0CTL = DMASRCBYTE | DMADSTBYTE | DMADSTINCR_3 | DMADT_0;
  /* trigger 14 is USCIA0RXIFG*/
  DMACTL0 |= DMA0TSEL_14;
  DMA0SA = (uintptr_t)&UCA0RXBUF;
  DMA0DA = (uintptr_t)dma_cmd_buf;

  /* DMA1 handles 'write' requests by shoveling data into memory */
  DMA1CTL = (DMASRCBYTE | DMADSTBYTE | DMADSTINCR_3 | DMADT_0);
  DMACTL0 |= DMA1TSEL_14;
  DMA1SA = (uintptr_t)&UCA0RXBUF;

  /* DMA2 handles 'read' requests by shoveling data out of memory */
  DMA2CTL = (DMASRCBYTE | DMADSTBYTE | DMASRCINCR_3 | DMADT_0);
  DMACTL1 |= DMA2TSEL_15;
  DMA2DA = (uintptr_t)&UCA0TXBUF;
}

static int gpio_init(void) {

  /* To save energy, all non-shared GPIOs are put to a defined state */

  /* Exclude THRCTRL.H0, C2C.CS, C2C. CLK */
  P1OUT = 0x0;
  P1DIR = ~(BIT3 | BIT4 | BIT5);
  /* Configure THRCTRL.H0 as analog input to disable input buffer */
  P1SEL0 = BIT3;
  P1SEL1 = BIT3;

  /* Exclude C2C.MOSI, C2C.MISO, D2, D3, D1, D0 */
  P2OUT = 0;
  P2DIR = ~(BIT0 | BIT1 | BIT3 | BIT4 | BIT5 | BIT6);
  P2SEL0 = 0x0;
  P2SEL1 = 0x0;

  /* Exclude THRCTRL.H1, D5 */
  P3OUT = 0;
  P3DIR = ~(BIT3 | BIT6);
  /* Configure THRCTRL.H1 as analog input to disable input buffer */
  P3SEL0 = BIT3;
  P3SEL1 = BIT3;

  /* Exclude D4 */
  P4OUT = 0;
  P4DIR = ~BIT6;
  P4SEL0 = 0x0;
  P4SEL1 = 0x0;

  /* Exclude D10, D9, D8, D7, PWRGD_L, PWRGD_H  */
  P5OUT = 0;
  P5DIR = ~(BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5);
  P5SEL0 = 0x0;
  P5SEL1 = 0x0;

  /* Exclude THRCTRL.L0, SYS.SDA, SYS.SCL */
  P6OUT = 0;
  P6DIR = ~(BIT2 | BIT4 | BIT5);
  P6SEL0 = 0x0;
  P6SEL1 = 0x0;

  /* Exclude THRCTRL.L1, RTC_INT, VCAP_SENSE */
  P7OUT = 0;
  P7DIR = ~(BIT0 | BIT3 | BIT5);
  /* Configure VCAP_SENSE as analog input to disable input buffer */
  P7SEL0 = BIT5;
  P7SEL1 = BIT5;

  P8OUT = 0;
  P8DIR = 0xFF;
  P8SEL0 = 0x0;
  P8SEL1 = 0x0;

  PJOUT = 0;
  /* Exclude LED_CTRL, MAX_INT D6, Take control of C2C.GPIO */
  PJDIR = ~(BIT0 | BIT1 | BIT6);
  PJSEL0 = 0x0;
  PJSEL1 = 0x0;

  return 0;
}

__attribute__((interrupt(PORT1_VECTOR))) void PORT1_ISR(void) {
  P1IFG &= ~CS_PIN;
  LPM4_EXIT;
}

static inline bool cs_is_high(void) {
  return (P1IN & CS_PIN) != 0;
}

/*
 * Sleeps in the low-power mode given by lpm_bits until CS has the requested
 * level. If set_ready is true, the ready signal is raised before going to
 * sleep.
 *
 * Edge detection is armed and the level is checked with interrupts disabled.
 * GIE and the LPM bits are then set by a single instruction. If CS changes
 * after the level check, the port interrupt is serviced right after entering
 * the low-power mode and clears the LPM bits on the stack, so the edge cannot
 * get lost. The ready signal is raised only after edge detection is armed so
 * that the controller cannot assert CS before we are able to notice it.
 *
 * Returns with interrupts disabled.
 */
static void wait_for_cs(bool high, uint16_t lpm_bits, bool set_ready) {
  __disable_interrupt();

  if (high)
    P1IES &= ~CS_PIN;
  else
    P1IES |= CS_PIN;
  /* Writing P1IES may set P1IFG, so clear it afterwards */
  P1IFG &= ~CS_PIN;

  if (set_ready)
    PJOUT |= READY_PIN;

  while (cs_is_high() != high) {
    __bis_SR_register(GIE | lpm_bits);
    __disable_interrupt();
  }
}

static inline int setup_transfer() {
  uint16_t transfer_size;

  /*
   * Start watchdog in case the command never completes. 64 cycles of VLOCLK
   * (~9.4kHz typ.) result in a timeout of roughly 7ms.
   */
  WDTCTL = WDTPW | WDTIS__64 | WDTCNTCL | WDTSSEL__VLOCLK;

  /* Wait for DMA transfer to finish */
  while ((DMA0CTL & DMAIFG) == 0) {
  };
  DMA0CTL &= ~DMAIFG;
  /* Stop watchdog */
  WDTCTL = WDTPW | WDTHOLD | WDTCNTCL;

  /* The address is transferred lowest byte first */
  uint32_t cmd = (uint32_t)dma_cmd_buf[0] | ((uint32_t)dma_cmd_buf[1] << 8) | ((uint32_t)dma_cmd_buf[2] << 16);

  /* Reject malformed commands */
  if (cmd & CMD_RSVD_Msk)
    return -1;

  /* Compute the physical address with 32-bit arithmetic, so it cannot wrap around */
  uint32_t addr = CODEDATA_END + (cmd & CMD_ADDR_Msk);
  uint32_t limit;

  /* Protect interrupt vector table and memory beyond the end of FRAM from access */
  if (addr < IVT_START)
    limit = IVT_START;
  else if (addr < IVT_END)
    return -1;
  else if (addr < FRAM_END)
    limit = FRAM_END;
  else
    return -1;

  if (limit - addr > TRANSFER_SIZE_MAX)
    transfer_size = TRANSFER_SIZE_MAX;
  else
    transfer_size = limit - addr;

  uint32_t rw = cmd & CMD_RW_Msk;

  if (rw == CMD_RW_WRITE) {
    _data16_write_addr(&DMA1DA, addr);
    UCA0IFG &= ~UCRXIFG;
    DMA1CTL |= DMAEN;
    DMA1SZ = transfer_size;
  } else { /* rw == CMD_RW_READ */
    UCA0TXBUF = *(uint8_t *)(uintptr_t)addr;
    _data16_write_addr(&DMA2SA, addr + 1);
    DMA2CTL |= DMAEN;
    DMA2SZ = transfer_size;
  }

  return 0;
}

int main(void) {

  /* Stop watchdog */
  WDTCTL = WDTPW | WDTHOLD | WDTCNTCL;

  /* Apply the GPIO configuration */
  PM5CTL0 &= ~LOCKLPM5;

  gpio_init();

  /* Use password and add wait states */
  FRCTL0 = FRCTLPW | NWAITS_1;

  /* Unlock clock system registers */
  CSCTL0 = 0xA5 << 8;

  /* Set to 16MHz */
  CSCTL1 = (DCORSEL | DCOFSEL_4);

  /* No divider */
  CSCTL3 = DIVS_0 | DIVM_0 | DIVA_0;

  /*
   * Source ACLK from VLO. The LFXT crystal on PJ.4/PJ.5 is not enabled: it
   * would be stopped in LPM4 and could not start up within the few
   * milliseconds the watchdog runs after waking up.
   */
  CSCTL2 = SELM__DCOCLK | SELS__DCOCLK | SELA__VLOCLK;

  /* Clear pending interrupt */
  P1IFG &= ~CS_PIN;
  /* Enable interrupt */
  P1IE |= CS_PIN;

  /*
   * Wait in LPM4 until CS is high, i.e., until the controller is not in the
   * middle of a transaction. This has to wait for the level and not for a
   * rising edge: If CS is already high, the controller does not assert CS
   * before the ready signal is raised, so a rising edge would never come.
   */
  wait_for_cs(true, LPM4_bits, false);

  spi_init();
  dma_init();

  while (1) {
    /*
     * Prepare for command transfer. DMA0 is armed before going to sleep so
     * that the command bytes are captured independently of how long it takes
     * to wake up from LPM4 after CS is asserted.
     */
    DMA0SZ = 3;
    /* Clear RX interrupt */
    UCA0IFG &= ~UCRXIFG;
    /* Enable DMA0 */
    DMA0CTL |= DMAEN;

    /* Signal the controller that we're ready for a command and sleep in LPM4 until CS is asserted */
    wait_for_cs(false, LPM4_bits, true);
    /* Signal the controller that we're busy */
    PJOUT &= ~READY_PIN;
    int rc = setup_transfer();

    /*
     * Signal the controller that we're ready for a transaction and sleep in LPM0 (to still allow DMA to move data)
     * until CS is released. If the command was rejected, the ready signal stays low until CS is released, so the
     * controller can detect the error before transferring data.
     */
    wait_for_cs(true, LPM0_bits, rc == 0);
    /* Signal the controller that we're busy */
    PJOUT &= ~READY_PIN;

    /* Reset DMA channels */
    DMA0CTL &= ~DMAEN;
    DMA1CTL &= ~DMAEN;
    DMA2CTL &= ~DMAEN;

    /* Reset SPI */
    UCA0CTLW0 |= UCSWRST;
    UCA0TXBUF = 0x0;
    UCA0CTLW0 &= ~UCSWRST;
  };
}
