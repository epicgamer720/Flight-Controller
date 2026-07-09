/* ============================================================
 * dfu.c — enter the STM32F7 USB DFU bootloader (no SWD on board,
 * CLAUDE.md §5.1). Uses the magic-marker + system-reset pattern:
 * the actual jump happens in SystemInit() with the chip in clean
 * reset state — a direct jump from a running system leaves enough
 * peripheral state behind to wedge the ROM's USB enumeration
 * (observed on this board). Marker lives at 0x2000FFF8: above
 * .data/.bss (~22 KB), far below the initial stack (top of RAM),
 * and RAM is retained across NVIC_SystemReset.
 * ============================================================ */
#include "app.h"

#define DFU_MAGIC_ADDR  ((volatile uint32_t *)0x2000FFF8UL)
#define DFU_MAGIC_VAL   0xDEADBEEFUL

void dfu_enter_bootloader(void)
{
    /* Detach cleanly so the host notices the CDC device leaving. */
    if (hpcd_USB_OTG_FS.Instance != NULL) {
        (void)HAL_PCD_DevDisconnect(&hpcd_USB_OTG_FS);
        HAL_Delay(20);
    }

    *DFU_MAGIC_ADDR = DFU_MAGIC_VAL;
    __DSB();
    NVIC_SystemReset();                  /* SystemInit() does the jump */
    for (;;) { }                         /* not reached */
}
