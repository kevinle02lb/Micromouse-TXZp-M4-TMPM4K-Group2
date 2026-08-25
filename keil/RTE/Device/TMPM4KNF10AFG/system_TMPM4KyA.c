/**
 *******************************************************************************
 * @file    system_TMPM4KyA.c
 * @brief   CMSIS Cortex-M4 Device Peripheral Access Layer Source File for the
 *          TOSHIBA 'TMPM4Ky' Device Series  (TMPM4KNF10AFG)
 * @version V1.0.1
 *
 * Internal high-speed oscillator (10 MHz) -> PLL -> gear-selected core clock.
 * Flash read-timing API is relocated to RAM per Toshiba's design.
 *
 * DO NOT USE THIS SOFTWARE WITHOUT THE SOFTWARE LICENSE AGREEMENT.
 * Copyright(C) Toshiba Electronic Device Solutions Corporation 2022
 *******************************************************************************
 */

#include "TMPM4KyA.h"

/*==============================================================================
 *  USER CONFIGURATION
 *
 *  CLOCK_SETUP : 0 = Internal HOSC (10 MHz)   [use this]
 *                1 = External HOSC
 *
 *  USE_PLL     : 1 = PLL on  -> fc = 160 MHz  [use this]
 *                0 = PLL off -> fc = 10 MHz
 *
 *  CORE_GEAR   : divides fc down to the core clock (fsysh).
 *                Start slow and work up until the board is stable.
 *                  SYS_CG_GEAR_FC_4 -> 40 MHz   (safe starting point)
 *                  SYS_CG_GEAR_FC_2 -> 80 MHz
 *                  SYS_CG_GEAR_FC_1 -> 160 MHz  (full speed)
 *============================================================================*/
#define CLOCK_SETUP             (0U)
#define USE_PLL                 (1U)
#define CORE_GEAR               (SYS_CG_GEAR_FC_1)   /* Select */

/*-------- Clock gear values ------------------------------------------------*/
#define SYS_CG_GEAR_FC_1            (0x00000000UL)   /* fc      */
#define SYS_CG_GEAR_FC_2            (0x00000001UL)   /* fc/2    */
#define SYS_CG_GEAR_FC_4            (0x00000002UL)   /* fc/4    */
#define SYS_CG_GEAR_FC_8            (0x00000003UL)   /* fc/8    */
#define SYS_CG_GEAR_FC_16           (0x00000004UL)   /* fc/16   */

/*-------- Prescaler clock values -------------------------------------------*/
#define SYS_CG_PRCK_FC_1            (0x00000000UL)
#define SYS_CG_PRCK_FC_2            (0x00000100UL)
#define SYS_CG_PRCK_FC_4            (0x00000200UL)
#define SYS_CG_PRCK_FC_8            (0x00000300UL)
#define SYS_CG_PRCK_FC_16           (0x00000400UL)
#define SYS_CG_PRCK_FC_32           (0x00000500UL)
#define SYS_CG_PRCK_FC_64           (0x00000600UL)
#define SYS_CG_PRCK_FC_128          (0x00000700UL)
#define SYS_CG_PRCK_FC_256          (0x00000800UL)
#define SYS_CG_PRCK_FC_512          (0x00000900UL)

/*-------- Middle-speed clock select ----------------------------------------*/
#define SYS_CG_MCKSEL_1             (0x00000000UL)   /* fsysm = fsysh    */
#define SYS_CG_MCKSEL_2             (0x00000040UL)   /* fsysm = fsysh/2  */
#define SYS_CG_MCKSEL_4             (0x00000080UL)   /* fsysm = fsysh/4  */

#define SYS_CG_PLL0SEL_PLL0ON_SET   ((uint32_t)0x00000001)

/*-------- PLL multiplication values (fc target 160 MHz) --------------------*/
#define SYS_CG_6M_MUL_26_656_FPLL   (0x001C1535UL<<8U)   /* 6MHz  x 26.656  */
#define SYS_CG_8M_MUL_20_FPLL       (0x00245028UL<<8U)   /* 8MHz  x 20      */
#define SYS_CG_10M_MUL_16_FPLL      (0x002E9020UL<<8U)   /* 10MHz x 16 -> 160 */
#define SYS_CG_12M_MUL_13_312_FPLL  (0x0036DA1AUL<<8U)   /* 12MHz x 13.312  */

/*-------- Flash read-clock (wait state) codes ------------------------------*/
#define FC_KCR_KEYCODE      (0xA74A9D23UL)
#define FC_ACCR_FDLC_4      (0x00000300UL)
#define FC_ACCR_FDLC_5      (0x00000400UL)
#define FC_ACCR_FDLC_6      (0x00000500UL)
#define FC_ACCR_FCLC_1      (0x00000000UL)
#define FC_ACCR_FCLC_2      (0x00000001UL)
#define FC_ACCR_FCLC_3      (0x00000002UL)
#define FC_ACCR_FCLC_4      (0x00000003UL)
#define FC_ACCR_FCLC_5      (0x00000004UL)
#define FC_ACCR_FCLC_6      (0x00000005UL)

/*-------- Clock Generator (CG) --------------------------------------------*/
#define CG_PROTECT_UNLOCK       ((uint32_t)0xC1)

#define CG_PRCK_MASK            ((uint32_t)0x00000F00)
#define CG_MCKSEL_MASK          ((uint32_t)0x000000C0)
#define CG_PRCKST_MASK          ((uint32_t)0x0F000000)
#define CG_MCKSELPST_MASK       ((uint32_t)0xC0000000)
#define CG_MCKSELGST_MASK       ((uint32_t)0x00C00000)

#define SYSCR_GEAR_Val          (CORE_GEAR)
#define SYSCR_PRCK_Val          (SYS_CG_PRCK_FC_1)
#define SYSCR_MCKSEL_Val        (SYS_CG_MCKSEL_2)

#define STBYCR_Val              (0x00000000UL)

/*-------- Oscillator frequencies ------------------------------------------*/
#define IOSC_10M                (10000000UL)
#define IXTALH                  IOSC_10M

#define CG_PLL0SEL_FPLL         ((uint32_t)0x00000002)
#define CG_PLL0SEL_FOSC         ((uint32_t)0x00000000)
#define PLL0SEL_MASK            (0xFFFFFF00UL)

#define CG_PLL0SEL_PLL0ON_CLEAR                ((uint32_t)0xFFFFFFFE)
#define CG_PLL0SEL_PLL0SEL_SET                 ((uint32_t)0x00000002)
#define CG_PLL0SEL_PLL0SEL_CLEAR               ((uint32_t)0xFFFFFFFD)

#define CG_OSCSEL_EHOSC         ((uint32_t)0x00000100)

#define CG_OSCCR_IHOSC1EN_CLEAR              ((uint32_t)0xFFFFFFFE)
#define CG_OSCCR_IHOSC1EN_SET                ((uint32_t)0x00000001)
#define CG_OSCCR_EOSCEN_SET                  ((uint32_t)0x00000002)
#define CG_OSCCR_EOSCEN_CLEAR                ((uint32_t)0xFFFFFFFD)
#define CG_OSCCR_OSCSEL_SET                  ((uint32_t)0x00000100)

/*-------- Warm-up timing --------------------------------------------------*/
#define HZ_1M                   (1000000UL)
#define WU_TIME_EXT             (5000UL)         /* EXT warm-up 5 ms         */
#define WU_TIME_INT             (1634UL)         /* INT warm-up 163.4us x10  */

#define WUPHCR_WUPT_EXT         ((uint32_t)(((((uint64_t)WU_TIME_EXT * EXTALH / HZ_1M) - 16UL) /16UL) << 20U))
#define WUPHCR_WUPT_INT         ((uint32_t)(((((uint64_t)(WU_TIME_INT - 633UL) * (IXTALH / 10) / HZ_1M) - 41UL) / 16UL) << 20U))

#define CG_WUPHCR_WUON_START_SET             ((uint32_t)0x00000001)
#define CG_WUPHCR_WUCLK_MASK                 ((uint32_t)0x00000100)
#define CG_WUPHCR_WUPT_MASK                  ((uint32_t)0xFFF00000)

#define INIT_TIME_PLL           (100UL)          /* PLL initial time  100us  */
#define LOCKUP_TIME_PLL         (400UL)          /* PLL lock-up time  400us  */

#if (CLOCK_SETUP)
    #define CG_WUPHCR_WUCLK_SET                ((uint32_t)0x00000100)
    #define WUPHCR_INIT_PLL     ((uint32_t)(((((uint64_t)INIT_TIME_PLL * EXTALH / HZ_1M) - 16UL) /16UL) << 20U))
    #define WUPHCR_LUPT_PLL     ((uint32_t)(((((uint64_t)LOCKUP_TIME_PLL * EXTALH / HZ_1M) - 16UL) /16UL) << 20U))
    #define PLL0SEL_Ready       SYS_CG_10M_MUL_16_FPLL
#else
    #define CG_WUPHCR_WUCLK_SET                ((uint32_t)0x00000000)
    #define WUPHCR_INIT_PLL     ((uint32_t)(((((uint64_t)((INIT_TIME_PLL * 10) - 633UL) * (IXTALH / 10) / HZ_1M) - 41UL) /16UL) << 20U))
    #define WUPHCR_LUPT_PLL     ((uint32_t)(((((uint64_t)((LOCKUP_TIME_PLL * 10) - 633UL) * (IXTALH /10) / HZ_1M) - 41UL) /16UL) << 20U))
    #define PLL0SEL_Ready       SYS_CG_10M_MUL_16_FPLL
#endif

/*-------- SIWDT ------------------------------------------------------------*/
#define SIWD_SETUP              (1U)             /* 1: disable SIWD          */
#define SIWDEN_Val              (0x00000000UL)
#define SIWDCR_Val              (0x000000B1UL)

#if (USE_PLL)
    #define PLL0SEL_Val         (PLL0SEL_Ready | 0x00000003UL)
#else
    #define PLL0SEL_Val         (0x00000000UL)
#endif

#define EOSC_10M                (10000000UL)
#define EXTALH                  EOSC_10M
#define IOSC_10M_DIV2_PLLON     (160000000UL)    /* 10MHz x 32 / 2 = 160MHz  */

#define FIXED_80MHz             (80000000UL)
#define SYSCORECLOCK_ACCR       FIXED_80MHz      /* >80MHz needs 5 wait states */

/*-------- FLASH_CODE_RAM relocation symbols --------------------------------*/
#if defined ( __CC_ARM )
extern uint32_t Load$$FLASH_CODE_RAM$$Base;
extern uint32_t Image$$FLASH_CODE_RAM$$Base;
extern uint32_t Load$$FLASH_CODE_RAM$$Length;
#define FLASH_API_ROM           (uint32_t *)&Load$$FLASH_CODE_RAM$$Base
#define FLASH_API_RAM           (uint32_t *)&Image$$FLASH_CODE_RAM$$Base
#define SIZE_FLASH_API          (uint32_t)&Load$$FLASH_CODE_RAM$$Length

#elif defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6000000)   /* AC6 */
extern uint32_t Load$$FLASH_CODE_RAM$$Base;
extern uint32_t Image$$FLASH_CODE_RAM$$Base;
extern uint32_t Load$$FLASH_CODE_RAM$$Length;
#define FLASH_API_ROM           (uint32_t *)&Load$$FLASH_CODE_RAM$$Base
#define FLASH_API_RAM           (uint32_t *)&Image$$FLASH_CODE_RAM$$Base
#define SIZE_FLASH_API          (uint32_t)&Load$$FLASH_CODE_RAM$$Length

#elif defined ( __ICCARM__ )
#pragma section = "FLASH_CODE_RAM"
#pragma section = "FLASH_CODE_ROM"
#define FLASH_API_ROM           ((uint32_t *)__section_begin("FLASH_CODE_ROM"))
#define FLASH_API_RAM           ((uint32_t *)__section_begin("FLASH_CODE_RAM"))
#define SIZE_FLASH_API          ((uint32_t)__section_size("FLASH_CODE_ROM"))

#elif defined ( __SES_ARM )
extern uint32_t *__FLASH_CODE_ROM_segment_used_start__;
extern uint32_t *__FLASH_CODE_RAM_segment_used_start__;
extern uint32_t *__FLASH_CODE_RAM_segment_used_size__;
#define FLASH_API_ROM           (uint32_t *)&__FLASH_CODE_ROM_segment_used_start__
#define FLASH_API_RAM           (uint32_t *)&__FLASH_CODE_RAM_segment_used_start__
#define SIZE_FLASH_API          (uint32_t)&__FLASH_CODE_RAM_segment_used_size__
#endif

/*-------- Core clock frequency (compile-time) ------------------------------*/
#if (CLOCK_SETUP)
  #define CORE_TALH (EXTALH)
#else
  #define CORE_TALH (IXTALH)
#endif

#if ((PLL0SEL_Val & (1U<<1U)) && (PLL0SEL_Val & (1U<<0U)))
    #if (CORE_TALH == IOSC_10M)
        #define __CORE_CLK   IOSC_10M_DIV2_PLLON   /* 160 MHz */
    #else
        #define __CORE_CLK   (0U)
        #error "Core Oscillator Frequency invalid!"
    #endif
#else
    #define __CORE_CLK   (CORE_TALH)
#endif

/* Apply the clock gear to get the actual core clock (fsysh). */
#if   ((SYSCR_GEAR_Val & 7U) == 0U)
  #define __CORE_SYS   (__CORE_CLK)
#elif ((SYSCR_GEAR_Val & 7U) == 1U)
  #define __CORE_SYS   (__CORE_CLK / 2U)
#elif ((SYSCR_GEAR_Val & 7U) == 2U)
  #define __CORE_SYS   (__CORE_CLK / 4U)
#elif ((SYSCR_GEAR_Val & 7U) == 3U)
  #define __CORE_SYS   (__CORE_CLK / 8U)
#elif ((SYSCR_GEAR_Val & 7U) == 4U)
  #define __CORE_SYS   (__CORE_CLK / 16U)
#else
  #define __CORE_SYS   (0U)
#endif

uint32_t SystemCoreClock = __CORE_SYS;

/*-------- Forward declarations of RAM-resident flash routines ---------------*/
static void CopyRoutine(uint32_t *dest, uint32_t *source, uint32_t size);
static void FlashReadClockSet(uint32_t sysclock);

/*----------------------------------------------------------------------------*/
/*  Copy 32-bit data from source to dest (used to relocate flash API to RAM). */
/*----------------------------------------------------------------------------*/
static void CopyRoutine(uint32_t *dest, uint32_t *source, uint32_t size)
{
    uint32_t *dest_addr, *source_addr, tmpsize;
    uint32_t i, tmps, tmpd, mask;

    dest_addr   = dest;
    source_addr = source;

    tmpsize = size >> 2U;
    for (i = 0U; i < tmpsize; i++) {
        *dest_addr = *source_addr;
        dest_addr++;
        source_addr++;
    }
    if (size & 0x00000003U) {
        mask = 0xFFFFFF00U;
        i    = size & 0x00000003U;
        tmps = *source_addr;
        tmpd = *dest_addr;
        while (i - 1U) {
            mask = mask << 8U;
            i--;
        }
        tmps = tmps & (~mask);
        tmpd = tmpd & (mask);
        *dest_addr = tmps + tmpd;
    }
}

/*----------------------------------------------------------------------------*/
/*  SystemInit - configure clocks, flash wait states, and the PLL.            */
/*----------------------------------------------------------------------------*/
void SystemInit(void)
{
    /* Unlock CG registers (required before any CG write). */
    TSB_CG->PROTECT = CG_PROTECT_UNLOCK;

#if (SIWD_SETUP)
    TSB_SIWD0->EN = SIWDEN_Val;
    TSB_SIWD0->CR = SIWDCR_Val;
#endif

#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2));
#endif

    /* Relocate the flash read-clock routines to RAM, then set wait states.
       Runs at the reset default clock (10 MHz), before the PLL is engaged. */
    CopyRoutine(FLASH_API_RAM, FLASH_API_ROM, SIZE_FLASH_API);
    FlashReadClockSet(__CORE_SYS);

    // TSB_FC->KCR = FC_KCR_KEYCODE;
    // if (__CORE_SYS > SYSCORECLOCK_ACCR) {
    //     TSB_FC->FCACCR = (FC_ACCR_FDLC_5 | FC_ACCR_FCLC_5);
    // } else {
    //     TSB_FC->FCACCR = (FC_ACCR_FDLC_4 | FC_ACCR_FCLC_4);
    // }

#if (CLOCK_SETUP)
    /* External high-speed oscillator path. */
    TSB_CG->SYSCR |= SYSCR_GEAR_Val;
    TSB_CG->SYSCR |= SYSCR_PRCK_Val;
    TSB_CG->OSCCR |= CG_OSCCR_EOSCEN_SET;
    TSB_CG->WUPHCR = (WUPHCR_WUPT_EXT | CG_WUPHCR_WUCLK_SET);
    TSB_CG->WUPHCR = (WUPHCR_WUPT_EXT | CG_WUPHCR_WUCLK_SET | CG_WUPHCR_WUON_START_SET);
    while (TSB_CG_WUPHCR_WUEF) { ; }
    TSB_CG->OSCCR |= CG_OSCCR_OSCSEL_SET;
    while (!TSB_CG_OSCCR_OSCF) { ; }
#else
    /* Internal high-speed oscillator1 path (10 MHz). */
    TSB_CG->OSCCR |= CG_OSCCR_IHOSC1EN_SET;

    TSB_CG->WUPHCR &= ~CG_WUPHCR_WUCLK_MASK;
    TSB_CG->WUPHCR  = (TSB_CG->WUPHCR & ~CG_WUPHCR_WUPT_MASK) | WUPHCR_WUPT_INT;
    TSB_CG->WUPHCR |= CG_WUPHCR_WUON_START_SET;
    while (TSB_CG_WUPHCR_WUEF) { ; }
#endif

    /* Apply gear / prescaler / middle-speed clock and wait for it to take. */
    TSB_CG->SYSCR |= SYSCR_GEAR_Val;
    TSB_CG->SYSCR |= SYSCR_MCKSEL_Val;
    while ((TSB_CG->SYSCR & CG_MCKSELGST_MASK) !=
           (((SYSCR_GEAR_Val | SYSCR_PRCK_Val | SYSCR_MCKSEL_Val) & CG_MCKSEL_MASK) << 16U)) { ; }
    while ((TSB_CG->SYSCR & CG_MCKSELPST_MASK) !=
           (((SYSCR_GEAR_Val | SYSCR_PRCK_Val | SYSCR_MCKSEL_Val) & CG_MCKSEL_MASK) << 24U)) { ; }
    while ((TSB_CG->SYSCR & CG_PRCKST_MASK) !=
           (((SYSCR_GEAR_Val | SYSCR_PRCK_Val | SYSCR_MCKSEL_Val) & CG_PRCK_MASK) << 16U)) { ; }

#if (USE_PLL)
    /* ---- PLL startup (see reference manual 1.2.5) ---- */
    TSB_CG->WUPHCR = (WUPHCR_INIT_PLL | CG_WUPHCR_WUCLK_SET);

    /* Select fosc and stop the PLL while configuring. */
    TSB_CG->PLL0SEL &= CG_PLL0SEL_PLL0SEL_CLEAR;
    TSB_CG->PLL0SEL &= CG_PLL0SEL_PLL0ON_CLEAR;

    /* Load the multiplication value (10 MHz x 16 / 2 = 160 MHz). */
    TSB_CG->PLL0SEL = PLL0SEL_Ready;

    /* PLL initial warm-up (~100 us). */
    TSB_CG->WUPHCR = (WUPHCR_INIT_PLL | CG_WUPHCR_WUCLK_SET | CG_WUPHCR_WUON_START_SET);
    while (TSB_CG_WUPHCR_WUEF) { ; }

    /* Prepare lock-up warm-up. */
    TSB_CG->WUPHCR = (WUPHCR_LUPT_PLL | CG_WUPHCR_WUCLK_SET);

    /* Turn the PLL on. */
    TSB_CG->PLL0SEL |= SYS_CG_PLL0SEL_PLL0ON_SET;
    TSB_CG->STBYCR   = STBYCR_Val;

    /* PLL lock-up warm-up (~400 us). */
    TSB_CG->WUPHCR = (WUPHCR_LUPT_PLL | CG_WUPHCR_WUCLK_SET | CG_WUPHCR_WUON_START_SET);
    while (TSB_CG_WUPHCR_WUEF) { ; }

    /* Select the PLL as the fsys source and wait for the status. */
    TSB_CG->PLL0SEL |= CG_PLL0SEL_PLL0SEL_SET;
    while (!TSB_CG_PLL0SEL_PLL0ST) { ; }
#else
    /* PLL disabled: stay on fosc. */
    TSB_CG->PLL0SEL &= CG_PLL0SEL_PLL0SEL_CLEAR;
    TSB_CG->PLL0SEL &= CG_PLL0SEL_PLL0ON_CLEAR;
#endif

#if (CLOCK_SETUP)
    /* External path only: internal HOSC no longer needed. */
    TSB_CG->OSCCR &= CG_OSCCR_IHOSC1EN_CLEAR;
#endif
}

/*----------------------------------------------------------------------------*/
/*  SystemCoreClockUpdate - recompute SystemCoreClock from live registers.    */
/*----------------------------------------------------------------------------*/
void SystemCoreClockUpdate(void)
{
    uint32_t coreClock;
    uint32_t coreClockInput;
    uint32_t regval;

    regval    = TSB_CG->OSCCR;
    coreClock = (regval & CG_OSCSEL_EHOSC) ? EXTALH : IXTALH;

    regval = TSB_CG->PLL0SEL;
    if ((regval & CG_PLL0SEL_FPLL) && (regval & SYS_CG_PLL0SEL_PLL0ON_SET)) {
        if ((coreClock == IOSC_10M) &&
            ((TSB_CG->PLL0SEL & PLL0SEL_MASK) == SYS_CG_10M_MUL_16_FPLL)) {
            coreClockInput = IOSC_10M_DIV2_PLLON;
        } else {
            coreClockInput = 0U;
        }
    } else {
        coreClockInput = coreClock;
    }

    switch (TSB_CG->SYSCR & 7U) {
        case 0U: SystemCoreClock = coreClockInput;        break;
        case 1U: SystemCoreClock = coreClockInput / 2U;   break;
        case 2U: SystemCoreClock = coreClockInput / 4U;   break;
        case 3U: SystemCoreClock = coreClockInput / 8U;   break;
        case 4U: SystemCoreClock = coreClockInput / 16U;  break;
        default: SystemCoreClock = 0U;                    break;
    }
}

/*============================================================================*/
/*  RAM-resident flash routines.                                              */
/*  These MUST run from RAM: they rewrite the flash read-timing register,     */
/*  so the core cannot be fetching from flash while they execute.             */
/*  The AC6 branch is required - stock Toshiba code lacks it, which leaves     */
/*  FLASH_ROM empty on ARM Compiler 6.                                        */
/*============================================================================*/
#if defined ( __CC_ARM )
#pragma arm section code="FLASH_ROM"
#elif defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6000000)   /* AC6 */
__attribute__((section("FLASH_ROM"), noinline))
#elif defined ( __ICCARM__ )
#pragma location = "FLASH_ROM"
#elif defined ( __SES_ARM )
__attribute__((section("FLASH_ROM"), noinline))
#endif
static void FCRegSet(volatile uint32_t r0, volatile uint32_t r1,
                     volatile uint32_t r2, volatile uint32_t r3)
{
    __asm("str r1, [r2]");   /* write key code to KCR   */
    __asm("str r0, [r3]");   /* write timing to FCACCR  */
    return;
}

#if defined ( __CC_ARM )
#pragma arm section code="FLASH_ROM"
#elif defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6000000)   /* AC6 */
__attribute__((section("FLASH_ROM"), noinline))
#elif defined ( __ICCARM__ )
#pragma location = "FLASH_ROM"
#elif defined ( __SES_ARM )
__attribute__((section("FLASH_ROM"), noinline))
#endif
static void FlashReadClockSet(uint32_t sysclock)
{
    volatile uint32_t regval       = 0;
    volatile uint32_t kcr          = FC_KCR_KEYCODE;
    volatile uint32_t kcr_address  = (uint32_t)&TSB_FC->KCR;
    volatile uint32_t accr_address = (uint32_t)&TSB_FC->FCACCR;
    uint32_t kcr_tmp               = kcr;
    uint32_t kcr_addr_tmp          = kcr_address;
    uint32_t accr_addr_tmp         = accr_address;

    if (sysclock > SYSCORECLOCK_ACCR) {
        regval = (uint32_t)(FC_ACCR_FDLC_5 | FC_ACCR_FCLC_5);   /* 5 wait states */
        FCRegSet(regval, kcr_tmp, kcr_addr_tmp, accr_addr_tmp);
        while (TSB_FC->FCACCR != (uint32_t)(FC_ACCR_FDLC_5 | FC_ACCR_FCLC_5)) { ; }
    } else {
        regval = (uint32_t)(FC_ACCR_FDLC_4 | FC_ACCR_FCLC_4);   /* 4 wait states */
        FCRegSet(regval, kcr_tmp, kcr_addr_tmp, accr_addr_tmp);
        while (TSB_FC->FCACCR != (uint32_t)(FC_ACCR_FDLC_4 | FC_ACCR_FCLC_4)) { ; }
    }
}