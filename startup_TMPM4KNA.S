/**
 * @file        startup_TMPM4KNA.S
 * @brief       Cortex-M4 startup for the TOSHIBA TMPM4KNA device series,
 *              GNU assembler syntax.
 *
 * @details
 *   Vector table, reset entry, and weak default handlers. Transliterated from
 *   the Toshiba armasm startup file; the table order and handler names match
 *   it exactly, so a handler defined anywhere in the project overrides the
 *   default by name alone.
 *
 *   Reset_Handler performs the work Keil delegates to __main: copy .data and
 *   the RAM-resident code region out of flash, zero .bss, then call
 *   SystemInit followed by main.
 *
 *   Symbols beginning with an underscore are supplied by the linker script.
 *
 * @note
 *   Original vector table Copyright(C) Toshiba Electronic Device Solutions
 *   Corporation 2023.
 */

    .syntax unified
    .cpu cortex-m4
    .fpu fpv4-sp-d16
    .thumb

.global __Vectors
.global Reset_Handler

/* ==========================================================================
 *   Vector Table
 * ========================================================================== */

    .section .isr_vector,"a",%progbits
    .type __Vectors, %object

__Vectors:
    .word   _estack                     /* Top of Stack */
    .word   Reset_Handler               /* Reset Handler */
    .word   NMI_Handler                 /* NMI Handler */
    .word   HardFault_Handler           /* Hard Fault Handler */
    .word   MemManage_Handler           /* MPU Fault Handler */
    .word   BusFault_Handler            /* Bus Fault Handler */
    .word   UsageFault_Handler          /* Usage Fault Handler */
    .word   0                           /* Reserved */
    .word   0                           /* Reserved */
    .word   0                           /* Reserved */
    .word   0                           /* Reserved */
    .word   SVC_Handler                 /* SVCall Handler */
    .word   DebugMon_Handler            /* Debug Monitor Handler */
    .word   0                           /* Reserved */
    .word   PendSV_Handler              /* PendSV Handler */
    .word   SysTick_Handler             /* SysTick Handler */
    .word   INT00_IRQHandler            /* 0:	Interrupt pin 00 */
    .word   INT01_IRQHandler            /* 1:	Interrupt pin 01a/01b */
    .word   INT02_IRQHandler            /* 2:	Interrupt pin 02a/02b */
    .word   INT03_IRQHandler            /* 3:	Interrupt pin 03a/03b */
    .word   INT04_IRQHandler            /* 4:	Interrupt pin 04a/04b */
    .word   INT05_IRQHandler            /* 5:	Interrupt pin 05a/05b */
    .word   INT06_IRQHandler            /* 6:	Interrupt pin 06a/06b */
    .word   INT07_IRQHandler            /* 7:	Interrupt pin 07a/07b */
    .word   INT08_IRQHandler            /* 8:	Interrupt pin 08a/08b */
    .word   INT09_IRQHandler            /* 9:	Interrupt pin 09 */
    .word   INT10_IRQHandler            /* 10:	Interrupt pin 10 */
    .word   INT11_IRQHandler            /* 11:	Interrupt pin 11a/11b */
    .word   INT12_IRQHandler            /* 12:	Interrupt pin 12 */
    .word   INT13_IRQHandler            /* 13:	Interrupt pin 13 */
    .word   INT14_IRQHandler            /* 14:	Interrupt pin 14a/14b */
    .word   INT15_IRQHandler            /* 15:	Interrupt pin 15 */
    .word   INT16_IRQHandler            /* 16:	Interrupt pin 16a/16b */
    .word   INT17_IRQHandler            /* 17:	Interrupt pin 17a/17b */
    .word   INT18_IRQHandler            /* 18:	Interrupt pin 18a/18b */
    .word   0                           /* 19:	Reserved */
    .word   0                           /* 20:	Reserved */
    .word   INT21_IRQHandler            /* 21:	Interrupt pin 21 */
    .word   INTVCN0_IRQHandler          /* 22:	A-VE+ ch0 Interrupt */
    .word   INTVCT0_IRQHandler          /* 23:	A-VE+ ch0 Task termination interrupt */
    .word   INTEMG0_IRQHandler          /* 24:	A-PMD ch0 EMG interrupt */
    .word   INTEMG1_IRQHandler          /* 25:	A-PMD ch1 EMG interrupt */
    .word   INTEMG2_IRQHandler          /* 26:	A-PMD ch2 EMG interrupt */
    .word   INTOVV0_IRQHandler          /* 27:	A-PMD ch0 OVV interrupt */
    .word   INTOVV1_IRQHandler          /* 28:	A-PMD ch1 OVV interrupt */
    .word   INTOVV2_IRQHandler          /* 29:	A-PMD ch2 OVV interrupt */
    .word   INTPWM0_IRQHandler          /* 30:	A-PMD ch0 PWM interrupt */
    .word   INTPWM1_IRQHandler          /* 31:	A-PMD ch1 PWM interrupt */
    .word   INTPWM2_IRQHandler          /* 32:	A-PMD ch2 PWM interrupt */
    .word   INTENC00_IRQHandler         /* 33:	A-ENC32 ch0 Encoder interrupt 0 */
    .word   INTENC01_IRQHandler         /* 34:	A-ENC32 ch0 Encoder interrupt 1 */
    .word   INTENC10_IRQHandler         /* 35:	A-ENC32 ch1 Encoder interrupt 0 */
    .word   INTENC11_IRQHandler         /* 36:	A-ENC32 ch1 Encoder interrupt 1 */
    .word   INTENC20_IRQHandler         /* 37:	A-ENC32 ch2 Encoder interrupt 0 */
    .word   INTENC21_IRQHandler         /* 38:	A-ENC32 ch2 Encoder interrupt 1 */
    .word   INTADAPDA_IRQHandler        /* 39:	ADC unit A PMD trigger program interrupt A */
    .word   INTADAPDB_IRQHandler        /* 40:	ADC unit A PMD trigger program interrupt B */
    .word   INTADACP0_IRQHandler        /* 41:	ADC unit A Monitor function 0 interrupt */
    .word   INTADACP1_IRQHandler        /* 42:	ADC unit A Monitor function 1 interrupt */
    .word   INTADATRG_IRQHandler        /* 43:	ADC unit A General purpose trigger program interrupt */
    .word   INTADASGL_IRQHandler        /* 44:	ADC unit A Single program interrupt */
    .word   INTADACNT_IRQHandler        /* 45:	ADC unit A Continuity program interrupt */
    .word   INTADBPDA_IRQHandler        /* 46:	ADC unit B PMD trigger program interrupt A */
    .word   INTADBPDB_IRQHandler        /* 47:	ADC unit B PMD trigger program interrupt B */
    .word   INTADBCP0_IRQHandler        /* 48:	ADC unit B Monitor function 0 interrupt */
    .word   INTADBCP1_IRQHandler        /* 49:	ADC unit B Monitor function 1 interrupt */
    .word   INTADBTRG_IRQHandler        /* 50:	ADC unit B General purpose trigger program interrupt */
    .word   INTADBSGL_IRQHandler        /* 51:	ADC unit B Single program interrupt */
    .word   INTADBCNT_IRQHandler        /* 52:	ADC unit B Continuity program interrupt */
    .word   INTADCPDA_IRQHandler        /* 53:	ADC unit C PMD trigger program interrupt A */
    .word   INTADCPDB_IRQHandler        /* 54:	ADC unit C PMD trigger program interrupt B */
    .word   INTADCCP0_IRQHandler        /* 55:	ADC unit C Monitor function 0 interrupt */
    .word   INTADCCP1_IRQHandler        /* 56:	ADC unit C Monitor function 1 interrupt */
    .word   INTADCTRG_IRQHandler        /* 57:	ADC unit C General purpose trigger program interrupt */
    .word   INTADCSGL_IRQHandler        /* 58:	ADC unit C Single program interrupt */
    .word   INTADCCNT_IRQHandler        /* 59:	ADC unit C Continuity program interrupt */
    .word   INTSC0RX_IRQHandler         /* 60:	TSPI/UART ch0 Reception interrupt */
    .word   INTSC0TX_IRQHandler         /* 61:	TSPI/UART ch0 Transmit interrupt */
    .word   INTSC0ERR_IRQHandler        /* 62:	TSPI/UART ch0 Error interrupt */
    .word   INTSC1RX_IRQHandler         /* 63:	TSPI/UART ch1 Reception interrupt */
    .word   INTSC1TX_IRQHandler         /* 64:	TSPI/UART ch1 Transmit interrupt */
    .word   INTSC1ERR_IRQHandler        /* 65:	TSPI/UART ch1 Error interrupt */
    .word   INTSC2RX_IRQHandler         /* 66:	UART ch2 Reception interrupt */
    .word   INTSC2TX_IRQHandler         /* 67:	UART ch2 Transmit interrupt */
    .word   INTSC2ERR_IRQHandler        /* 68:	UART ch2 Error interrupt */
    .word   INTSC3RX_IRQHandler         /* 69:	UART ch3 Reception interrupt */
    .word   INTSC3TX_IRQHandler         /* 70:	UART ch3 Transmit interrupt */
    .word   INTSC3ERR_IRQHandler        /* 71:	UART ch3 Error interrupt */
    .word   INTI2C0NST_IRQHandler       /* 72:	I2C ch0 Interrupt / EI2C ch0 Status interrupt */
    .word   INTI2C0ATX_IRQHandler       /* 73:	I2C ch0 Arbitration lost detection interrupt / EI2C ch0 Transmission buffer empty interrupt */
    .word   INTI2C0BRX_IRQHandler       /* 74:	I2C ch0 Bus free detection interrupt / EI2C ch0 Reception buffer full interrupt */
    .word   INTI2C0NA_IRQHandler        /* 75:	I2C ch0 NACK detection interrupt */
    .word   INTI2C1NST_IRQHandler       /* 76:	I2C ch1 Interrupt / EI2C ch1 Status interrupt */
    .word   INTI2C1ATX_IRQHandler       /* 77:	I2C ch1 Arbitration lost detection interrupt / EI2C ch1 Transmission buffer empty interrupt */
    .word   INTI2C1BRX_IRQHandler       /* 78:	I2C ch1 Bus free detection interrupt / EI2C ch1 Reception buffer full interrupt */
    .word   INTI2C1NA_IRQHandler        /* 79:	I2C ch1 NACK detection interrupt */
    .word   0                           /* 80:	Reserved */
    .word   0                           /* 81:	Reserved */
    .word   0                           /* 82:	Reserved */
    .word   INTT32A00AC_IRQHandler      /* 83:	T32A ch0 Timer A/C Compare match detection / Over flow / Under flow */
    .word   INTT32A00ACCAP0_IRQHandler  /* 84:	T32A ch0 Timer A/C Input capture 0 */
    .word   INTT32A00ACCAP1_IRQHandler  /* 85:	T32A ch0 Timer A/C Input capture 1 */
    .word   INTT32A00B_IRQHandler       /* 86:	T32A ch0 Timer B Compare match detection / Over flow / Under flow */
    .word   INTT32A00BCAP0_IRQHandler   /* 87:	T32A ch0 Timer B Input capture 0 */
    .word   INTT32A00BCAP1_IRQHandler   /* 88:	T32A ch0 Timer B Input capture 1 */
    .word   INTT32A01AC_IRQHandler      /* 89:	T32A ch1 Timer A/C Compare match detection / Over flow / Under flow */
    .word   INTT32A01ACCAP0_IRQHandler  /* 90:	T32A ch1 Timer A/C Input capture 0 */
    .word   INTT32A01ACCAP1_IRQHandler  /* 91:	T32A ch1 Timer A/C Input capture 1 */
    .word   INTT32A01B_IRQHandler       /* 92:	T32A ch1 Timer B Compare match detection / Over flow / Under flow */
    .word   INTT32A01BCAP0_IRQHandler   /* 93:	T32A ch1 Timer B Input capture 0 */
    .word   INTT32A01BCAP1_IRQHandler   /* 94:	T32A ch1 Timer B Input capture 1 */
    .word   INTT32A02AC_IRQHandler      /* 95:	T32A ch2 Timer A/C Compare match detection / Over flow / Under flow */
    .word   INTT32A02ACCAP0_IRQHandler  /* 96:	T32A ch2 Timer A/C Input capture 0 */
    .word   INTT32A02ACCAP1_IRQHandler  /* 97:	T32A ch2 Timer A/C Input capture 1 */
    .word   INTT32A02B_IRQHandler       /* 98:	T32A ch2 Timer B Compare match detection / Over flow / Under flow */
    .word   INTT32A02BCAP0_IRQHandler   /* 99:	T32A ch2 Timer B Input capture 0 */
    .word   INTT32A02BCAP1_IRQHandler   /* 100:	T32A ch2 Timer B Input capture 1 */
    .word   INTT32A03AC_IRQHandler      /* 101:	T32A ch3 Timer A/C Compare match detection / Over flow / Under flow */
    .word   INTT32A03ACCAP0_IRQHandler  /* 102:	T32A ch3 Timer A/C Input capture 0 */
    .word   INTT32A03ACCAP1_IRQHandler  /* 103:	T32A ch3 Timer A/C Input capture 1 */
    .word   INTT32A03B_IRQHandler       /* 104:	T32A ch3 Timer B Compare match detection / Over flow / Under flow */
    .word   INTT32A03BCAP0_IRQHandler   /* 105:	T32A ch3 Timer B Input capture 0 */
    .word   INTT32A03BCAP1_IRQHandler   /* 106:	T32A ch3 Timer B Input capture 1 */
    .word   INTT32A04AC_IRQHandler      /* 107:	T32A ch4 Timer A/C Compare match detection / Over flow / Under flow */
    .word   INTT32A04ACCAP0_IRQHandler  /* 108:	T32A ch4 Timer A/C Input capture 0 */
    .word   INTT32A04ACCAP1_IRQHandler  /* 109:	T32A ch4 Timer A/C Input capture 1 */
    .word   INTT32A04B_IRQHandler       /* 110:	T32A ch4 Timer B Compare match detection / Over flow / Under flow */
    .word   INTT32A04BCAP0_IRQHandler   /* 111:	T32A ch4 Timer B Input capture 0 */
    .word   INTT32A04BCAP1_IRQHandler   /* 112:	T32A ch4 Timer B Input capture 1 */
    .word   INTT32A05AC_IRQHandler      /* 113:	T32A ch5 Timer A/C Compare match detection / Over flow / Under flow */
    .word   INTT32A05ACCAP0_IRQHandler  /* 114:	T32A ch5 Timer A/C Input capture 0 */
    .word   INTT32A05ACCAP1_IRQHandler  /* 115:	T32A ch5 Timer A/C Input capture 1 */
    .word   INTT32A05B_IRQHandler       /* 116:	T32A ch5 Timer B Compare match detection / Over flow / Under flow */
    .word   INTT32A05BCAP0_IRQHandler   /* 117:	T32A ch5 Timer B Input capture 0 */
    .word   INTT32A05BCAP1_IRQHandler   /* 118:	T32A ch5 Timer B Input capture 1 */
    .word   INTPARI0_IRQHandler         /* 119:	RAMP ch0 Parity error interrupt */
    .word   INTPARI1_IRQHandler         /* 120:	RAMP ch1 Parity error interrupt */
    .word   INTDMAATC_IRQHandler        /* 121:	DMAC unit A End of transfer (ch0 - 31) */
    .word   INTDMAAERR_IRQHandler       /* 122:	DMAC unit A Transfer error */
    .word   INTFLCRDY_IRQHandler        /* 123:	Code FLASH Ready interrupt */
    .word   INTFLDRDY_IRQHandler        /* 124:	Data FLASH Ready interrupt */
    .size __Vectors, . - __Vectors

/* ==========================================================================
 *   Reset Entry
 * ========================================================================== */

    .section .text.Reset_Handler
    .weak Reset_Handler
    .type Reset_Handler, %function

Reset_Handler:
    ldr   sp, =_estack

    /* Initialised data: flash image to RAM */
    ldr   r0, =_sdata
    ldr   r1, =_edata
    ldr   r2, =_sidata
    movs  r3, #0
    b     .data_check
.data_copy:
    ldr   r4, [r2, r3]
    str   r4, [r0, r3]
    adds  r3, r3, #4
.data_check:
    adds  r4, r0, r3
    cmp   r4, r1
    bcc   .data_copy

    /* RAM-resident code region, FLASH_CODE_RAM in the scatter file */
    ldr   r0, =_sramfunc
    ldr   r1, =_eramfunc
    ldr   r2, =_siramfunc
    movs  r3, #0
    b     .ramfunc_check
.ramfunc_copy:
    ldr   r4, [r2, r3]
    str   r4, [r0, r3]
    adds  r3, r3, #4
.ramfunc_check:
    adds  r4, r0, r3
    cmp   r4, r1
    bcc   .ramfunc_copy

    /* Zero-initialised data */
    ldr   r2, =_sbss
    ldr   r4, =_ebss
    movs  r3, #0
    b     .bss_check
.bss_zero:
    str   r3, [r2]
    adds  r2, r2, #4
.bss_check:
    cmp   r2, r4
    bcc   .bss_zero

    bl    SystemInit
    bl    main

.reset_trap:
    b     .reset_trap

    .size Reset_Handler, . - Reset_Handler

/* ==========================================================================
 *   Default Handler
 *
 *   Every vector below is weak and aliased here. A handler defined elsewhere
 *   in the project replaces its entry automatically; anything unhandled traps
 *   in this loop, which is visible in the debugger as a stopped core.
 * ========================================================================== */

    .section .text.Default_Handler,"ax",%progbits
    .type Default_Handler, %function

Default_Handler:
.default_trap:
    b     .default_trap

    .size Default_Handler, . - Default_Handler

    .weak   NMI_Handler
    .thumb_set NMI_Handler, Default_Handler
    .weak   HardFault_Handler
    .thumb_set HardFault_Handler, Default_Handler
    .weak   MemManage_Handler
    .thumb_set MemManage_Handler, Default_Handler
    .weak   BusFault_Handler
    .thumb_set BusFault_Handler, Default_Handler
    .weak   UsageFault_Handler
    .thumb_set UsageFault_Handler, Default_Handler
    .weak   SVC_Handler
    .thumb_set SVC_Handler, Default_Handler
    .weak   DebugMon_Handler
    .thumb_set DebugMon_Handler, Default_Handler
    .weak   PendSV_Handler
    .thumb_set PendSV_Handler, Default_Handler
    .weak   SysTick_Handler
    .thumb_set SysTick_Handler, Default_Handler
    .weak   INT00_IRQHandler
    .thumb_set INT00_IRQHandler, Default_Handler
    .weak   INT01_IRQHandler
    .thumb_set INT01_IRQHandler, Default_Handler
    .weak   INT02_IRQHandler
    .thumb_set INT02_IRQHandler, Default_Handler
    .weak   INT03_IRQHandler
    .thumb_set INT03_IRQHandler, Default_Handler
    .weak   INT04_IRQHandler
    .thumb_set INT04_IRQHandler, Default_Handler
    .weak   INT05_IRQHandler
    .thumb_set INT05_IRQHandler, Default_Handler
    .weak   INT06_IRQHandler
    .thumb_set INT06_IRQHandler, Default_Handler
    .weak   INT07_IRQHandler
    .thumb_set INT07_IRQHandler, Default_Handler
    .weak   INT08_IRQHandler
    .thumb_set INT08_IRQHandler, Default_Handler
    .weak   INT09_IRQHandler
    .thumb_set INT09_IRQHandler, Default_Handler
    .weak   INT10_IRQHandler
    .thumb_set INT10_IRQHandler, Default_Handler
    .weak   INT11_IRQHandler
    .thumb_set INT11_IRQHandler, Default_Handler
    .weak   INT12_IRQHandler
    .thumb_set INT12_IRQHandler, Default_Handler
    .weak   INT13_IRQHandler
    .thumb_set INT13_IRQHandler, Default_Handler
    .weak   INT14_IRQHandler
    .thumb_set INT14_IRQHandler, Default_Handler
    .weak   INT15_IRQHandler
    .thumb_set INT15_IRQHandler, Default_Handler
    .weak   INT16_IRQHandler
    .thumb_set INT16_IRQHandler, Default_Handler
    .weak   INT17_IRQHandler
    .thumb_set INT17_IRQHandler, Default_Handler
    .weak   INT18_IRQHandler
    .thumb_set INT18_IRQHandler, Default_Handler
    .weak   INT21_IRQHandler
    .thumb_set INT21_IRQHandler, Default_Handler
    .weak   INTVCN0_IRQHandler
    .thumb_set INTVCN0_IRQHandler, Default_Handler
    .weak   INTVCT0_IRQHandler
    .thumb_set INTVCT0_IRQHandler, Default_Handler
    .weak   INTEMG0_IRQHandler
    .thumb_set INTEMG0_IRQHandler, Default_Handler
    .weak   INTEMG1_IRQHandler
    .thumb_set INTEMG1_IRQHandler, Default_Handler
    .weak   INTEMG2_IRQHandler
    .thumb_set INTEMG2_IRQHandler, Default_Handler
    .weak   INTOVV0_IRQHandler
    .thumb_set INTOVV0_IRQHandler, Default_Handler
    .weak   INTOVV1_IRQHandler
    .thumb_set INTOVV1_IRQHandler, Default_Handler
    .weak   INTOVV2_IRQHandler
    .thumb_set INTOVV2_IRQHandler, Default_Handler
    .weak   INTPWM0_IRQHandler
    .thumb_set INTPWM0_IRQHandler, Default_Handler
    .weak   INTPWM1_IRQHandler
    .thumb_set INTPWM1_IRQHandler, Default_Handler
    .weak   INTPWM2_IRQHandler
    .thumb_set INTPWM2_IRQHandler, Default_Handler
    .weak   INTENC00_IRQHandler
    .thumb_set INTENC00_IRQHandler, Default_Handler
    .weak   INTENC01_IRQHandler
    .thumb_set INTENC01_IRQHandler, Default_Handler
    .weak   INTENC10_IRQHandler
    .thumb_set INTENC10_IRQHandler, Default_Handler
    .weak   INTENC11_IRQHandler
    .thumb_set INTENC11_IRQHandler, Default_Handler
    .weak   INTENC20_IRQHandler
    .thumb_set INTENC20_IRQHandler, Default_Handler
    .weak   INTENC21_IRQHandler
    .thumb_set INTENC21_IRQHandler, Default_Handler
    .weak   INTADAPDA_IRQHandler
    .thumb_set INTADAPDA_IRQHandler, Default_Handler
    .weak   INTADAPDB_IRQHandler
    .thumb_set INTADAPDB_IRQHandler, Default_Handler
    .weak   INTADACP0_IRQHandler
    .thumb_set INTADACP0_IRQHandler, Default_Handler
    .weak   INTADACP1_IRQHandler
    .thumb_set INTADACP1_IRQHandler, Default_Handler
    .weak   INTADATRG_IRQHandler
    .thumb_set INTADATRG_IRQHandler, Default_Handler
    .weak   INTADASGL_IRQHandler
    .thumb_set INTADASGL_IRQHandler, Default_Handler
    .weak   INTADACNT_IRQHandler
    .thumb_set INTADACNT_IRQHandler, Default_Handler
    .weak   INTADBPDA_IRQHandler
    .thumb_set INTADBPDA_IRQHandler, Default_Handler
    .weak   INTADBPDB_IRQHandler
    .thumb_set INTADBPDB_IRQHandler, Default_Handler
    .weak   INTADBCP0_IRQHandler
    .thumb_set INTADBCP0_IRQHandler, Default_Handler
    .weak   INTADBCP1_IRQHandler
    .thumb_set INTADBCP1_IRQHandler, Default_Handler
    .weak   INTADBTRG_IRQHandler
    .thumb_set INTADBTRG_IRQHandler, Default_Handler
    .weak   INTADBSGL_IRQHandler
    .thumb_set INTADBSGL_IRQHandler, Default_Handler
    .weak   INTADBCNT_IRQHandler
    .thumb_set INTADBCNT_IRQHandler, Default_Handler
    .weak   INTADCPDA_IRQHandler
    .thumb_set INTADCPDA_IRQHandler, Default_Handler
    .weak   INTADCPDB_IRQHandler
    .thumb_set INTADCPDB_IRQHandler, Default_Handler
    .weak   INTADCCP0_IRQHandler
    .thumb_set INTADCCP0_IRQHandler, Default_Handler
    .weak   INTADCCP1_IRQHandler
    .thumb_set INTADCCP1_IRQHandler, Default_Handler
    .weak   INTADCTRG_IRQHandler
    .thumb_set INTADCTRG_IRQHandler, Default_Handler
    .weak   INTADCSGL_IRQHandler
    .thumb_set INTADCSGL_IRQHandler, Default_Handler
    .weak   INTADCCNT_IRQHandler
    .thumb_set INTADCCNT_IRQHandler, Default_Handler
    .weak   INTSC0RX_IRQHandler
    .thumb_set INTSC0RX_IRQHandler, Default_Handler
    .weak   INTSC0TX_IRQHandler
    .thumb_set INTSC0TX_IRQHandler, Default_Handler
    .weak   INTSC0ERR_IRQHandler
    .thumb_set INTSC0ERR_IRQHandler, Default_Handler
    .weak   INTSC1RX_IRQHandler
    .thumb_set INTSC1RX_IRQHandler, Default_Handler
    .weak   INTSC1TX_IRQHandler
    .thumb_set INTSC1TX_IRQHandler, Default_Handler
    .weak   INTSC1ERR_IRQHandler
    .thumb_set INTSC1ERR_IRQHandler, Default_Handler
    .weak   INTSC2RX_IRQHandler
    .thumb_set INTSC2RX_IRQHandler, Default_Handler
    .weak   INTSC2TX_IRQHandler
    .thumb_set INTSC2TX_IRQHandler, Default_Handler
    .weak   INTSC2ERR_IRQHandler
    .thumb_set INTSC2ERR_IRQHandler, Default_Handler
    .weak   INTSC3RX_IRQHandler
    .thumb_set INTSC3RX_IRQHandler, Default_Handler
    .weak   INTSC3TX_IRQHandler
    .thumb_set INTSC3TX_IRQHandler, Default_Handler
    .weak   INTSC3ERR_IRQHandler
    .thumb_set INTSC3ERR_IRQHandler, Default_Handler
    .weak   INTI2C0NST_IRQHandler
    .thumb_set INTI2C0NST_IRQHandler, Default_Handler
    .weak   INTI2C0ATX_IRQHandler
    .thumb_set INTI2C0ATX_IRQHandler, Default_Handler
    .weak   INTI2C0BRX_IRQHandler
    .thumb_set INTI2C0BRX_IRQHandler, Default_Handler
    .weak   INTI2C0NA_IRQHandler
    .thumb_set INTI2C0NA_IRQHandler, Default_Handler
    .weak   INTI2C1NST_IRQHandler
    .thumb_set INTI2C1NST_IRQHandler, Default_Handler
    .weak   INTI2C1ATX_IRQHandler
    .thumb_set INTI2C1ATX_IRQHandler, Default_Handler
    .weak   INTI2C1BRX_IRQHandler
    .thumb_set INTI2C1BRX_IRQHandler, Default_Handler
    .weak   INTI2C1NA_IRQHandler
    .thumb_set INTI2C1NA_IRQHandler, Default_Handler
    .weak   INTT32A00AC_IRQHandler
    .thumb_set INTT32A00AC_IRQHandler, Default_Handler
    .weak   INTT32A00ACCAP0_IRQHandler
    .thumb_set INTT32A00ACCAP0_IRQHandler, Default_Handler
    .weak   INTT32A00ACCAP1_IRQHandler
    .thumb_set INTT32A00ACCAP1_IRQHandler, Default_Handler
    .weak   INTT32A00B_IRQHandler
    .thumb_set INTT32A00B_IRQHandler, Default_Handler
    .weak   INTT32A00BCAP0_IRQHandler
    .thumb_set INTT32A00BCAP0_IRQHandler, Default_Handler
    .weak   INTT32A00BCAP1_IRQHandler
    .thumb_set INTT32A00BCAP1_IRQHandler, Default_Handler
    .weak   INTT32A01AC_IRQHandler
    .thumb_set INTT32A01AC_IRQHandler, Default_Handler
    .weak   INTT32A01ACCAP0_IRQHandler
    .thumb_set INTT32A01ACCAP0_IRQHandler, Default_Handler
    .weak   INTT32A01ACCAP1_IRQHandler
    .thumb_set INTT32A01ACCAP1_IRQHandler, Default_Handler
    .weak   INTT32A01B_IRQHandler
    .thumb_set INTT32A01B_IRQHandler, Default_Handler
    .weak   INTT32A01BCAP0_IRQHandler
    .thumb_set INTT32A01BCAP0_IRQHandler, Default_Handler
    .weak   INTT32A01BCAP1_IRQHandler
    .thumb_set INTT32A01BCAP1_IRQHandler, Default_Handler
    .weak   INTT32A02AC_IRQHandler
    .thumb_set INTT32A02AC_IRQHandler, Default_Handler
    .weak   INTT32A02ACCAP0_IRQHandler
    .thumb_set INTT32A02ACCAP0_IRQHandler, Default_Handler
    .weak   INTT32A02ACCAP1_IRQHandler
    .thumb_set INTT32A02ACCAP1_IRQHandler, Default_Handler
    .weak   INTT32A02B_IRQHandler
    .thumb_set INTT32A02B_IRQHandler, Default_Handler
    .weak   INTT32A02BCAP0_IRQHandler
    .thumb_set INTT32A02BCAP0_IRQHandler, Default_Handler
    .weak   INTT32A02BCAP1_IRQHandler
    .thumb_set INTT32A02BCAP1_IRQHandler, Default_Handler
    .weak   INTT32A03AC_IRQHandler
    .thumb_set INTT32A03AC_IRQHandler, Default_Handler
    .weak   INTT32A03ACCAP0_IRQHandler
    .thumb_set INTT32A03ACCAP0_IRQHandler, Default_Handler
    .weak   INTT32A03ACCAP1_IRQHandler
    .thumb_set INTT32A03ACCAP1_IRQHandler, Default_Handler
    .weak   INTT32A03B_IRQHandler
    .thumb_set INTT32A03B_IRQHandler, Default_Handler
    .weak   INTT32A03BCAP0_IRQHandler
    .thumb_set INTT32A03BCAP0_IRQHandler, Default_Handler
    .weak   INTT32A03BCAP1_IRQHandler
    .thumb_set INTT32A03BCAP1_IRQHandler, Default_Handler
    .weak   INTT32A04AC_IRQHandler
    .thumb_set INTT32A04AC_IRQHandler, Default_Handler
    .weak   INTT32A04ACCAP0_IRQHandler
    .thumb_set INTT32A04ACCAP0_IRQHandler, Default_Handler
    .weak   INTT32A04ACCAP1_IRQHandler
    .thumb_set INTT32A04ACCAP1_IRQHandler, Default_Handler
    .weak   INTT32A04B_IRQHandler
    .thumb_set INTT32A04B_IRQHandler, Default_Handler
    .weak   INTT32A04BCAP0_IRQHandler
    .thumb_set INTT32A04BCAP0_IRQHandler, Default_Handler
    .weak   INTT32A04BCAP1_IRQHandler
    .thumb_set INTT32A04BCAP1_IRQHandler, Default_Handler
    .weak   INTT32A05AC_IRQHandler
    .thumb_set INTT32A05AC_IRQHandler, Default_Handler
    .weak   INTT32A05ACCAP0_IRQHandler
    .thumb_set INTT32A05ACCAP0_IRQHandler, Default_Handler
    .weak   INTT32A05ACCAP1_IRQHandler
    .thumb_set INTT32A05ACCAP1_IRQHandler, Default_Handler
    .weak   INTT32A05B_IRQHandler
    .thumb_set INTT32A05B_IRQHandler, Default_Handler
    .weak   INTT32A05BCAP0_IRQHandler
    .thumb_set INTT32A05BCAP0_IRQHandler, Default_Handler
    .weak   INTT32A05BCAP1_IRQHandler
    .thumb_set INTT32A05BCAP1_IRQHandler, Default_Handler
    .weak   INTPARI0_IRQHandler
    .thumb_set INTPARI0_IRQHandler, Default_Handler
    .weak   INTPARI1_IRQHandler
    .thumb_set INTPARI1_IRQHandler, Default_Handler
    .weak   INTDMAATC_IRQHandler
    .thumb_set INTDMAATC_IRQHandler, Default_Handler
    .weak   INTDMAAERR_IRQHandler
    .thumb_set INTDMAAERR_IRQHandler, Default_Handler
    .weak   INTFLCRDY_IRQHandler
    .thumb_set INTFLCRDY_IRQHandler, Default_Handler
    .weak   INTFLDRDY_IRQHandler
    .thumb_set INTFLDRDY_IRQHandler, Default_Handler
