/**
 ******************************************************************************
 * @file        main.c
 * @brief       Micromouse main firmware entry point
 * @version     V1.0.0
 * @date        05-05-2026
 *
 * @details
 *   Initializes the system clock and all modules, then runs the main control
 *   loop. Every 1 kHz tick (T32A1 interval timer) runs the full pipeline in
 *   fixed order:
 *
 *     Encoder_Update()   -> read quadrature counters, filter wheel speed
 *     Odometry_Update()  -> integrate pose (x, y, heading) from encoders
 *     Motion_Update()    -> PID speed loop, drive motor PWM (ALWAYS runs)
 *     Navigator_Update() -> non-blocking FSM: plan -> turn -> drive -> pause
 *
 *   Navigator is the top-level brain: it calls FloodFill (pure planner) for
 *   the next move, then sequences the physical turn/drive without ever
 *   blocking, so the PID loop above keeps running every tick.
 *
 *   Deployment calibration (must set before real runs):
 *   - IrSensor.c : ir_cal[] table (measured ADC-to-distance points)
 *   - Odometry.h : WHEELBASE_MM (measure wheel-center to wheel-center)
 *   - Encoder    : confirm forward motion yields INCREASING position
 *   - PID.h      : tune gains for CPS-scaled error
 *
 * ___________________________________________________________________________
 * Hardware:
 *   - MCU:     TMPM4KNF10AFG (ARM Cortex-M4, 5V, 160 MHz)
 *   - Motors:  TB67H450AFNG x2 + Pololu #5211 N20 30:1
 *   - Sensors: IR emitter/receiver pairs x4
 *   - Debug:   External CMSIS-DAP w/ Level Shifter (TXB0104)
 *
 * ___________________________________________________________________________
 * Pin Assignment:
 *   ┌─────────────────────────────────────────────────────────────┐
 *   │  PORT A  │ PA3  = 	 T32A00OUTA  (Left PWM IN1)            │
 *   │          │ PA4  = 	 T32A00OUTB  (Left PWM IN2)            │
 *   ├─────────────────────────────────────────────────────────────┤
 *   │  PORT C  │ PC2  = 	 T32A30OUTA  (Right PWM IN1)           │
 *   │          │ PC3  = 	 T32A30OUTB  (Right PWM IN2)           │
 *   ├─────────────────────────────────────────────────────────────┤
 *   │  PORT N  │ PN0  = 	 ENC0A  (Left encoder A)               │
 *   │          │ PN1  = 	 ENC0B  (Left encoder B)               │
 *   ├─────────────────────────────────────────────────────────────┤
 *   │  PORT D  │ PD3  = 	 ENC2A  (Right encoder A)              │
 *   │          │ PD4  = 	 ENC2B  (Right encoder B)              │
 *   ├─────────────────────────────────────────────────────────────┤
 *   │  PORT L  │ PL0  = 	 AINA16  (Far Left IR)                 │
 *   │          │ PL1  = 	 AINA15  (Left IR)                     │
 *   ├─────────────────────────────────────────────────────────────┤
 *   │  PORT J  │ PJ0  =     AINC00  (Far Right IR)                │
 *   │          │ PJ1  = 	 AINC01  (Right IR)                    │
 *   ├─────────────────────────────────────────────────────────────┤
 *   │  PORT U  │ PU0  =     Left IR Emitter                       │
 *   │          │ PU1  =     Far Left IR Emitter                   │
 *   ├─────────────────────────────────────────────────────────────┤
 *   │  PORT G  │ PG4  =     Far Right IR Emitter                  │
 *   │          │ PG5  =     Right IR Emitter                      │
 *   └─────────────────────────────────────────────────────────────┘
 *
 * ___________________________________________________________________________
 * Aliases (for quick reference):
 *   - PA3 = IN1, PA4 = IN2  (Left motor, T32A00OUTA/B)
 *   - PC2 = IN1, PC3 = IN2  (Right motor, T32A30OUTA/B)
 *   - PN0 = ENC0A, PN1 = ENC0B  (Left encoder)
 *   - PD3 = ENC2A, PD4 = ENC2B  (Right encoder)
 *   - PL0 = AINA16 (Far Left IR), PL1 = AINA15 (Left IR)
 *   - PJ1 = AINC01 (Right IR), PJ0 = AINC00 (Far Right IR)
 *   - PU1 = Far Left IR Emitter, PU0 = Left IR Emitter
 *   - PG5 = Right IR Emitter, PG4 = Far Right IR Emitter
 *
 * ___________________________________________________________________________
 * Notes:
 *   - MCU must be powered at 5V for ADC to function.
 *   - See TMPM4KNF10AFG datasheet section 7.2.
 *
 * ___________________________________________________________________________
 * References:
 *   - Product Info:  https://toshiba.semicon-storage.com/info/TXZP-PINFO-M4K(2)_en_20231225.pdf?did=70854
 *   - Datasheet:     https://toshiba.semicon-storage.com/info/TMPM4KNF10AFG_datasheet_en_20250516.pdf?did=155921&prodName=TMPM4KNF10AFG
 *
 * Copyright (c) [Kevin Le] 2026
 ******************************************************************************
 */


/* Inclusions */
#include "TMPM4KyA.h"
#include "system_TMPM4KyA.h"
#include "modules/Timebase.h"    // 1 kHz tick flag (T32A1)
#include "modules/Encoder.h"     // quadrature speed + position
#include "modules/Odometry.h"    // pose estimate (x, y, heading)
#include "modules/Motion.h"      // PID speed loop + motor drive
#include "modules/IrSensor.h"    // IR wall sensing (ADC + DMA)
#include "modules/FloodFill.h"   // pure maze planner (BFS)
#include "modules/Navigator.h"   // top-level motion sequencer FSM
#include "drivers/uart.h"		 // uart for debugging


void ModuleInit(void);




int main(void)
{
	SystemInit();								/* Initialize system clock and peripherals */
	ModuleInit();
	

	/* Control loop: fires once per 1 kHz tick. Order matters — each stage
	 * consumes the output of the one above it within the same tick.       */
	while (1)
	{
		if (Timebase_GetAndClear())     // true once per 1 ms tick
		{
			Encoder_Update();           // 1. read encoders, filter speed
			Odometry_Update();          // 2. update pose from encoders
			Motion_Update();            // 3. PID -> motor PWM (never blocks)
			Navigator_Update();         // 4. plan/turn/drive FSM step
		}
	}

	return 0;
}



/**
 * @brief  Initialize all modules in dependency order.
 * @note   Call once after SystemInit().
 *
 *   Each driver configures its own GPIO port internally, so no separate
 *   GPIO_Init() pass is needed. SysTick needs no init (stateless).
 *
 *   Order:
 *   1. Motion   -> Encoder + Motor (PWM T32A0/3, encoder ports) up first
 *   2. Odometry -> caches encoder position, so must follow Motion
 *   3. IR       -> DMAC + ADC + emitter ports; ADC uses SysTick_us (stateless)
 *   4. FloodFill-> planner; samples IR on first Plan(), so IR must precede it
 *   5. Navigator-> depends on FloodFill + Motion
 *   6. Timebase -> starts 1 kHz tick LAST
 */
void ModuleInit(void)
{
    Motion_Init();      // -> Encoder_Init() + Motor_Init()
    Odometry_Init();    // reads Encoder position
    IR_Init();          // -> DMAC_Init() + ADC_Init() + emitter ports
    FloodFill_Init();
    Navigator_Init();
    Timebase_Init();    // start tick last
}