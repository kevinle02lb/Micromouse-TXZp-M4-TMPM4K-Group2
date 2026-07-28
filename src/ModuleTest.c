/**
 * @file        ModuleTest.c
 * @brief       Module Testing 
 * @version     V1.0.0
 * @date        09-07-2026
 *
 * @details
 * @note
 *   File structure and Doxygen formatting assisted by AI.
 *
 * Copyright (c) [Kevin Le] 2026
 */

#include "system_TMPM4KyA.h"

#include "modules/Timebase.h"
#include "modules/Encoder.h"
#include "modules/Motor.h"
#include "modules/IrSensor.h"   
#include "drivers/uart.h"
#include "modules/Motion.h"
#include "gpio.h"

/* ==========================================================================
 *   Module Defines
 * ========================================================================== */
#define MOTOR_TEST
//#define IR_TEST
//#define MOTION_TEST


/* ==========================================================================
 *   Motor Test
 * ========================================================================== */
#ifdef MOTOR_TEST

    #define TEST_DUTY_LOW           10U
    #define TEST_DUTY_MID           20U
    #define TEST_DUTY_HIGH          30U

    #define TEST_PHASE_MS           2000U   /*!< Hold time per phase */
    #define TEST_SETTLE_MS          750U    /*!< Brake gap between phases */


    /**
     * @brief  One step of the open-loop motor sequence.
     */
    typedef struct
    {
        motor_dir_t left_dir;
        uint8_t     left_duty;
        motor_dir_t right_dir;
        uint8_t     right_duty;
        uint32_t    hold_ms;
        uint8_t     idn;                    /*!< Phase number, mirrored into test_phase */
    } motor_phase_t;

    /*  Phase plan — read the wheel, then read test_enc_* in the debugger.
    *
    *    #  Left            Right           Expect
    *    _  ____________  ____________     ________________________________________________
    *    1  FORWARD 25      STOP            ONLY left wheel turns, forward
    *    2  STOP            FORWARD 25      ONLY right wheel turns, forward
    *    3  REVERSE 25      STOP            left wheel backward, enc_left decreasing
    *    4  STOP            REVERSE 25      right wheel backward
    *    5  FORWARD 25      FORWARD 25      straight ahead, both counts rising
    *    6  BRAKE           BRAKE           hard stop (shorted), not a coast
    *    7  FORWARD 50      REVERSE 50      spin CCW in place (left turn)
    *    8  REVERSE 50      FORWARD 50      spin CW in place (right turn)
    *    9  STOP            STOP            standby
    *   10  FOWARD  100     STOP            ONLY left wheel tuns, forward
    *   11  STOP            FORWARD 100     ONLY right wheel turns, forward
    *   12  REVERSE 100     STOP            left wheel backward, enc_left decreasing
    *   13  STOP            REVERSE 100     right wheel backward
    *   14  FORWARD 100     FORWARD 100     straight ahead, both counts rising
    *   15  BRAKE           BRAKE           hard stop (shorted), not a coast
    *   16  FORWARD 100     REVERSE 100     spin CCW in place (left turn)
    *   17  REVERSE 100     FORWARD 100     spin CW in place (right turn)
    *   18  STOP            STOP            standby
    */
    static const motor_phase_t motor_test_seq[] =
    {
        /*  Low duty: identification and sign convention  */
        { FORWARD, TEST_DUTY_LOW,  STOP,    0U,             TEST_PHASE_MS,   1U },
        { STOP,    0U,             FORWARD, TEST_DUTY_LOW,  TEST_PHASE_MS,   2U },
        { REVERSE, TEST_DUTY_LOW,  STOP,    0U,             TEST_PHASE_MS,   3U },
        { STOP,    0U,             REVERSE, TEST_DUTY_LOW,  TEST_PHASE_MS,   4U },
        { FORWARD, TEST_DUTY_LOW,  FORWARD, TEST_DUTY_LOW,  TEST_PHASE_MS,   5U },
        { STOP,    0U,             STOP,    0U,             TEST_SETTLE_MS,  6U },
        { FORWARD, TEST_DUTY_MID,  REVERSE, TEST_DUTY_MID,  TEST_PHASE_MS,   7U },
        { REVERSE, TEST_DUTY_MID,  FORWARD, TEST_DUTY_MID,  TEST_PHASE_MS,   8U },
        { STOP,    0U,             STOP,    0U,             TEST_SETTLE_MS,  9U },

        /*  Full duty: saturation, current draw, encoder ceiling  */
        { FORWARD, TEST_DUTY_HIGH, STOP,    0U,             TEST_PHASE_MS,  10U },
        { STOP,    0U,             FORWARD, TEST_DUTY_HIGH, TEST_PHASE_MS,  11U },
        { REVERSE, TEST_DUTY_HIGH, STOP,    0U,             TEST_PHASE_MS,  12U },
        { STOP,    0U,             REVERSE, TEST_DUTY_HIGH, TEST_PHASE_MS,  13U },
        { FORWARD, TEST_DUTY_HIGH, FORWARD, TEST_DUTY_HIGH, TEST_PHASE_MS,  14U },
        { STOP,   0U,              STOP,    0U,             TEST_SETTLE_MS, 15U },
        { FORWARD, TEST_DUTY_HIGH, REVERSE, TEST_DUTY_HIGH, TEST_PHASE_MS,  16U },
        { REVERSE, TEST_DUTY_HIGH, FORWARD, TEST_DUTY_HIGH, TEST_PHASE_MS,  17U },
        { STOP,    0U,             STOP,    0U,             TEST_SETTLE_MS, 18U }
    };

    #define MOTOR_SEQ_LENGTH        ( sizeof(motor_test_seq) / sizeof(motor_test_seq[0]) )

    /* ==========================================================================
    *   Debug Variables
    * ========================================================================== */
    volatile uint8_t  phase_idn         = 0U;       /*!< Phase id number running currently */
    volatile int32_t  test_enc_left     = 0;        /*!< Encoder position, left  (counts) */
    volatile int32_t  test_enc_right    = 0;        /*!< Encoder position, right (counts) */
    volatile int32_t  test_cps_left     = 0;        /*!< Filtered speed, left  (counts/s) */
    volatile int32_t  test_cps_right    = 0;        /*!< Filtered speed, right (counts/s) */
    volatile int32_t  test_phase_dl     = 0;        /*!< Left  counts distance travelled during a phase */
    volatile int32_t  test_phase_dr     = 0;        /*!< Right counts distance travelled during a phase */
    volatile bool     test_done         = false;

    /**
     * @brief  Spin for ms milliseconds, servicing the encoder every 1 kHz tick.
     * @param  ms  Hold time in milliseconds.
     * @note   Blocking by design — this is a test harness, not the control loop.
     *         Encoder_Update() MUST run here or delta/speed are meaningless.
     */
    static void Test_HoldMS(uint32_t ms)
    {
        uint32_t ticks = 0U;

        while ( ticks < ms )
        {
            if(Timebase_GetAndClear())
            {
                Encoder_Update();

                test_enc_left  = Encoder_GetPosition(MOTOR_LEFT);
                test_enc_right = Encoder_GetPosition(MOTOR_RIGHT);
                test_cps_left  = Encoder_GetSpeed_CPS(MOTOR_LEFT);
                test_cps_right = Encoder_GetSpeed_CPS(MOTOR_RIGHT);
    
                ++ticks;
            }
        }
    }

    /**
     * @brief  Run the open-loop motor sequence once.
     */
    static void MotorTest_Run(void)
    {
        uint32_t idn = 0;
        int32_t start_left = 0 , start_right = 0;


        for ( idn = 0 ; idn < MOTOR_SEQ_LENGTH ; ++idn)
        {
            const motor_phase_t* phase = &motor_test_seq[idn];

            phase_idn = phase->idn;

            start_left = Encoder_GetPosition(MOTOR_LEFT);
            start_right = Encoder_GetPosition(MOTOR_RIGHT);

            Motor_Set(MOTOR_LEFT, phase->left_dir, phase->left_duty);
            Motor_Set(MOTOR_RIGHT, phase->right_dir, phase->right_duty);

            Test_HoldMS(phase->hold_ms);

            test_phase_dl = Encoder_GetPosition(MOTOR_LEFT) - start_left;
            test_phase_dr = Encoder_GetPosition(MOTOR_RIGHT) - start_right;


            /* Reset - Set up for next phase testing */
            Motor_Set(MOTOR_LEFT, STOP, 0U);
            Motor_Set(MOTOR_RIGHT, STOP, 0U);
            Test_HoldMS(TEST_SETTLE_MS);
        }

        Motor_Stop();
        test_done = true;
    }


#endif /* MOTOR_TEST */

/* ==========================================================================
 *   IR Test
 * ========================================================================== */
#ifdef IR_TEST

    #define IR_TEST_PERIOD_MS   50U   /* stream ~20 Hz */

    static void Test_HoldMS(uint32_t ms)
    {
        uint32_t ticks = 0U;
        while (ticks < ms)
        {
            if (Timebase_GetAndClear())
            {
                ++ticks;
            }
        }
    }

    /* Print one labeled channel: "FL:1234 " */
    static void IRTest_PrintChannel(const char *label, ir_channel_t ch)
    {
        UART_SendString(label);
        UART_SendString(":");
        UART_SendUint(IR_GetFiltered(ch));
        UART_SendByte(' ');
    }

    static void IRTest_Run(void)
    {
        IR_SampleAll();  

        IRTest_PrintChannel("FL", IR_FAR_LEFT);
        IRTest_PrintChannel("L",  IR_LEFT);
        IRTest_PrintChannel("R",  IR_RIGHT);
        IRTest_PrintChannel("FR", IR_FAR_RIGHT);
        UART_CRLF();

        Test_HoldMS(IR_TEST_PERIOD_MS);   /* pace the stream */
    }


#endif /* IR_TEST */


/* ==========================================================================
 *   Motion Test
 * ========================================================================== */
#ifdef MOTION_TEST

    #define MOTION_TEST_TARGET_CPS   500.0f    /* forward target */
    #define MOTION_TEST_PRINT_EVERY  20U       /* decimation: 1 kHz / 20 = 50 Hz stream */

    /* Column order — keep in sync with the MATLAB import */
    #define MOTION_TEST_HEADER   "sp,pvL,pvR,mvL,mvR"

    /**
     * @brief  Emit one telemetry row as comma-separated text (no labels).
     * @details  Columns: setpoint, PV left, PV right, MV left, MV right.
     *           PV = filtered wheel speed (CPS). MV = PID output [-100,100],
     *           truncated to int (integer resolution is fine for spotting
     *           saturation).
     */
    static void MotionTest_StreamRow(void)
    {
        UART_SendInt((int32_t)MOTION_TEST_TARGET_CPS);          /* sp  */
        UART_SendByte(',');
        UART_SendInt(Encoder_GetSpeed_CPS(MOTOR_LEFT));         /* pvL */
        UART_SendByte(',');
        UART_SendInt(Encoder_GetSpeed_CPS(MOTOR_RIGHT));        /* pvR */
        UART_SendByte(',');
        UART_SendInt((int32_t)Motion_GetOutput(MOTOR_LEFT));    /* mvL */
        UART_SendByte(',');
        UART_SendInt((int32_t)Motion_GetOutput(MOTOR_RIGHT));   /* mvR */
        UART_CRLF();
    }

    /**
     * @brief  Closed-loop speed test: command a forward target, stream
     *         SP / PV / MV so PID convergence is visible in MATLAB.
     * @note   Runs forever. If a wheel's PV races away from SP instead of
     *         toward it, the feedback sign is inverted (runaway) — kill power.
     *         Verify motor signs with MOTOR_TEST first.
     */
    static void MotionTest_Run(void)
    {
        uint32_t tick = 0U;

        UART_SendString(MOTION_TEST_HEADER);   /* one-time header; MATLAB skips it */
        UART_CRLF();

        Motion_SetMoveForwardSpeed(MOTION_TEST_TARGET_CPS);

        while (1)
        {
            if (Timebase_GetAndClear())
            {
                Encoder_Update();    /* feedback must refresh before the loop reads it */
                Motion_Update();     /* PID -> motors */

                if (++tick >= MOTION_TEST_PRINT_EVERY)
                {
                    tick = 0U;
                    MotionTest_StreamRow();
                }
            }
        }
    }
#endif /* MOTION_TEST */


/* ==========================================================================
 *   Test Init
 * ========================================================================== */

static void Test_Init(void)
{
    #ifdef MOTOR_TEST
        Motor_Init();
        Encoder_Init();
        Timebase_Init();
    #endif

    #ifdef IR_TEST
        IR_Init();
        UART_Init();
        Timebase_Init();
    #endif

    #ifdef MOTION_TEST
        Motion_Init();      /* Encoder + Motor internally initailized */
        UART_Init();
        Timebase_Init();
    #endif
}

/* ==========================================================================
 *   Main 
 * ========================================================================== */

int main(void)
{   
    Test_Init();

    #ifdef MOTOR_TEST
        MotorTest_Run();
    #endif

    #ifdef MOTION_TEST
        MotionTest_Run();
    #endif

    while(1)
    {
        /* Busy Loop */

        #ifdef IR_TEST
            IRTest_Run();
        #endif

    }

    return 0;
}