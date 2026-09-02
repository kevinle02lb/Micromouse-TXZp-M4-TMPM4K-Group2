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
 * @author [Kevin Le] 2026
 */

#include "system_TMPM4KyA.h"

#include "modules/Timebase.h"
#include "modules/Encoder.h"
#include "modules/Motor.h"
#include "modules/IrSensor.h"   
#include "drivers/uart.h"
#include "modules/Motion.h"
#include "modules/Odometry.h"
#include "modules/Profile.h"
#include "gpio.h"

/* ==========================================================================
 *   Module Defines
 * ========================================================================== */
//#define MOTOR_TEST
//#define IR_TEST
#define MOTION_TEST
//#define IR_EMITTER_TEST
//#define ODOM_TEST
//#define TURN_TEST
//#define DRIVE_TEST

/* ==========================================================================
 *   Shared Helper
 * ========================================================================== */

#if defined(ODOM_TEST) || defined(TURN_TEST) || defined(DRIVE_TEST)
/**
 * @brief  Send a float with one decimal place.
 * @param  value  Value to print.
 * @note   UART_SendInt emits integers only, so the sign, whole part and tenth
 *         are written separately.
 */
static void Test_SendDecimal(float value)
{
    int32_t tenths = (int32_t)(value * 10.0f);

    if (tenths < 0)
    {
        UART_SendByte('-');
        tenths = -tenths;
    }

    UART_SendInt(tenths / 10);
    UART_SendByte('.');
    UART_SendInt(tenths % 10);
}
#endif

/* ==========================================================================
 *   Motor Test
 * ========================================================================== */
#ifdef MOTOR_TEST

    #define TEST_DUTY_LOW           20U
    #define TEST_DUTY_MID           25U
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
    *    1  FORWARD L       STOP            ONLY left wheel turns, forward
    *    2  STOP            FORWARD L       ONLY right wheel turns, forward
    *    3  REVERSE L       STOP            left wheel backward, enc_left decreasing
    *    4  STOP            REVERSE L       right wheel backward
    *    5  FORWARD L       FORWARD L       straight ahead, both counts rising
    *    6  BRAKE           BRAKE           hard stop (shorted), not a coast
    *    7  FORWARD M       REVERSE M       spin CCW in place (left turn)
    *    8  REVERSE M       FORWARD M       spin CW in place (right turn)
    *    9  STOP            STOP            standby
    *   10  FOWARD  H       STOP            ONLY left wheel tuns, forward
    *   11  STOP            FORWARD H       ONLY right wheel turns, forward
    *   12  REVERSE H       STOP            left wheel backward, enc_left decreasing
    *   13  STOP            REVERSE H       right wheel backward
    *   14  FORWARD H       FORWARD H       straight ahead, both counts rising
    *   15  BRAKE           BRAKE           hard stop (shorted), not a coast
    *   16  FORWARD H       REVERSE H       spin CCW in place (left turn)
    *   17  REVERSE H       FORWARD H       spin CW in place (right turn)
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

        /*  High duty: saturation, current draw, encoder ceiling  */
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

    #define IR_TEST_FSM             /* step-based sampler instead of blocking */

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
        #ifdef IR_TEST_FSM
            if (!Timebase_GetAndClear())
                return;

            IR_SampleStep();           /* mid-cycle — nothing new to print */
            
        #else
            IR_SampleAll();
        #endif

            IRTest_PrintChannel("FL", IR_FAR_LEFT);
            IRTest_PrintChannel("L",  IR_LEFT);
            IRTest_PrintChannel("R",  IR_RIGHT);
            IRTest_PrintChannel("FR", IR_FAR_RIGHT);

            UART_CRLF();
            /* Distance in mm */
            UART_SendString("Distance: ");
            UART_SendString(" FL:");
            UART_SendUint(IR_GetDistance_mm(IR_FAR_LEFT));
            UART_SendString(" L:");
            UART_SendUint(IR_GetDistance_mm(IR_LEFT));
            UART_SendString(" R:");
            UART_SendUint(IR_GetDistance_mm(IR_RIGHT));
            UART_SendString(" FR:");
            UART_SendUint(IR_GetDistance_mm(IR_FAR_RIGHT));
            UART_CRLF();
        
            Test_HoldMS(IR_TEST_PERIOD_MS);   /* pace the stream */
        
    }



#endif /* IR_TEST */


/* ==========================================================================
 *   IR Emitter Lightshow Test  (all 4 emitters) - Soldered on LEDs (Not IR Emitters yet )
 *  
 *   Just testing the Output 
 *
 *   Emitters (physical left -> right):
 *       FL = PU1   L = PU0   R = PG5   FR = PG4
 *   Split across two ports, so this drives Port U AND Port G.
 *
 *
 *   MOTORS: define EMITTER_TEST_WITH_MOTORS below to also pivot the robot
 *           in place while the lights run (flips direction each lap). Leave
 *           it OFF for bench testing with the debugger wires attached.
 * ========================================================================== */
#ifdef IR_EMITTER_TEST

    /* Spin the robot in place while the lights run. Comment out for bench
       testing with debug wires attached. */
    //#define EMITTER_TEST_WITH_MOTORS

    /* Steady DC hold — all four emitters ON, forever. For metering the
       receiver voltage on the bench. Comment this define out to run the
       lightshow (EmitterTest_Run) instead. */
    #define EMITTER_DC_HOLD

    #define SHOW_STEP_MS   90U    /* base frame time */
    #define SHOW_GAP_MS    400U   /* dark pause between patterns */

    #ifdef EMITTER_TEST_WITH_MOTORS
        #define PIVOT_DUTY  20U   /* gentle spin; open-loop */

        /* --- PIVOT DIRECTION FIX (test-only, no Motor.c changes) ---------
         * The LEFT side is mirror-mounted on the chassis (see SIGN_LEFT in
         * Motion.c), so its motor rotates opposite the right for the same
         * command. This flag inverts the LEFT wheel's commanded direction
         * INSIDE THIS TEST ONLY so a pivot actually counter-rotates.
         * If it drives straight instead, flip it to 0.
         * ---------------------------------------------------------------- */
        #define PIVOT_LEFT_INVERT  1
    #endif

    /* Emitter bit map — bit0..bit3, physical left -> right */
    #define E_FL   0x01U   /* PU1 */
    #define E_L    0x02U   /* PU0 */
    #define E_R    0x04U   /* PG5 */
    #define E_FR   0x08U   /* PG4 */
    #define E_ALL  0x0FU

    static void Emitter_Write(uint8_t m);   /* fwd decl — defined below */

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

    static void EmitterTest_DC(void)
    {
        Emitter_Write(E_ALL);      /* FL|L|R|FR on */
        while (1)
        {
            /* held — put the meter on each receiver output */
        }
    }

    /**
     * @brief  Write all four emitters at once from a 4-bit pattern.
     * @param  m  bit0=FL(PU1) bit1=L(PU0) bit2=R(PG5) bit3=FR(PG4)
     */
    static void Emitter_Write(uint8_t m)
    {
        if (m & E_FL) { GPIO_U_SetData(Px1_MASK); } else { GPIO_U_ClrData(Px1_MASK); }
        if (m & E_L)  { GPIO_U_SetData(Px0_MASK); } else { GPIO_U_ClrData(Px0_MASK); }
        if (m & E_R)  { GPIO_G_SetData(Px5_MASK); } else { GPIO_G_ClrData(Px5_MASK); }
        if (m & E_FR) { GPIO_G_SetData(Px4_MASK); } else { GPIO_G_ClrData(Px4_MASK); }
    }

    /* ---- Patterns ---------------------------------------------------- */

    /* Cylon / Knight-Rider: one dot bounces across the row */
    static void Show_Cylon(uint8_t sweeps)
    {
        static const uint8_t seq[] = { E_FL, E_L, E_R, E_FR, E_R, E_L };
        uint8_t s, i;
        for (s = 0U; s < sweeps; ++s)
        {
            for (i = 0U; i < sizeof(seq); ++i)
            {
                Emitter_Write(seq[i]);
                Test_HoldMS(SHOW_STEP_MS);
            }
        }
    }

    /* Fill from the left, then drain from the left */
    static void Show_FillFlush(uint8_t cycles)
    {
        static const uint8_t seq[] =
        {
            E_FL, E_FL|E_L, E_FL|E_L|E_R, E_ALL,
            E_L|E_R|E_FR, E_R|E_FR, E_FR, 0U
        };
        uint8_t c, i;
        for (c = 0U; c < cycles; ++c)
        {
            for (i = 0U; i < sizeof(seq); ++i)
            {
                Emitter_Write(seq[i]);
                Test_HoldMS(SHOW_STEP_MS);
            }
        }
    }

    /* Outer pair vs inner pair — flapping wings */
    static void Show_Wings(uint8_t times)
    {
        while (times--)
        {
            Emitter_Write(E_FL | E_FR);
            Test_HoldMS(SHOW_STEP_MS * 2U);
            Emitter_Write(E_L | E_R);
            Test_HoldMS(SHOW_STEP_MS * 2U);
        }
    }

    /* 4-bit binary counter 0..15 — also verifies FL/L/R/FR mapping */
    static void Show_Count(void)
    {
        uint8_t n;
        for (n = 0U; n <= E_ALL; ++n)
        {
            Emitter_Write(n);
            Test_HoldMS(SHOW_STEP_MS * 2U);
        }
    }

    /* Random sparkle (xorshift32, no stdlib) */
    static uint8_t Show_Rand4(void)
    {
        static uint32_t rng = 0x00C0FFEEU;
        rng ^= rng << 13;
        rng ^= rng >> 17;
        rng ^= rng << 5;
        return (uint8_t)(rng & E_ALL);
    }
    static void Show_Sparkle(uint8_t frames)
    {
        while (frames--)
        {
            Emitter_Write(Show_Rand4());
            Test_HoldMS(SHOW_STEP_MS);
        }
    }

    /* All-on strobe */
    static void Show_Blink(uint8_t times)
    {
        while (times--)
        {
            Emitter_Write(E_ALL);
            Test_HoldMS(SHOW_STEP_MS * 2U);
            Emitter_Write(0U);
            Test_HoldMS(SHOW_STEP_MS * 2U);
        }
    }

    #ifdef EMITTER_TEST_WITH_MOTORS
    /**
     * @brief  Pivot in place. ccw!=0 requests counter-clockwise.
     * @note   Right-wheel direction is optionally inverted here (test-only)
     *         so a driver/wiring sign mismatch still produces a real pivot.
     */
    static void Pivot(uint8_t ccw)
    {
        motor_dir_t l = ccw ? FORWARD : REVERSE;
        motor_dir_t r = ccw ? REVERSE : FORWARD;

    #if PIVOT_LEFT_INVERT
        l = (l == FORWARD) ? REVERSE : FORWARD;   /* left is mirror-mounted */
    #endif

        Motor_Set(MOTOR_LEFT,  l, PIVOT_DUTY);
        Motor_Set(MOTOR_RIGHT, r, PIVOT_DUTY);
    }
    #endif

    /* ---- Main show --------------------------------------------------- */

    static void EmitterTest_Run(void)
    {
        uint8_t dir = 0U;

        Emitter_Write(0U);

        while (1)
        {
        #ifdef EMITTER_TEST_WITH_MOTORS
            Pivot(dir);          /* flips each lap for a back-and-forth spin */
        #endif

            Show_Cylon(3U);      Emitter_Write(0U); Test_HoldMS(SHOW_GAP_MS);
            Show_FillFlush(2U);  Emitter_Write(0U); Test_HoldMS(SHOW_GAP_MS);
            Show_Wings(6U);      Emitter_Write(0U); Test_HoldMS(SHOW_GAP_MS);
            Show_Count();        Emitter_Write(0U); Test_HoldMS(SHOW_GAP_MS);
            Show_Sparkle(40U);   Emitter_Write(0U); Test_HoldMS(SHOW_GAP_MS);
            Show_Blink(4U);      Emitter_Write(0U); Test_HoldMS(SHOW_GAP_MS);

            dir ^= 1U;
        }
    }

#endif /* IR_EMITTER_TEST */


/* ==========================================================================
 *   Motion Test
 * ========================================================================== */
#ifdef MOTION_TEST

    #define MOTION_TEST_TARGET_CPS   250.0f    /* target */
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
     * @brief  Closed-loop speed test: command a target, stream
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

        //Motion_SetMoveForwardSpeed(MOTION_TEST_TARGET_CPS);

        Motion_SetSpeed(-MOTION_TEST_TARGET_CPS, MOTION_TEST_TARGET_CPS);

        while (1)
        {
            if (Timebase_GetAndClear())
            {
                Encoder_Update();    /* feedback must refresh before the loop reads it */

                // Motor_Set(MOTOR_LEFT,  FORWARD, 30);   /* bypass PID, known-good duty */
                // Motor_Set(MOTOR_RIGHT, FORWARD, 30);
                
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
 *   Odometry Test
 *
 *   Passive. Motors stay in standby so the robot can be pushed by hand.
 *
 *   Columns: encL,encR,x,deg,diff
 *     encL   left encoder position, counts
 *     encR   right encoder position, counts
 *     x      odometry X, mm
 *     deg    odometry heading, degrees
 *     diff   (D_right - D_left), mm. The numerator of the heading equation in
 *            Odometry.c. Divided by a known physical rotation it gives
 *            WHEELBASE_MM.
 * ========================================================================== */
#ifdef ODOM_TEST

    #define ODOM_PRINT_EVERY   100U    /* 1 kHz / 100 = 10 Hz */

    static void OdomTest_Run(void)
    {
        uint32_t tick = 0U;
        int32_t  posL, posR;

        UART_SendString("encL,encR,x,deg,diff");
        UART_CRLF();

        Encoder_ResetPosition(MOTOR_LEFT);
        Encoder_ResetPosition(MOTOR_RIGHT);
        Odometry_Reset();

        while (1)
        {
            if (!Timebase_GetAndClear())
                continue;

            Encoder_Update();
            Odometry_Update();

            if (++tick < ODOM_PRINT_EVERY)
                continue;

            tick = 0U;
            posL = Encoder_GetPosition(MOTOR_LEFT);
            posR = Encoder_GetPosition(MOTOR_RIGHT);

            UART_SendInt(posL);                                          UART_SendByte(',');
            UART_SendInt(posR);                                          UART_SendByte(',');
            Test_SendDecimal(Odometry_GetX_mm());                        UART_SendByte(',');
            Test_SendDecimal(Odometry_GetHeading_deg());                 UART_SendByte(',');
            Test_SendDecimal((float)(posR - posL) * MM_PER_COUNT);
            UART_CRLF();
        }
    }

#endif /* ODOM_TEST */


/* ==========================================================================
 *   Turn Test
 *
 *   One in-place turn driven the same way the Navigator drives one: a single
 *   trapezoidal Profile segment on the arc each wheel traces.
 *
 *   A pivot of theta radians gives each wheel an arc of
 *
 *       arc = theta * WHEELBASE_MM / 2
 *
 *   Streaming continues through the hold, which is where overshoot appears.
 *
 *   Speed constants are local to this block. A test harness reaching into
 *   another module's tuning would couple the two.
 *
 *   Columns: ms,arc,deg,pvL,pvR,sp,state
 *     ms     milliseconds since the turn was commanded
 *     arc    wheel arc covered, mm
 *     deg    odometry heading, degrees
 *     pvL/R  measured wheel speed, mm/s
 *     sp     profile setpoint, mm/s
 *     state  0 turning, 1 holding, 2 done
 * ========================================================================== */
#ifdef TURN_TEST

    #define TURN_TEST_ANGLE_RAD      M_PI_DIV_2   /* magnitude, always positive */
    #define TURN_TEST_CCW            1            /* 1 = left/CCW, 0 = right/CW */

    #define TURN_TEST_SPEED_MM_S     300.0f       /* cruise ceiling, per wheel */
    #define TURN_TEST_MIN_MM_S       0.0f       /* floor, clears stiction */
    #define TURN_TEST_ACCEL_MM_S2    500.0f       /* ramp and brake */

    #define TURN_TEST_HOLD_MS        500U         /* stationary hold after the turn */
    #define TURN_TEST_PRINT_EVERY     10U         /* 1 kHz / 10 = 100 Hz */

    /**
     * @brief  Measured wheel speed in mm/s.
     * @param  motor  MOTOR_LEFT or MOTOR_RIGHT.
     * @return float  Filtered speed converted from counts per second.
     */
    static float TurnTest_Speed_mm_s(motor_t motor)
    {
        return (float)Encoder_GetSpeed_CPS(motor) * MM_PER_COUNT;
    }

    /**
     * @brief  Command both wheels in mm/s.
     * @param  left_mm_s   Left wheel speed. Positive drives the robot forward.
     * @param  right_mm_s  Right wheel speed. Positive drives the robot forward.
     * @note   Motion_SetSpeed() takes counts per second.
     */
    static void TurnTest_Command(float left_mm_s, float right_mm_s)
    {
        Motion_SetSpeed(left_mm_s / MM_PER_COUNT, right_mm_s / MM_PER_COUNT);
    }

    /**
     * @brief  Run one profiled pivot, then hold and keep streaming.
     * @note   Runs forever. The rows after state reaches 1 carry the overshoot.
     */
    static void TurnTest_Run(void)
    {
        profile_t seg;
        int32_t   startL, startR;
        float     arc_target_mm;
        float     sign;
        float     swept = 0.0f;
        float     v     = 0.0f;
        uint32_t  ms    = 0U;
        uint32_t  tick  = 0U;
        uint32_t  hold  = 0U;
        uint8_t   state = 0U;

        sign          = (TURN_TEST_CCW) ? 1.0f : -1.0f;
        arc_target_mm = TURN_TEST_ANGLE_RAD * WHEELBASE_MM * 0.5f;

        UART_SendString("target,");
        Test_SendDecimal(arc_target_mm);
        UART_CRLF();
        UART_SendString("ms,arc,deg,pvL,pvR,sp,state");
        UART_CRLF();

        Odometry_Reset();

        startL = Encoder_GetPosition(MOTOR_LEFT);
        startR = Encoder_GetPosition(MOTOR_RIGHT);

        Profile_Begin(&seg, arc_target_mm,
                      TURN_TEST_SPEED_MM_S,
                      TURN_TEST_MIN_MM_S,
                      TURN_TEST_ACCEL_MM_S2);

        while (1)
        {
            if (!Timebase_GetAndClear())
                continue;

            Encoder_Update();
            Odometry_Update();

            /* Wheels sweep equal and opposite arcs, so each covers half the
               difference. The sign makes it count up for either direction. */
            swept = 0.5f * sign * MM_PER_COUNT *
                    ((float)(Encoder_GetPosition(MOTOR_RIGHT) - startR) -
                     (float)(Encoder_GetPosition(MOTOR_LEFT)  - startL));

            switch (state)
            {
                case 0U:
                    v = Profile_Step(&seg, swept);
                    TurnTest_Command(-v * sign, v * sign);

                    if (Profile_IsComplete(&seg, swept))
                    {
                        hold  = 0U;
                        state = 1U;
                    }
                    break;

                case 1U:
                    v = 0.0f;
                    TurnTest_Command(0.0f, 0.0f);

                    if (++hold >= TURN_TEST_HOLD_MS)
                        state = 2U;
                    break;

                default:
                    v = 0.0f;
                    TurnTest_Command(0.0f, 0.0f);
                    break;
            }

            Motion_Update();
            ms++;

            if (++tick < TURN_TEST_PRINT_EVERY)
                continue;

            tick = 0U;

            UART_SendInt((int32_t)ms);                                      UART_SendByte(',');
            Test_SendDecimal(swept);                                        UART_SendByte(',');
            Test_SendDecimal(Odometry_GetHeading_deg());                    UART_SendByte(',');
            UART_SendInt((int32_t)TurnTest_Speed_mm_s(MOTOR_LEFT));         UART_SendByte(',');
            UART_SendInt((int32_t)TurnTest_Speed_mm_s(MOTOR_RIGHT));        UART_SendByte(',');
            Test_SendDecimal(v);                                            UART_SendByte(',');
            UART_SendInt((int32_t)state);
            UART_CRLF();
        }
    }

#endif /* TURN_TEST */


/* ==========================================================================
 *   Drive Test
 *
 *   One straight move of a fixed distance, then a hold. Compare the printed
 *   dist10 against a ruler on the floor.
 *
 *   Columns: ms,dist,encL,encR,sp,state
 *     ms     milliseconds since the move was commanded
 *     dist   distance covered, mm
 *     encL   left encoder counts since the move started
 *     encR   right encoder counts since the move started
 *     sp     profile setpoint, mm/s
 *     state  0 driving, 1 done
 * ========================================================================== */
#ifdef DRIVE_TEST

    #define DRIVE_TEST_DISTANCE_MM   180.0f
    #define DRIVE_TEST_SPEED_MM_S    300.0f
    #define DRIVE_TEST_MIN_MM_S       0.0f
    #define DRIVE_TEST_ACCEL_MM_S2   300.0f
    #define DRIVE_TEST_PRINT_EVERY    10U

    static void DriveTest_Run(void)
    {
        profile_t seg;
        int32_t   startL, startR, dL, dR;
        float     dist = 0.0f;
        float     v    = 0.0f;
        uint32_t  ms   = 0U;
        uint32_t  tick = 0U;
        uint8_t   state = 0U;

        UART_SendString("target,");
        Test_SendDecimal(DRIVE_TEST_DISTANCE_MM);
        UART_CRLF();
        UART_SendString("ms,dist,encL,encR,sp,state");
        UART_CRLF();

        startL = Encoder_GetPosition(MOTOR_LEFT);
        startR = Encoder_GetPosition(MOTOR_RIGHT);

        Profile_Begin(&seg, DRIVE_TEST_DISTANCE_MM,
                      DRIVE_TEST_SPEED_MM_S, DRIVE_TEST_MIN_MM_S,
                      DRIVE_TEST_ACCEL_MM_S2);

        while (1)
        {
            if (!Timebase_GetAndClear())
                continue;

            Encoder_Update();

            dL   = Encoder_GetPosition(MOTOR_LEFT)  - startL;
            dR   = Encoder_GetPosition(MOTOR_RIGHT) - startR;
            dist = 0.5f * (float)(dL + dR) * MM_PER_COUNT;

            if (state == 0U)
            {
                v = Profile_Step(&seg, dist);

                if (Profile_IsComplete(&seg, dist))
                {
                    v     = 0.0f;
                    state = 1U;
                }
            }

            Motion_SetSpeed(v / MM_PER_COUNT, v / MM_PER_COUNT);
            Motion_Update();
            ms++;

            if (++tick < DRIVE_TEST_PRINT_EVERY)
                continue;

            tick = 0U;

            UART_SendInt((int32_t)ms);                  UART_SendByte(',');
            Test_SendDecimal(dist);                     UART_SendByte(',');
            UART_SendInt(dL);                           UART_SendByte(',');
            UART_SendInt(dR);                           UART_SendByte(',');
            Test_SendDecimal(v);                        UART_SendByte(',');
            UART_SendInt((int32_t)state);
            UART_CRLF();
        }
    }

#endif /* DRIVE_TEST */


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

    #ifdef IR_EMITTER_TEST
        Timebase_Init();
        PORT_G_Init();          /* Right pair  (PG4/PG5) */
        PORT_U_Init();          /* Left pair   (PU0/PU1) */
        #ifdef EMITTER_TEST_WITH_MOTORS
            Motor_Init();
        #endif
    #endif

    #ifdef ODOM_TEST
        Motor_Init();       /* bridges left in standby, wheels roll free */
        Encoder_Init();
        Odometry_Init();    /* caches encoder position, must follow Encoder */
        UART_Init();
        Timebase_Init();
    #endif

    #ifdef TURN_TEST
        Motion_Init();      /* Encoder_Init() + Motor_Init() */
        Odometry_Init();    /* caches encoder position, must follow Motion */
        UART_Init();
        Timebase_Init();
    #endif

    #ifdef DRIVE_TEST
        Motion_Init();      /* Encoder_Init() + Motor_Init() */
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

    #ifdef ODOM_TEST
        OdomTest_Run();
    #endif

    #ifdef TURN_TEST
        TurnTest_Run();
    #endif

    #ifdef DRIVE_TEST
        DriveTest_Run();
    #endif

    #ifdef IR_EMITTER_TEST
        #ifdef EMITTER_DC_HOLD
            EmitterTest_DC();      /* steady all-on for voltage measurement */
        #else
            EmitterTest_Run();     /* the lightshow */
        #endif
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