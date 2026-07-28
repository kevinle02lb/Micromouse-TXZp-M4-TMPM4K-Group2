/**
 * @file        PID.h
 * @brief       PID controller
 * @version     V1.0.0
 * @date        18-06-2026
 *
 * @details
 * @note
 *
 *
 *   File structure and Doxygen formatting assisted by AI.
 *
 * Copyright (c) [Kevin Le] 2026
 */

#ifndef PID_H
#define PID_H


typedef struct 
{
    float kp;           /*!< Proportional gain */
    float ki;           /*!< Integral gain */
    float kd;           /*!< Derivative gain */
    float integral;     /*!< Accumulated integral term */
    float derivative;   /*!< Derivative of Error */
    float prev_error;   /*!< Previous error (for derivative) */
    float dt;           /*!< Sample time in seconds (0.001f) */
    float out_min;      /*!< Minimum output clamp (e.g., -100.0f) */
    float out_max;      /*!< Maximum output clamp (e.g., 100.0f) */
} pid_t;

/* ==========================================================================
 *   PID Default Gains
 * ========================================================================== */

#define PID_KP               0.5f
#define PID_KI               0.1f
#define PID_KD               0.02f
#define PID_DT               0.001f
#define PID_OUT_MIN          -100.0f
#define PID_OUT_MAX          100.0f


/* ==========================================================================
 *   Function Prototypes
 * ========================================================================== */
void PID_Create(pid_t *pid);
void PID_Init(pid_t *pid, float Kp, float Ki, float Kd, float dt, float out_min, float out_max);
void PID_Reset(pid_t *pid);
float PID_Update(pid_t *pid, float error);
float CalculateError(float SP, float PV);


#endif /* PID_H */