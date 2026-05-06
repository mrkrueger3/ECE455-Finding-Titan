#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>

#define PPM_PROTON_HZ_PER_T   (42.57747892e6f)
#define PPM_NT_PER_HZ         (1.0e9f / PPM_PROTON_HZ_PER_T)

typedef enum {
    PPM_STATE_IDLE = 0,
    PPM_STATE_POLARIZE,
    PPM_STATE_RECOVERY_BLANK,
    PPM_STATE_ACQUIRE,
    PPM_STATE_DONE
} PPM_State;

typedef struct {
    bool  valid;
    float f_hz;
    float b_nT;
} PPM_Result;

typedef struct {
    /* Pin wiring */
    uint8_t tx_pin;
    uint8_t blank_pin;
    uint8_t fid_pin;

    /* "Timer tick" for captured periods
       We keep this at 1 MHz so 1 tick = 1 us, matching your STM32 setup */
    uint32_t timer_tick_hz;

    /* Timing (ms) */
    uint32_t polarize_ms;
    uint32_t recovery_ms;
    uint32_t acquire_ms;

    /* State */
    volatile PPM_State state;
    uint32_t t_state_ms;

    /* Capture bookkeeping */
    volatile uint32_t last_capture;
    volatile bool     have_last_capture;

    volatile uint64_t total_ticks;
    volatile uint32_t periods;

    uint32_t min_valid_ticks;
    uint32_t max_valid_ticks;

    /* Output */
    PPM_Result result;

} PPM_Controller;

void PPM_Init(PPM_Controller *ppm,
              uint8_t tx_pin,
              uint8_t blank_pin,
              uint8_t fid_pin,
              uint32_t timer_tick_hz);

void PPM_ConfigTiming(PPM_Controller *ppm,
                      uint32_t polarize_ms,
                      uint32_t recovery_ms,
                      uint32_t acquire_ms);

void PPM_StartCycle(PPM_Controller *ppm);
void PPM_Task(PPM_Controller *ppm);

void PPM_OnInputCaptureIRQ(PPM_Controller *ppm);

PPM_Result PPM_GetResult(const PPM_Controller *ppm);
bool PPM_IsDone(const PPM_Controller *ppm);
