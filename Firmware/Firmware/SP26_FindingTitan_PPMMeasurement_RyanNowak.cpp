#include "ppm_measurement.h"

#define PPM_MIN_PERIODS_FOR_VALID_RESULT   128u

static inline void gpio_write(uint8_t pin, bool on)
{
    digitalWrite(pin, on ? HIGH : LOW);
}

static inline uint32_t now_ms(void)
{
    return millis();
}

static inline uint32_t now_us(void)
{
    return micros();
}

static void set_state(PPM_Controller *ppm, PPM_State s)
{
    ppm->state = s;
    ppm->t_state_ms = now_ms();
}

static void capture_reset(PPM_Controller *ppm)
{
    ppm->last_capture      = 0u;
    ppm->have_last_capture = false;
    ppm->total_ticks       = 0u;
    ppm->periods           = 0u;
}

void PPM_Init(PPM_Controller *ppm,
              uint8_t tx_pin,
              uint8_t blank_pin,
              uint8_t fid_pin,
              uint32_t timer_tick_hz)
{
    ppm->tx_pin        = tx_pin;
    ppm->blank_pin     = blank_pin;
    ppm->fid_pin       = fid_pin;
    ppm->timer_tick_hz = timer_tick_hz;

    ppm->polarize_ms = 200u;
    ppm->recovery_ms = 30u;
    ppm->acquire_ms  = 800u;

    /* Same idea as your STM32 code */
    ppm->min_valid_ticks = timer_tick_hz / 10000u;   /* 100 us */
    if (ppm->min_valid_ticks < 20u) ppm->min_valid_ticks = 20u;

    ppm->max_valid_ticks = timer_tick_hz / 500u;     /* 2 ms */
    if (ppm->max_valid_ticks <= ppm->min_valid_ticks)
        ppm->max_valid_ticks = ppm->min_valid_ticks + 100u;

    pinMode(ppm->tx_pin, OUTPUT);
    pinMode(ppm->blank_pin, OUTPUT);
    pinMode(ppm->fid_pin, INPUT);

    capture_reset(ppm);

    ppm->result.valid = false;
    ppm->result.f_hz  = 0.0f;
    ppm->result.b_nT  = 0.0f;

    /* Safe idle state, same intent as STM32 version */
    gpio_write(ppm->tx_pin, false);      /* polarization off */
    gpio_write(ppm->blank_pin, true);    /* RX blanked */

    set_state(ppm, PPM_STATE_IDLE);
}

void PPM_ConfigTiming(PPM_Controller *ppm,
                      uint32_t polarize_ms,
                      uint32_t recovery_ms,
                      uint32_t acquire_ms)
{
    ppm->polarize_ms = polarize_ms;
    ppm->recovery_ms = recovery_ms;
    ppm->acquire_ms  = acquire_ms;
}

void PPM_StartCycle(PPM_Controller *ppm)
{
    noInterrupts();
    ppm->result.valid = false;
    ppm->result.f_hz  = 0.0f;
    ppm->result.b_nT  = 0.0f;
    capture_reset(ppm);
    interrupts();

    /* During pulse: TX on, RX blanked */
    gpio_write(ppm->blank_pin, true);
    gpio_write(ppm->tx_pin, true);

    set_state(ppm, PPM_STATE_POLARIZE);
}

void PPM_Task(PPM_Controller *ppm)
{
    uint32_t dt_ms = now_ms() - ppm->t_state_ms;

    switch (ppm->state)
    {
        case PPM_STATE_IDLE:
            break;

        case PPM_STATE_POLARIZE:
            if (dt_ms >= ppm->polarize_ms)
            {
                gpio_write(ppm->tx_pin, false);    /* end polarization */
                gpio_write(ppm->blank_pin, true);
                set_state(ppm, PPM_STATE_RECOVERY_BLANK);
            }
            break;

        case PPM_STATE_RECOVERY_BLANK:
            if (dt_ms >= ppm->recovery_ms)
            {
                noInterrupts();
                capture_reset(ppm);
                interrupts();

                gpio_write(ppm->blank_pin, false); /* unblank RX */
                set_state(ppm, PPM_STATE_ACQUIRE);
            }
            break;

        case PPM_STATE_ACQUIRE:
            if (dt_ms >= ppm->acquire_ms)
            {
                uint64_t total_ticks_copy;
                uint32_t periods_copy;

                noInterrupts();
                total_ticks_copy = ppm->total_ticks;
                periods_copy     = ppm->periods;
                interrupts();

                if ((periods_copy >= PPM_MIN_PERIODS_FOR_VALID_RESULT) &&
                    (total_ticks_copy > 0u))
                {
                    float f = ((float)ppm->timer_tick_hz * (float)periods_copy) /
                              (float)total_ticks_copy;

                    ppm->result.f_hz  = f;
                    ppm->result.b_nT  = f * PPM_NT_PER_HZ;
                    ppm->result.valid = true;
                }
                else
                {
                    ppm->result.valid = false;
                    ppm->result.f_hz  = 0.0f;
                    ppm->result.b_nT  = 0.0f;
                }

                gpio_write(ppm->blank_pin, true); /* blank between cycles */
                set_state(ppm, PPM_STATE_DONE);
            }
            break;

        case PPM_STATE_DONE:
            break;

        default:
            set_state(ppm, PPM_STATE_IDLE);
            break;
    }
}

void PPM_OnInputCaptureIRQ(PPM_Controller *ppm)
{
    if (ppm->state != PPM_STATE_ACQUIRE) return;

    uint32_t now = now_us();

    if (!ppm->have_last_capture)
    {
        ppm->last_capture = now;
        ppm->have_last_capture = true;
        return;
    }

    uint32_t dt = now - ppm->last_capture;
    ppm->last_capture = now;

    /* Reject glitches, same logic as STM32 version */
    if ((dt < ppm->min_valid_ticks) || (dt > ppm->max_valid_ticks))
    {
        ppm->have_last_capture = false;
        return;
    }

    ppm->total_ticks += (uint64_t)dt;
    ppm->periods++;
}

PPM_Result PPM_GetResult(const PPM_Controller *ppm)
{
    return ppm->result;
}

bool PPM_IsDone(const PPM_Controller *ppm)
{
    return (ppm->state == PPM_STATE_DONE);
}
