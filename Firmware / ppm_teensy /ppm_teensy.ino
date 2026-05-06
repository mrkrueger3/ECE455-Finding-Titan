#include "ppm_measurement.h"

#define BASELINE_SAMPLES     16u
#define DETECT_THRESHOLD_NT  65.0f

/* Temporary Teensy pin assignments
   Change these to match your wiring */
static const uint8_t TX_PULSE_PIN   = 2;
static const uint8_t BLANK_EN_PIN   = 3;
static const uint8_t FID_PIN        = 4;
static const uint8_t DETECT_OUT_PIN = 5;

static PPM_Controller ppm;

volatile float    g_last_f_hz      = 0.0f;
volatile float    g_last_b_nT      = 0.0f;
volatile float    g_baseline_b_nT  = 0.0f;
volatile float    g_delta_b_nT     = 0.0f;
volatile uint8_t  g_result_valid   = 0u;
volatile uint8_t  g_baseline_ready = 0u;
volatile uint8_t  g_detected       = 0u;
volatile uint32_t g_good_cycles    = 0u;
volatile uint32_t g_bad_cycles     = 0u;

static float    baseline_sum_nT = 0.0f;
static uint32_t baseline_count  = 0u;

void fid_isr()
{
    PPM_OnInputCaptureIRQ(&ppm);
}

void setup()
{
    pinMode(DETECT_OUT_PIN, OUTPUT);
    digitalWrite(DETECT_OUT_PIN, LOW);

    pinMode(13, OUTPUT);
    digitalWrite(13, LOW);

    Serial.begin(115200);
    delay(500);

    PPM_Init(&ppm,
             TX_PULSE_PIN,
             BLANK_EN_PIN,
             FID_PIN,
             1000000u);   /* 1 MHz time base, matching your STM32 assumption */

    PPM_ConfigTiming(&ppm, 200u, 30u, 800u);

    attachInterrupt(digitalPinToInterrupt(FID_PIN), fid_isr, RISING);

    Serial.println("PPM Teensy start");
    PPM_StartCycle(&ppm);
}

void loop()
{
    PPM_Task(&ppm);

    if (PPM_IsDone(&ppm))
    {
        PPM_Result r = PPM_GetResult(&ppm);

        if (r.valid)
        {
            float abs_delta;

            g_last_f_hz    = r.f_hz;
            g_last_b_nT    = r.b_nT;
            g_result_valid = 1u;
            g_good_cycles++;

            if (!g_baseline_ready)
            {
                baseline_sum_nT += r.b_nT;
                baseline_count++;

                if (baseline_count >= BASELINE_SAMPLES)
                {
                    g_baseline_b_nT  = baseline_sum_nT / (float)baseline_count;
                    g_baseline_ready = 1u;
                }

                g_delta_b_nT = 0.0f;
                g_detected   = 0u;
            }
            else
            {
                g_delta_b_nT = r.b_nT - g_baseline_b_nT;
                abs_delta = (g_delta_b_nT >= 0.0f) ? g_delta_b_nT : -g_delta_b_nT;

                if (abs_delta >= DETECT_THRESHOLD_NT)
                {
                    g_detected = 1u;
                }
                else
                {
                    g_detected = 0u;

                    /* slow baseline tracking when no target is present */
                    g_baseline_b_nT = (0.995f * g_baseline_b_nT) + (0.005f * r.b_nT);
                }
            }

            Serial.print("f_hz=");
            Serial.print(g_last_f_hz, 3);
            Serial.print("  B_nT=");
            Serial.print(g_last_b_nT, 2);
            Serial.print("  baseline_nT=");
            Serial.print(g_baseline_b_nT, 2);
            Serial.print("  delta_nT=");
            Serial.print(g_delta_b_nT, 2);
            Serial.print("  detected=");
            Serial.println(g_detected ? 1 : 0);

            digitalWrite(13, g_detected ? HIGH : LOW);
        }
        else
        {
            g_result_valid = 0u;
            g_detected     = 0u;
            g_bad_cycles++;

            Serial.println("Invalid cycle");
            digitalWrite(13, LOW);
        }

        digitalWrite(DETECT_OUT_PIN, g_detected ? HIGH : LOW);

        delay(200u);
        PPM_StartCycle(&ppm);
    }
}
