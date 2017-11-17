#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_power.h"
#include "nrf_drv_pwm.h"
#include "nrf_drv_timer.h"
#include "app_error.h"
#include "app_timer.h"
#include "app_util_platform.h"
#include "boards.h"
#include "bsp.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define GPIO_OUTPUT_PIN_NUMBER 25
#define TOP 10000
#define SAMPLES_IN_BUFFER 5

static nrf_drv_pwm_t m_pwm0 = NRF_DRV_PWM_INSTANCE(0);
static const nrf_drv_timer_t m_timer = NRF_DRV_TIMER_INSTANCE(0);
static nrf_saadc_value_t     m_buffer_pool[2][SAMPLES_IN_BUFFER];
static nrf_ppi_channel_t     m_ppi_channel;
static uint32_t              m_adc_reading;
static uint32_t              m_adc_num_samples;


void timer_handler(nrf_timer_event_t event_type, void * p_context)
{
    /*
    * Dummy function for timer event handler.
    */
}


void saadc_sampling_event_init(void)
{
    /*
    * Initializes timer and PPI channel, then connects them to ADC event.
    */

    ret_code_t err_code;

    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;

    err_code = nrf_drv_timer_init(&m_timer, &timer_cfg, timer_handler);
    APP_ERROR_CHECK(err_code);

    // Set up timer to trigger event every 400 ms.
    uint32_t ticks = nrf_drv_timer_ms_to_ticks(&m_timer, 400);
    nrf_drv_timer_extended_compare(&m_timer,
                                   NRF_TIMER_CC_CHANNEL0,
                                   ticks,
                                   NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                   false);
    nrf_drv_timer_enable(&m_timer);

    uint32_t timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(&m_timer,
                                                                                NRF_TIMER_CC_CHANNEL0);
    uint32_t saadc_sample_task_addr   = nrf_drv_saadc_sample_task_get();

    // Set up PPI channel such that timer triggers ADC event.
    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel,
                                          timer_compare_event_addr,
                                          saadc_sample_task_addr);
    APP_ERROR_CHECK(err_code);
}


void saadc_sampling_event_enable(void)
{
    /*
    * Turns on PPI channel containing timer and ADC event.
    */

    ret_code_t err_code = nrf_drv_ppi_channel_enable(m_ppi_channel);

    APP_ERROR_CHECK(err_code);
}


void saadc_sampling_event_disable(void)
{
    /*
    * Turns off PPI channel containing timer and ADC event.
    */

    ret_code_t err_code = nrf_drv_ppi_channel_disable(m_ppi_channel);

    APP_ERROR_CHECK(err_code);
}


void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    /*
    * Handler for ADC event.
    */

    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        ret_code_t err_code;

        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);

        // Take readings and add to total.
        int i;

        for (i = 0; i < SAMPLES_IN_BUFFER; i++)
        {
            m_adc_reading += p_event->data.done.p_buffer[i];
            m_adc_num_samples++;
        }
    }
}


void saadc_init(void)
{
    /*
    * Initializes ADC.
    */

    ret_code_t err_code;
    nrf_saadc_channel_config_t channel_config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);

    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

}


static void run_servo(int time_in_ms, int direction)
{
    NRF_LOG_INFO("Run Servo");

    /*
     * Rotates servo clockwise (direction = 1), counter-clockwise (direction = -1),
     * or stops servo (direction = 0). Servo runs for specified time.
     */

    //Initialize PWM instance.
    nrf_drv_pwm_config_t const config0 =
    {
        .output_pins =
        {
            GPIO_OUTPUT_PIN_NUMBER, // channel 0
            NRF_PWM_PIN_NOT_CONNECTED, // channel 1
            NRF_PWM_PIN_NOT_CONNECTED, // channel 2
            NRF_PWM_PIN_NOT_CONNECTED  // channel 3
        },
        .irq_priority = APP_IRQ_PRIORITY_LOWEST,
        .base_clock   = NRF_PWM_CLK_500kHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = TOP,
        .load_mode    = NRF_PWM_LOAD_COMMON,
        .step_mode    = NRF_PWM_STEP_AUTO
    };
    APP_ERROR_CHECK(nrf_drv_pwm_init(&m_pwm0, &config0, NULL));

    // Set PWM values.
    static nrf_pwm_values_common_t seq0_values[1] = {9300};
    static nrf_pwm_values_common_t seq1_values[1] = {9238};
    static nrf_pwm_values_common_t seq2_values[1] = {9200};

    int rep = time_in_ms / 20;
    nrf_pwm_sequence_t const seq0 =
    {
        .values.p_common = seq0_values,
        .length          = NRF_PWM_VALUES_LENGTH(seq0_values),
        .repeats         = rep,
        .end_delay       = 0
    };

    nrf_pwm_sequence_t const seq1 =
    {
        .values.p_common = seq1_values,
        .length          = NRF_PWM_VALUES_LENGTH(seq1_values),
        .repeats         = rep,
        .end_delay       = 0
    };

    nrf_pwm_sequence_t const seq2 =
    {
        .values.p_common = seq2_values,
        .length          = NRF_PWM_VALUES_LENGTH(seq2_values),
        .repeats         = rep,
        .end_delay       = 0
    };

    // Run servo.
    if (direction > 0)
        (void)nrf_drv_pwm_simple_playback(&m_pwm0, &seq0, 1, NRF_DRV_PWM_FLAG_STOP);
    if (direction == 0)
        (void)nrf_drv_pwm_simple_playback(&m_pwm0, &seq1, 1, NRF_DRV_PWM_FLAG_STOP);
    if (direction < 0)
        (void)nrf_drv_pwm_simple_playback(&m_pwm0, &seq2, 1, NRF_DRV_PWM_FLAG_STOP);

    // Free PWM instance.
    nrf_drv_pwm_uninit(&m_pwm0);
}


static void take_optic_reading(void)
{
    /*
    * Takes readings over 2 seconds, then averages and stores result
    * in m_adc_reading.
    */

    m_adc_reading = 0;
    m_adc_num_samples = 0;

    //Turn on PPI channel, collect readings for 2 seconds, then turn channel off.
    saadc_sampling_event_enable();
    nrf_delay_ms(2000);
    saadc_sampling_event_disable();
}


int main(void)
{
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    //APP_ERROR_CHECK(nrf_drv_power_init(NULL));
    //APP_ERROR_CHECK(nrf_pwr_mgmt_init());

    NRF_LOG_INFO("Starting...");

    saadc_init();
    saadc_sampling_event_init();

    while (1)
    {
        take_optic_reading();
        if (m_adc_reading > 1000)
        {
            run_servo(2000,1);
            run_servo(1000,0);
        }
        else
        {
            run_servo(2000,-1);
            run_servo(1000,0);
        }

        //nrf_pwr_mgmt_run();
        NRF_LOG_FLUSH();
        nrf_delay_ms(10000);
    }
}