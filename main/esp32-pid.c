#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gptimer.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"

const char *TAG = "";

#define PWM_FREQUENCY   100000
#define PWM_CHANNEL     LEDC_CHANNEL_0

#define voltageKp   0.16f
#define voltageKi   14.8f
#define voltageKd   0.0f
#define currentKp   0.0f
#define currentKi   44.0f
#define currentKd   0.0f

#define voltageSetpoint 12.0f

static float currentSetpoint = 3.0f;

static float e_voltage, e1_voltage=voltageSetpoint, e2_voltage=voltageSetpoint;
static float e_current, e1_current=0.0f, e2_current=0.0f;
static float voltage_sensor_xn1, voltage_sensor, current_sensor_xn1, current_sensor;
static float yn1_voltage, yn_voltage, yn1_current, yn_current;
static float dutyCycle;

static uint16_t dutyCycleTicks;

static int adc_raw[2];
static int voltage[2];

adc_cali_handle_t adc1_cali_chan0_handle = NULL;
adc_cali_handle_t adc1_cali_chan1_handle = NULL;
adc_oneshot_unit_handle_t adc1_handle = NULL;
TaskHandle_t pid_task_handle;

bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
void adc_calibration_deinit(adc_cali_handle_t handle);


static bool IRAM_ATTR timer_on_alarm(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    // // Debug GPIO
    // gpio_set_level(GPIO_NUM_2, 1);

    BaseType_t mustYield = pdFALSE;
    vTaskNotifyGiveFromISR(pid_task_handle, &mustYield);

    // return whether we need to yield at the end of ISR
    return (mustYield == pdTRUE);
}

static void IRAM_ATTR pid_task(void * pvParameters) {
    while(1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        adc_oneshot_read(adc1_handle, ADC_CHANNEL_7, &adc_raw[0]);
        adc_cali_raw_to_voltage(adc1_cali_chan0_handle, adc_raw[0], &voltage[0]);
        voltage_sensor = voltage[0] * 0.008808 * 0.9124 * 1.2;
        yn_voltage = 5.21885552778623E-01f * yn1_voltage + 2.39057223610688E-01f * voltage_sensor + 2.39057223610688E-01f * voltage_sensor_xn1;

        adc_oneshot_read(adc1_handle, ADC_CHANNEL_0, &adc_raw[1]);
        adc_cali_raw_to_voltage(adc1_cali_chan1_handle, adc_raw[1], &voltage[1]);
        current_sensor = (voltage[1] - 2945) * 0.01;
        yn_current = 5.21885552778623E-01f * yn1_current + 2.39057223610688E-01f * current_sensor + 2.39057223610688E-01f * current_sensor_xn1;

        e_voltage = voltageSetpoint - yn_voltage;
        
        // PID
        currentSetpoint = currentSetpoint + 
                            e_voltage*(voltageKp+(voltageKi*0.00015f)+(voltageKd/0.00015f)) - 
                            e1_voltage*(voltageKp+(2*voltageKd/0.00015f)) +
                            e2_voltage*(voltageKd/0.00015f);
        if (currentSetpoint > 3) {
          currentSetpoint = 3;
        } else if (currentSetpoint < 0) {
          currentSetpoint = 0;
        }
        e_current = currentSetpoint - yn_current;
        dutyCycle = dutyCycle + 
                    e_current*(currentKp+(currentKi*0.00015f)+(currentKd/0.00015f)) - 
                    e1_current*(currentKp+(2*currentKd/0.00015f)) +
                    e2_current*(currentKd/0.00015f);

        // Set PWM duty cycle
        if (dutyCycle > 0.95) {
            dutyCycle = 0.95;
        }
        else if (dutyCycle < 0.05) {
            dutyCycle = 0.05;
        }
        dutyCycleTicks = (uint16_t) (dutyCycle*511);    // Set duty: ((2 ** LEDC_TIMER_9_BIT) - 1) * dutyCycle
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, PWM_CHANNEL, dutyCycleTicks);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, PWM_CHANNEL);

        // Update n-1 values
        voltage_sensor_xn1 = voltage_sensor;
        current_sensor_xn1 = current_sensor;
        yn1_voltage = yn_voltage;
        yn1_current = yn_current;
        e2_voltage = e1_voltage;
        e2_current = e1_current;
        e1_voltage = e_voltage;
        e1_current = e_current;

        // // Debug GPIO
        // gpio_set_level(GPIO_NUM_2, 0);
    }
}

void app_main(void)
{
    //-------------Create task---------------//
    xTaskCreatePinnedToCore(
                    pid_task,           /* Function to implement the task */
                    "PID Task",         /* Name of the task */
                    4096,               /* Stack size in words */
                    NULL,               /* Task input parameter */
                    2,                  /* Priority of the task */
                    &pid_task_handle,    /* Task handle. */
                    1);                 /* Core where the task should run */

    //-------------GPIO Config---------------//
    gpio_reset_pin(GPIO_NUM_2);
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);

    //-------------ADC1 Init---------------//
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_11,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_7, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_0, &config));

    //-------------ADC1 Calibration Init---------------//
    bool do_calibration1_chan0 = adc_calibration_init(ADC_UNIT_1, ADC_CHANNEL_7, ADC_ATTEN_DB_11, &adc1_cali_chan0_handle);
    bool do_calibration1_chan1 = adc_calibration_init(ADC_UNIT_1, ADC_CHANNEL_0, ADC_ATTEN_DB_11, &adc1_cali_chan1_handle);

    if (!do_calibration1_chan0) {
        ESP_LOGE(TAG, "Channel0 calibration failed");
    }
    if (!do_calibration1_chan1) {
        ESP_LOGE(TAG, "Channel1 calibration failed");
    }

    //-------------PWM timer Init---------------//
    // LEDC PWM timer configuration
    ledc_timer_config_t PWM_timer = {
        .speed_mode       = LEDC_HIGH_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_9_BIT,
        .freq_hz          = PWM_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&PWM_timer));

    // LEDC PWM channel configuration
    ledc_channel_config_t PWM_channel = {
        .speed_mode     = LEDC_HIGH_SPEED_MODE,
        .channel        = PWM_CHANNEL,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = GPIO_NUM_16,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&PWM_channel));

    //-------------Timer Init---------------//
    ESP_LOGI(TAG, "Create timer handle");
    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, // 1MHz, 1 tick=1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_on_alarm,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, NULL));

    ESP_LOGI(TAG, "Enable timer");
    ESP_ERROR_CHECK(gptimer_enable(gptimer));

    //-------------Timer Config---------------//
    ESP_LOGI(TAG, "Start timer");
    gptimer_alarm_config_t alarm_config1 = {
        .reload_count = 0,
        .alarm_count = 150, // period in us
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config1));
    ESP_ERROR_CHECK(gptimer_start(gptimer));

    while (1) {
        // Do nothing
        vTaskDelay(pdMS_TO_TICKS(2000));

        // Print
        // ESP_LOGI(TAG, "Voltage int: %d", voltage[0]);
        // ESP_LOGI(TAG, "Voltage: %f", yn_voltage);
        // ESP_LOGI(TAG, "Duty: %f", dutyCycle);
    }

    // Delete ADC
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
    if (do_calibration1_chan0) {
        adc_calibration_deinit(adc1_cali_chan0_handle);
    }
    if (do_calibration1_chan1) {
        adc_calibration_deinit(adc1_cali_chan1_handle);
    }

    // Delete timer
    ESP_LOGI(TAG, "Stop timer");
    ESP_ERROR_CHECK(gptimer_stop(gptimer));
    ESP_LOGI(TAG, "Disable timer");
    ESP_ERROR_CHECK(gptimer_disable(gptimer));
    ESP_LOGI(TAG, "Delete timer");
    ESP_ERROR_CHECK(gptimer_del_timer(gptimer));
}
