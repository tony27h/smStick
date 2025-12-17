#include "air_app.h"

#include <stdio.h>
#include <string.h>

#include "bme69x.h"
#include "bme690_port.h"

/* BSEC */
#include "bsec_interface.h"
#include "bsec_datatypes.h"
#include "bsec_iaq.h"

/* BSEC state store */
#include "bsec_state_store.h"

/* ---------- USER TUNABLES ---------- */
#define I2C_ADDR_BME_RAW   (0x77)
#define I2C_ADDR_BME_BSEC  (0x76)

#define PRINT_PERIOD_MS    (10000)
#define UART_TIMEOUT_MS    (200)

/* Save BSEC state every 5 minutes */
#define BSEC_SAVE_PERIOD_MS (5u * 60u * 1000u)

/* ---------- STATIC STATE ---------- */
static I2C_HandleTypeDef *s_hi2c = NULL;
static UART_HandleTypeDef *s_huart = NULL;

static struct bme69x_dev s_bme_raw;
static struct bme69x_dev s_bme_bsec;

static air_readings_t s_latest;

static uint32_t s_last_print_ms = 0;
static uint32_t s_last_raw_ms = 0;

/* ---- BSEC instance memory */
static uint8_t s_bsec_inst_mem[6000];
static void *s_bsec_inst = s_bsec_inst_mem;

static uint8_t s_bsec_workbuf[BSEC_MAX_WORKBUFFER_SIZE];

/* Track whether we successfully loaded state once (optional) */
static uint8_t s_bsec_state_loaded = 0;

static int64_t millis_to_ns(uint32_t ms)
{
    return (int64_t)ms * 1000000LL;
}

static void uart_print_line(const char *s)
{
    if (!s_huart || !s) return;
    HAL_UART_Transmit(s_huart, (uint8_t*)s, (uint16_t)strlen(s), UART_TIMEOUT_MS);
}

/* Configure BME69x basic T/H/P settings (used for raw sensor) */
static int8_t config_bme_tph(struct bme69x_dev *dev)
{
    struct bme69x_conf conf = {0};
    conf.os_temp = BME69X_OS_2X;
    conf.os_pres = BME69X_OS_16X;
    conf.os_hum  = BME69X_OS_1X;
    conf.filter  = BME69X_FILTER_SIZE_3;
    conf.odr     = BME69X_ODR_NONE;

    return bme69x_set_conf(&conf, dev);
}

/* Raw sensor read helper (0x77).
   This is the ONLY source for: temperature, RH, pressure display. */
static int8_t read_raw_sensor(float *t_c, float *rh, float *p_pa)
{
    int8_t rslt;
    struct bme69x_data data;
    uint8_t n = 0;

    rslt = bme69x_set_op_mode(BME69X_FORCED_MODE, &s_bme_raw);
    if (rslt != BME69X_OK) return rslt;

    /* Wait for measurement completion */
    struct bme69x_conf conf;
    rslt = bme69x_get_conf(&conf, &s_bme_raw);
    if (rslt != BME69X_OK) return rslt;

    uint32_t dur_us = bme69x_get_meas_dur(BME69X_FORCED_MODE, &conf, &s_bme_raw);
    s_bme_raw.delay_us(dur_us + 20000, s_bme_raw.intf_ptr); /* +20ms margin */

    rslt = bme69x_get_data(BME69X_FORCED_MODE, &data, &n, &s_bme_raw);
    if (rslt != BME69X_OK || n == 0) return rslt;

#ifdef BME69X_USE_FPU
    *t_c = data.temperature;
    *p_pa = data.pressure;
    *rh = data.humidity;
#else
    *t_c = data.temperature / 100.0f;
    *p_pa = (float)data.pressure;
    *rh = data.humidity / 1000.0f;
#endif
    return BME69X_OK;
}

/* Apply BSEC-requested settings to the BME sensor @0x76 and trigger measurement if needed. */
static int8_t bsec_apply_settings_and_measure(const bsec_bme_settings_t *s, struct bme69x_data *out)
{
    int8_t rslt;

    if (!s->trigger_measurement)
    {
        return BME69X_W_NO_NEW_DATA;
    }

    /* Configure oversampling and filter for BSEC sensor */
    struct bme69x_conf conf = {0};
    conf.os_temp = s->temperature_oversampling;
    conf.os_hum  = s->humidity_oversampling;
    conf.os_pres = s->pressure_oversampling;
    conf.filter  = BME69X_FILTER_SIZE_3;
    conf.odr     = BME69X_ODR_NONE;

    rslt = bme69x_set_conf(&conf, &s_bme_bsec);
    if (rslt != BME69X_OK) return rslt;

    /* Configure heater if requested */
    struct bme69x_heatr_conf h = {0};
    if (s->run_gas)
    {
        h.enable = BME69X_ENABLE;
        h.heatr_temp = s->heater_temperature;
        h.heatr_dur  = s->heater_duration;
    }
    else
    {
        h.enable = BME69X_DISABLE;
        h.heatr_temp = 0;
        h.heatr_dur  = 0;
    }

    rslt = bme69x_set_heatr_conf(BME69X_FORCED_MODE, &h, &s_bme_bsec);
    if (rslt != BME69X_OK) return rslt;

    /* Trigger measurement */
    rslt = bme69x_set_op_mode(BME69X_FORCED_MODE, &s_bme_bsec);
    if (rslt != BME69X_OK) return rslt;

    /* Wait until measurement should be done */
    uint32_t dur_us = bme69x_get_meas_dur(BME69X_FORCED_MODE, &conf, &s_bme_bsec);
    s_bme_bsec.delay_us(dur_us + ((uint32_t)s->heater_duration * 1000U) + 20000U, s_bme_bsec.intf_ptr);

    /* Read data */
    uint8_t n = 0;
    rslt = bme69x_get_data(BME69X_FORCED_MODE, out, &n, &s_bme_bsec);
    if (rslt != BME69X_OK || n == 0) return rslt;

    return BME69X_OK;
}

static void uart_print_now(void)
{
    char line[220];

    /* IMPORTANT: T/RH/P come ONLY from raw sensor (0x77) stored in s_latest */
    float p_hpa = s_latest.p_pa / 100.0f;

    int len = snprintf(line, sizeof(line),
                       "T=%.2f C RH=%.2f %% P=%.2f hPa | IAQ=%.1f (acc=%u)\r\n",
                       s_latest.t_c,
                       s_latest.rh,
                       p_hpa,
                       s_latest.iaq,
                       (unsigned)s_latest.iaq_accuracy);

    HAL_UART_Transmit(s_huart, (uint8_t*)line, (uint16_t)len, UART_TIMEOUT_MS);
}

HAL_StatusTypeDef air_app_init(I2C_HandleTypeDef *hi2c1, UART_HandleTypeDef *huart1)
{
    s_hi2c = hi2c1;
    s_huart = huart1;
    memset(&s_latest, 0, sizeof(s_latest));


    /* -------- RAW BME @0x77 ---------- */
    if (bme690_port_init_i2c(&s_bme_raw, s_hi2c, I2C_ADDR_BME_RAW) != BME69X_OK) return HAL_ERROR;
    if (bme69x_init(&s_bme_raw) != BME69X_OK) return HAL_ERROR;
    if (config_bme_tph(&s_bme_raw) != BME69X_OK) return HAL_ERROR;

    /* -------- BSEC BME @0x76 ---------- */
    if (bme690_port_init_i2c(&s_bme_bsec, s_hi2c, I2C_ADDR_BME_BSEC) != BME69X_OK) return HAL_ERROR;
    if (bme69x_init(&s_bme_bsec) != BME69X_OK) return HAL_ERROR;

    /* -------- BSEC init ---------- */
    size_t need = bsec_get_instance_size();
    if (need > sizeof(s_bsec_inst_mem)) return HAL_ERROR;

    if (bsec_init(s_bsec_inst) != BSEC_OK) return HAL_ERROR;

    /* Load IAQ config blob */
    if (bsec_set_configuration(s_bsec_inst,
                              bsec_config_iaq,
                              (uint32_t)sizeof(bsec_config_iaq),
                              s_bsec_workbuf,
                              (uint32_t)sizeof(s_bsec_workbuf)) != BSEC_OK)
    {
        return HAL_ERROR;
    }

    /* ---- BSEC state store init + load ---- */


    /* Subscribe to IAQ output at LP mode */
    bsec_sensor_configuration_t req[1];
    req[0].sensor_id = BSEC_OUTPUT_IAQ;
    req[0].sample_rate = BSEC_SAMPLE_RATE_LP;

    bsec_sensor_configuration_t required[BSEC_MAX_PHYSICAL_SENSOR];
    uint8_t n_required = BSEC_MAX_PHYSICAL_SENSOR;

    bsec_library_return_t br = bsec_update_subscription(s_bsec_inst, req, 1, required, &n_required);
    if (br != BSEC_OK) return HAL_ERROR;



    /* Force an immediate first update+print on the first air_app_process() call */
    s_last_raw_ms   = HAL_GetTick() - PRINT_PERIOD_MS;
    s_last_print_ms = HAL_GetTick() - PRINT_PERIOD_MS;

    return HAL_OK;
}

HAL_StatusTypeDef air_app_process(void)
{
    uint32_t now_ms = HAL_GetTick();

    /* ---- Update raw sensor (0x77) every 10 seconds */
    if ((now_ms - s_last_raw_ms) >= PRINT_PERIOD_MS)
    {
        float t, rh, p;
        if (read_raw_sensor(&t, &rh, &p) == BME69X_OK)
        {
            s_latest.t_c = t;
            s_latest.rh = rh;
            s_latest.p_pa = p;
        }
        s_last_raw_ms = now_ms;
    }

    /* ---- Run BSEC state machine (0x76) ---- */
    bsec_bme_settings_t s;
    bsec_library_return_t br;

    br = bsec_sensor_control(s_bsec_inst, millis_to_ns(now_ms), &s);
    if (br == BSEC_OK)
    {
        if (s.trigger_measurement)
        {
            struct bme69x_data d;
            int8_t r = bsec_apply_settings_and_measure(&s, &d);

            if (r == BME69X_OK)
            {
                bsec_input_t in[4];
                uint8_t n_in = 0;

#ifdef BME69X_USE_FPU
                float t_c = d.temperature;
                float p_pa = d.pressure;
                float rh = d.humidity;
                float gas_ohm = d.gas_resistance;
#else
                float t_c = d.temperature / 100.0f;
                float p_pa = (float)d.pressure;
                float rh = d.humidity / 1000.0f;
                float gas_ohm = (float)d.gas_resistance;
#endif

                int64_t ts = millis_to_ns(now_ms);

                if (s.process_data & BSEC_PROCESS_TEMPERATURE)
                    in[n_in++] = (bsec_input_t){ .time_stamp = ts, .signal = t_c, .signal_dimensions = 1, .sensor_id = BSEC_INPUT_TEMPERATURE };

                if (s.process_data & BSEC_PROCESS_HUMIDITY)
                    in[n_in++] = (bsec_input_t){ .time_stamp = ts, .signal = rh, .signal_dimensions = 1, .sensor_id = BSEC_INPUT_HUMIDITY };

                if (s.process_data & BSEC_PROCESS_PRESSURE)
                    in[n_in++] = (bsec_input_t){ .time_stamp = ts, .signal = p_pa, .signal_dimensions = 1, .sensor_id = BSEC_INPUT_PRESSURE };

                if (s.process_data & BSEC_PROCESS_GAS)
                    in[n_in++] = (bsec_input_t){ .time_stamp = ts, .signal = gas_ohm, .signal_dimensions = 1, .sensor_id = BSEC_INPUT_GASRESISTOR };

                bsec_output_t out[8];
                uint8_t n_out = 8;

                br = bsec_do_steps(s_bsec_inst, in, n_in, out, &n_out);
                if (br == BSEC_OK || br > 0)
                {
                    for (uint8_t i = 0; i < n_out; i++)
                    {
                        if (out[i].sensor_id == BSEC_OUTPUT_IAQ)
                        {
                            s_latest.iaq = out[i].signal;
                            s_latest.iaq_accuracy = out[i].accuracy;
                        }
                    }
                }
            }
        }
    }

    /* ---- UART print every 10 seconds */
    if ((now_ms - s_last_print_ms) >= PRINT_PERIOD_MS)
    {
        uart_print_now();
        s_last_print_ms = now_ms;
    }

    /* ---- Save BSEC state every 5 minutes (only when accuracy >= 1) */
    if (s_latest.iaq_accuracy >= 1)
    {
        HAL_StatusTypeDef st = bsec_state_store_maybe_save(s_bsec_inst, now_ms, BSEC_SAVE_PERIOD_MS);

        if (st == HAL_OK)
        {
            uart_print_line("BSEC state: SAVE succeeded\r\n");
        }
        else if (st == HAL_ERROR)
        {
            uart_print_line("BSEC state: SAVE FAILED\r\n");
        }
        /* HAL_BUSY => not time yet */
    }

    (void)s_bsec_state_loaded;
    return HAL_OK;
}

HAL_StatusTypeDef air_app_get(air_readings_t *out)
{
    if (!out) return HAL_ERROR;
    *out = s_latest;
    return HAL_OK;
}
