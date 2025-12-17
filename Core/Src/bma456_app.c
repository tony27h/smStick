/**
  ******************************************************************************
  * @file    bma456_app.c
  * @brief   BMA456 accelerometer application implementation
  *          Implements movement/impact detection using high-g interrupt
  *          
  * Hardware Configuration:
  *   - BMA456 connected via I2C1 at address 0x18
  *   - BMA456 INT1 pin connected to MCU PA9 (configured as EXTI with pull-down)
  *   - LED on PB1 (LED_YELLO_Pin)
  *   - Timer TIM16 used for LED timeout
  *   - UART1 for force reporting
  * 
  * Behavior:
  *   - On high-g detection (>~2g), INT1 goes high
  *   - PA9 interrupt handler turns on PB1 LED
  *   - Accelerometer data is read and force magnitude calculated
  *   - Force value and axis components sent via UART
  *   - TIM16 is started for 5-second timeout
  *   - After 5 seconds, LED is turned off
  *   - If new event occurs while LED is on, timer is restarted (retriggerable)
  ******************************************************************************
  */

#include "bma456_app.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

/* Private variables */
static struct bma4_dev bma456_dev;
static I2C_HandleTypeDef *bma456_hi2c = NULL;
static UART_HandleTypeDef *bma456_huart = NULL;
extern TIM_HandleTypeDef htim16;
static volatile uint8_t led_timer_active = 0;

/* UART timeout for transmit */
#define UART_TIMEOUT_MS  200

/* Private function prototypes */
static BMA4_INTF_RET_TYPE bma456_i2c_read(uint8_t reg_addr, uint8_t *read_data, uint32_t len, void *intf_ptr);
static BMA4_INTF_RET_TYPE bma456_i2c_write(uint8_t reg_addr, const uint8_t *write_data, uint32_t len, void *intf_ptr);
static void bma456_delay_us(uint32_t period, void *intf_ptr);

/**
  * @brief  BMA456 I2C read callback
  * @param  reg_addr: Register address to read from
  * @param  read_data: Pointer to buffer to store read data
  * @param  len: Number of bytes to read
  * @param  intf_ptr: Interface pointer (I2C handle)
  * @retval 0 for success, non-zero for failure
  */
static BMA4_INTF_RET_TYPE bma456_i2c_read(uint8_t reg_addr, uint8_t *read_data, uint32_t len, void *intf_ptr)
{
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)intf_ptr;
    HAL_StatusTypeDef status;

    /* Read from I2C device */
    status = HAL_I2C_Mem_Read(hi2c, (BMA456_I2C_ADDR << 1), reg_addr,
                               I2C_MEMADD_SIZE_8BIT, read_data, len, HAL_MAX_DELAY);

    return (status == HAL_OK) ? 0 : -1;
}

/**
  * @brief  BMA456 I2C write callback
  * @param  reg_addr: Register address to write to
  * @param  write_data: Pointer to data to write
  * @param  len: Number of bytes to write
  * @param  intf_ptr: Interface pointer (I2C handle)
  * @retval 0 for success, non-zero for failure
  */
static BMA4_INTF_RET_TYPE bma456_i2c_write(uint8_t reg_addr, const uint8_t *write_data, uint32_t len, void *intf_ptr)
{
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)intf_ptr;
    HAL_StatusTypeDef status;

    /* Write to I2C device */
    status = HAL_I2C_Mem_Write(hi2c, (BMA456_I2C_ADDR << 1), reg_addr,
                                I2C_MEMADD_SIZE_8BIT, (uint8_t *)write_data, len, HAL_MAX_DELAY);

    return (status == HAL_OK) ? 0 : -1;
}

/**
  * @brief  BMA456 delay callback
  * @param  period: Delay period in microseconds
  * @param  intf_ptr: Interface pointer (not used)
  * @retval None
  */
static void bma456_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;

    /* Convert microseconds to milliseconds for HAL_Delay */
    uint32_t delay_ms = period / 1000;
    if (delay_ms == 0 && period > 0) {
        delay_ms = 1;  /* Minimum 1ms delay for HAL_Delay */
    }

    if (delay_ms > 0) {
        HAL_Delay(delay_ms);
    }
}

/**
  * @brief  Initialize BMA456 accelerometer
  * @param  hi2c: Pointer to I2C handle
  * @param  huart: Pointer to UART handle for force reporting
  * @retval HAL status
  */
HAL_StatusTypeDef bma456_app_init(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart)
{
    int8_t rslt;
    struct bma456mm_high_g_config high_g_config;

    if (hi2c == NULL || huart == NULL) {
        return HAL_ERROR;
    }

    bma456_hi2c = hi2c;
    bma456_huart = huart;

    /* Initialize BMA4 device structure */
    bma456_dev.intf = BMA4_I2C_INTF;
    bma456_dev.bus_read = bma456_i2c_read;
    bma456_dev.bus_write = bma456_i2c_write;
    bma456_dev.delay_us = bma456_delay_us;
    bma456_dev.intf_ptr = hi2c;
    bma456_dev.variant = BMA42X_VARIANT;
    bma456_dev.read_write_len = 32;  /* Typical maximum read/write length */

    /* Initialize BMA456MM sensor */
    rslt = bma456mm_init(&bma456_dev);
    if (rslt != BMA4_OK) {
        return HAL_ERROR;
    }

    /* Write config file to enable sensor features - CRITICAL STEP! */
    rslt = bma456mm_write_config_file(&bma456_dev);
    if (rslt != BMA4_OK) {
        return HAL_ERROR;
    }

    /* Wait for sensor to be ready */
    HAL_Delay(10);

    /* Configure accelerometer: 2g range, 100Hz ODR */
    struct bma4_accel_config accel_config;
    accel_config.odr = BMA4_OUTPUT_DATA_RATE_100HZ;
    accel_config.range = BMA4_ACCEL_RANGE_2G;
    accel_config.bandwidth = BMA4_ACCEL_NORMAL_AVG4;
    accel_config.perf_mode = BMA4_CONTINUOUS_MODE;

    rslt = bma4_set_accel_config(&accel_config, &bma456_dev);
    if (rslt != BMA4_OK) {
        return HAL_ERROR;
    }

    /* Enable accelerometer */
    rslt = bma4_set_accel_enable(BMA4_ENABLE, &bma456_dev);
    if (rslt != BMA4_OK) {
        return HAL_ERROR;
    }

    /* Disable advance power save mode - REQUIRED for INT pin output! */
    rslt = bma4_set_advance_power_save(BMA4_DISABLE, &bma456_dev);
    if (rslt != BMA4_OK) {
        return HAL_ERROR;
    }

    /* Wait for power mode to stabilize */
    HAL_Delay(5);

    /* Configure high-g detection
     * Threshold: ~2g (in 5.11g format)
     * Duration: 10 samples at 100Hz = 100ms
     * Hysteresis: ~0.5g
     * Enable all axes (X, Y, Z)
     */
    high_g_config.threshold = BMA456_HIGH_G_THRESHOLD;
    high_g_config.duration = BMA456_HIGH_G_DURATION;
    high_g_config.hysteresis = BMA456_HIGH_G_HYSTERESIS;
    high_g_config.axes_en = BMA456MM_HIGH_G_EN_ALL_AXIS;

    rslt = bma456mm_set_high_g_config(&high_g_config, &bma456_dev);
    if (rslt != BMA4_OK) {
        return HAL_ERROR;
    }

    /* Also configure any-motion detection for testing (more sensitive) */
    struct bma456mm_any_no_mot_config any_mot_config;
    any_mot_config.threshold = 20;  /* Lower threshold in 5.11g format (~0.16g) */
    any_mot_config.duration = 5;    /* 5 samples at 50Hz = 100ms */
    any_mot_config.axes_en = BMA456MM_EN_ALL_AXIS;
    any_mot_config.intr_bhvr = 0;
    any_mot_config.slope = 0;

    (void)bma456mm_set_any_mot_config(&any_mot_config, &bma456_dev);

    /* Configure INT1 pin FIRST: push-pull, active high, output enabled */
    struct bma4_int_pin_config int_config;
    int_config.edge_ctrl = BMA4_LEVEL_TRIGGER;  /* Level for latched mode */
    int_config.lvl = BMA4_ACTIVE_HIGH;
    int_config.od = BMA4_PUSH_PULL;
    int_config.output_en = BMA4_OUTPUT_ENABLE;
    int_config.input_en = BMA4_INPUT_DISABLE;

    rslt = bma4_set_int_pin_config(&int_config, BMA4_INTR1_MAP, &bma456_dev);
    if (rslt != BMA4_OK) {
        return HAL_ERROR;
    }

    /* Keep the "force write" safety (not UART debug): ensures INT1_IO_CTRL is correct */
    uint8_t int1_ctrl_verify;
    (void)bma4_read_regs(0x53, &int1_ctrl_verify, 1, &bma456_dev);  /* 0x53 = INT1_IO_CTRL */
    if (int1_ctrl_verify != 0x0A) {
        uint8_t int1_ctrl_val = 0x0A;  /* output_en=1, active_high=1, push-pull=0 */
        (void)bma4_write_regs(0x53, &int1_ctrl_val, 1, &bma456_dev);
        HAL_Delay(1);
        (void)bma4_read_regs(0x53, &int1_ctrl_verify, 1, &bma456_dev);
    }

    /* Set latched interrupt mode */
    rslt = bma4_set_interrupt_mode(BMA4_LATCH_MODE, &bma456_dev);
    if (rslt != BMA4_OK) {
        return HAL_ERROR;
    }

    /* Enable high-g feature */
    rslt = bma456mm_feature_enable(BMA456MM_HIGH_G, BMA4_ENABLE, &bma456_dev);
    if (rslt != BMA4_OK) {
        return HAL_ERROR;
    }

    /* Small delay to ensure feature registers are updated before mapping interrupts */
    HAL_Delay(10);

    /* Map high-g interrupt to INT1 pin */
    rslt = bma456mm_map_interrupt(BMA4_INTR1_MAP, BMA456MM_HIGH_G_INT, BMA4_ENABLE, &bma456_dev);
    if (rslt != BMA4_OK) {
        return HAL_ERROR;
    }

    /* Map any-motion interrupt to INT1 (optional / for testing) */
    (void)bma456mm_map_interrupt(BMA4_INTR1_MAP, BMA456MM_ANY_MOT_INT, BMA4_ENABLE, &bma456_dev);

    return HAL_OK;
}

/**
  * @brief  Handle BMA456 interrupt (called from EXTI callback)
  *         Turns on LED, reads accelerometer data, and sends force via UART
  * @retval None
  */
void bma456_app_handle_interrupt(void)
{
    uint16_t int_status;
    int8_t rslt;
    struct bma4_accel accel_data;

    /* Read and clear interrupt status */
    rslt = bma456mm_read_int_status(&int_status, &bma456_dev);

    /* Check if high-g OR any-motion interrupt occurred */
    if ((rslt == BMA4_OK) && (int_status & (BMA456MM_HIGH_G_INT | BMA456MM_ANY_MOT_INT))) {

        /* Turn on LED (active LOW - RESET=ON) */
        HAL_GPIO_WritePin(LED_YELLO_GPIO_Port, LED_YELLO_Pin, GPIO_PIN_RESET);

        /* Read current accelerometer data */
        rslt = bma4_read_accel_xyz(&accel_data, &bma456_dev);

        if (rslt == BMA4_OK && bma456_huart != NULL) {
            /* Convert raw accelerometer data to g-force
             * For 2g range: LSB = 16384 counts/g
             * Formula: g = (raw_value / 16384.0)
             */
            float accel_x_g = accel_data.x / 16384.0f;
            float accel_y_g = accel_data.y / 16384.0f;
            float accel_z_g = accel_data.z / 16384.0f;

            /* Calculate magnitude of acceleration vector */
            float magnitude_g = sqrtf(accel_x_g * accel_x_g +
                                      accel_y_g * accel_y_g +
                                      accel_z_g * accel_z_g);

            /* Send force data via UART */
            char uart_msg[80];
            int len = snprintf(uart_msg, sizeof(uart_msg),
                             "Impact detected! Force: %.2fg (X:%.2fg Y:%.2fg Z:%.2fg)\r\n",
                             magnitude_g, accel_x_g, accel_y_g, accel_z_g);

            (void)HAL_UART_Transmit(bma456_huart, (uint8_t*)uart_msg, (uint16_t)len, UART_TIMEOUT_MS);
        }

        /* Stop timer if already running (retriggerable behavior) */
        if (led_timer_active) {
            HAL_TIM_Base_Stop_IT(&htim16);
        }

        /* Reset and start timer for 5 seconds */
        __HAL_TIM_SET_COUNTER(&htim16, 0);
        HAL_TIM_Base_Start_IT(&htim16);
        led_timer_active = 1;
    }
}
//
/**
  * @brief  Timer callback to turn off LED after timeout
  *         Called from TIM16 interrupt handler
  * @retval None
  */
void bma456_app_timer_callback(void)
{
    /* Turn off LED (active LOW - SET=OFF) */
    HAL_GPIO_WritePin(LED_YELLO_GPIO_Port, LED_YELLO_Pin, GPIO_PIN_SET);

    /* Stop timer */
    HAL_TIM_Base_Stop_IT(&htim16);
    led_timer_active = 0;
}
