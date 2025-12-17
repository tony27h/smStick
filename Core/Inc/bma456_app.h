/**
  ******************************************************************************
  * @file    bma456_app.h
  * @brief   BMA456 accelerometer application header file
  *          Implements movement/impact detection using high-g interrupt
  ******************************************************************************
  */

#ifndef BMA456_APP_H
#define BMA456_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "bma4.h"
#include "bma456mm.h"

/* BMA456 I2C address (7-bit) */
#define BMA456_I2C_ADDR           0x18

/* High-g detection threshold in 5.11g format (~2g)
 * Formula: threshold_value = desired_g * 1365.33 (since 3072 = 2.25g per datasheet)
 * For 2g: 2 * 1365.33 = 2731
 */
#define BMA456_HIGH_G_THRESHOLD   2731

/* High-g duration in 100Hz samples (10ms per sample)
 * 10 samples = 100ms
 */
#define BMA456_HIGH_G_DURATION    10

/* High-g hysteresis in 0.74g format
 * For ~0.5g hysteresis: (0.5 / 0.74) * 4096 = ~2770
 */
#define BMA456_HIGH_G_HYSTERESIS  2770

/* LED on duration in milliseconds */
#define BMA456_LED_ON_DURATION_MS 5000

/* Function prototypes */
HAL_StatusTypeDef bma456_app_init(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart);
void bma456_app_handle_interrupt(void);
void bma456_app_timer_callback(void);

#ifdef __cplusplus
}
#endif

#endif /* BMA456_APP_H */
