/**
  ******************************************************************************
  * @file    bma456_app.h
  * @brief   BMA456 accelerometer application header file
  *          Implements impact detection using any-motion interrupt with software filter
  *          - Any-motion: Hardware interrupt for motion detection
  *          - Software filter: Only triggers LED/UART for impacts >2g
  *          - 16g measurement range for accurate high-force measurements
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
