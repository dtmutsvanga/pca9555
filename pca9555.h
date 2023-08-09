/**
 * @file pca9555.h
 * @author dtm
 * @brief 
 * @version 0.1
 * @date 2023-06-14
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once
#include <stdint.h>
typedef enum 
{
  ePCA_PM_INPUT = 1,
  ePCA_PM_OUTPUT = 0,
}ePCA_PM;

typedef enum
{
  PCA_PIN_LOW,
  PCA_PIN_HIGH,
} ePCA_PIN_VAL;

typedef enum
{
  PCA_POL_NORM,
  PCA_PIN_INV
} ePCA_POL;

typedef struct 
{
  void *p_pvt;  /* Pointer to private internal data */
  uint8_t i2c_addr;
  uint8_t initd;
} pca_t;

uint8_t pca_init(pca_t *self);
uint8_t pca_cfgPin(pca_t *self, uint8_t pin, ePCA_PM pm);
uint8_t pca_wrtPin(pca_t *self, uint8_t pin, ePCA_PIN_VAL val);
uint8_t pca_rdPin(pca_t *self, uint8_t pin, ePCA_PIN_VAL *val);
uint8_t pca_sync(pca_t *self);
uint8_t pca_stop(pca_t *self);