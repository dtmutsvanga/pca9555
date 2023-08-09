/**
 * @file pca9555_hal.h
 * @author dtm
 * @brief hardware abstraction layer for the pca9555 
 * @version 0.1
 * @date 2023-06-14
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#define MAX_PCA_NUM 2

typedef struct 
{
  void *instance;
  uint8_t address;
} pcahal_t;

/**
 * @brief Initializes the low level I2C driver
 * 
 * @return uint8_t 1 if successful
 */
uint8_t pcahal_init(void);

/**
 * @brief Constructor for pcahal_t, initializes the instance of self
 * 
 * @param self pointer to the statically allocated pcahal_ type
 * @return uint8_t 1 if successful, 0 if otherwise
 * @note The maximum number of instances to be created is defined by @ref MAX_PCA_NUM
 */
uint8_t pcahal_ctor(pcahal_t *self);

/**
 * @brief Read the data from the PCA.
 * @details The register to be read should be in pbuff[0]
 * @param hal 
 * @param pbuff 
 * @param bufflen 
 * @return uint8_t 
 */
uint8_t pcahal_read(pcahal_t *hal, uint8_t *pbuff, size_t bufflen);

/**
 * @brief Write data to the pca
 * @details the register to the written to should be in pdata[0]
 * @param hal 
 * @param pdata 
 * @param datalen 
 * @return uint8_t 
 */
uint8_t pcahal_write(pcahal_t *hal, uint8_t *pdata, size_t datalen);

/**
 * @brief Delay milliseconds 
 * 
 * @param ms 
 */
void pcahal_delay_ms(uint32_t ms);
/**
 * @brief Abort transmission. To handle bus hangs 
 * 
 * @param hal 
 */
void pcahal_abortTx(pcahal_t *hal);