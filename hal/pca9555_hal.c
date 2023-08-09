/**
 * @file pca9555_hal.c
 * @author dtm
 * @brief hardware abstraction layer for the pca9555 
 * @version 0.1
 * @date 2023-06-14
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "pca9555_hal.h"
#include "flexio_i2c1.h"
#include "projDefines.h"
#include "pinout.h"
#include "FreeRTOS.h"
#include "task.h"
#include "interrupt_manager.h"

#define USE_BLOCKING_TRNSFR 0
#define SDA_PIN 4U
#define SCL_PIN 5U
#define FREQ_Hz 100000U
#define ENABLE_CRITICAL 1
#if ENABLE_CRITICAL
#define __enter_critical portENTER_CRITICAL()
#define __exit_critical portEXIT_CRITICAL() 
#else 
#define __enter_critical (void)0;
#define __exit_critical (void)0;
#endif 
#define SET_DEFAULT_MASTER_CFG(master_cfg, slv_addr)           \
    {                                                          \
        master_cfg.driverType = FLEXIO_DRIVER_TYPE_INTERRUPTS; \
        master_cfg.baudRate = FREQ_Hz;                         \
        master_cfg.slaveAddress = slv_addr;                    \
        master_cfg.sdaPin = SDA_PIN;                           \
        master_cfg.sclPin = SCL_PIN;                           \
        master_cfg.callback = NULL;                            \
        master_cfg.callbackParam = NULL;                       \
        master_cfg.txDMAChannel = 255;                         \
        master_cfg.rxDMAChannel = 255;                         \
    }
/**
 * @brief class for holding master cfg and state for s32k144
 *
 */
typedef struct 
{
    flexio_i2c_master_user_config_t masterConfig;
    flexio_i2c_master_state_t masterState;
    uint8_t isInitialized;
} s32k144_flexio_t;

static flexio_device_state_t flexIODeviceState; /* State of the I2C device */
static s32k144_flexio_t i2c_master[MAX_PCA_NUM] = {0};  /* State of the Master devices */
static int m_initd_masters = 0;                     /* Number of initialized / used master configurations */

static uint8_t init_i2c(void)
{
    uint8_t ret = (STATUS_SUCCESS == FLEXIO_DRV_InitDevice(INST_FLEXIO_I2C1, &flexIODeviceState));
    uint8_t prio = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY+1;
    INT_SYS_SetPriority(FLEXIO_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY+1);
    return ret;
}

static uint8_t init_master_device(s32k144_flexio_t *pmaster, uint8_t slv_addr)
{
    SET_DEFAULT_MASTER_CFG(pmaster->masterConfig, slv_addr);
    return STATUS_SUCCESS == FLEXIO_I2C_DRV_MasterInit(INST_FLEXIO_I2C1, &pmaster->masterConfig, &pmaster->masterState);
}

static uint8_t i2c_read(pcahal_t *hal, uint8_t *data, uint32_t len)
{
    s32k144_flexio_t *pmaster = hal->instance;
    if(pmaster->isInitialized == 0)
    {
        return 0;
    }
    /* Send register address first, which is 1st byte of data. No stop */
#if USE_BLOCKING_TRNSFR
    FLEXIO_I2C_DRV_MasterSendDataBlocking(&pmaster->masterState, data, 1, false, 20);
    /* Then read data */
    return  STATUS_SUCCESS == FLEXIO_I2C_DRV_MasterReceiveDataBlocking(&pmaster->masterState, &data[1], len-1, true, 20);
#else
    FLEXIO_I2C_DRV_MasterSendData(&pmaster->masterState, data, 1, false);
    /* Then read data */
    return  STATUS_SUCCESS == FLEXIO_I2C_DRV_MasterReceiveData(&pmaster->masterState, &data[1], len-1, true);
#endif // USE_BLOCKING_TRNSFR
    
}

static uint8_t i2c_write(pcahal_t *hal, uint8_t *data, uint32_t len)
{
    s32k144_flexio_t *pmaster = hal->instance;
    if(pmaster->isInitialized == 0)
        return 0;
    #if USE_BLOCKING_TRNSFR
     return STATUS_SUCCESS == FLEXIO_I2C_DRV_MasterSendDataBlocking(&pmaster->masterState, data, len, true, 20);
     #else 
     return STATUS_SUCCESS == FLEXIO_I2C_DRV_MasterSendData(&pmaster->masterState, data, len, true);
     #endif
}

uint8_t pcahal_init(void)
{
    static int initd = 0;
    if(initd)
    {
        return 1;
    }
    initd = 1; 
    /* Power up the device */

    /* Init FlexIO device */
    return init_i2c();
}

uint8_t pcahal_ctor(pcahal_t* self) {
    if (m_initd_masters >= MAX_PCA_NUM) {
        return 0;
    }
    if (!init_master_device(&i2c_master[m_initd_masters], self->address)) {
        return 0;
    }
    i2c_master[m_initd_masters].isInitialized = 1;
    self->instance = &i2c_master[m_initd_masters];
    m_initd_masters++;
    return 1;
}

uint8_t pcahal_read(pcahal_t* hal, uint8_t* pbuff, size_t bufflen) {
    uint8_t ret = 0;
    __enter_critical;
    ret = i2c_read(hal, pbuff, bufflen);
    __exit_critical;
    return ret;
}

uint8_t pcahal_write(pcahal_t* hal, uint8_t* pdata, size_t datalen) {
    uint8_t ret = 0;
    __enter_critical;
    ret = i2c_write(hal, pdata, datalen);
    __exit_critical;
    return ret;
}

void pcahal_delay_ms(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

void pcahal_abortTx(pcahal_t *hal)
{
    s32k144_flexio_t *pmaster = hal->instance;
    FLEXIO_I2C_DRV_MasterTransferAbort(&pmaster->masterState);
}