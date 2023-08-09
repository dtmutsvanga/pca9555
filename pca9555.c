/**
 * @file pca9555.c
 * @author dtm
 * @brief 
 * @version 0.1
 * @date 2023-06-14
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "pca9555.h"
#include"pca9555_hal.h"
#define LITTLE_ENDIAN 1

#define MAX_PCM_DRVR  MAX_PCA_NUM /* Used for statically allocating memory to the PCA9555 drivers */

typedef enum
{
  PCA_REG_INPUT = 0,
  PCA_REG_OUTPUT = 2,
  PCA_REG_POL_INV = 4,
  PCA_REG_CFG = 6
} ePCA_REG;

#if LITTLE_ENDIAN
/**
 * This struct is a helper struct for easier debugging.
 * It is used to display the state of the registers in a human-readable way. 
 * The fields are named according to the PCA9555 datasheet.
 * As this uses bitfields, it is NOT platformn independant!
*/
typedef struct {
  /* Register 0 */
  uint8_t reg0_0 : 1; 
  uint8_t reg0_1 : 1; 
  uint8_t reg0_2 : 1;
  uint8_t reg0_3 : 1;
  uint8_t reg0_4 : 1;
  uint8_t reg0_5 : 1;
  uint8_t reg0_6 : 1;
  uint8_t reg0_7 : 1;

  /* Register 1*/
  uint8_t reg1_0 : 1;
  uint8_t reg1_1 : 1;
  uint8_t reg1_2 : 1;
  uint8_t reg1_3 : 1;
  uint8_t reg1_4 : 1;
  uint8_t reg1_5 : 1;
  uint8_t reg1_6 : 1;
  uint8_t reg1_7 : 1;
}regPair_t;

#else // big endian
typedef struct {
  /* Register 0 */
  uint8_t reg0_7 : 1;
  uint8_t reg0_6 : 1;
  uint8_t reg0_5 : 1;
  uint8_t reg0_4 : 1;
  uint8_t reg0_3 : 1;
  uint8_t reg0_2 : 1;
  uint8_t reg0_1 : 1;
  uint8_t reg0_0 : 1;

  /* Register 1*/
  uint8_t reg1_7 : 1;
  uint8_t reg1_6 : 1;
  uint8_t reg1_5 : 1;
  uint8_t reg1_4 : 1;
  uint8_t reg1_3 : 1;
  uint8_t reg1_2 : 1;
  uint8_t reg1_1 : 1;
  uint8_t reg1_0 : 1;
}regPair_t;
#endif // LITTLE_ENDIAN

/**
 * @brief Register pair class
 * 
 */
typedef union 
{
  uint8_t u8_regs[2];                           /* All registers in the PCA are organized into 2- byte register pairs */
  uint16_t u16_regPair;                         /* Convenience u16 */
  regPair_t rp;                                 /* Convenience, for ease of debugging */
} regPair_u;

/**
 * @brief Flags for synchronization
 * @details Used for synchronizing the internal registers with the hardware.
 * When run is called, if a flag is true, the relevant register in the hardware is updated.
 */
typedef struct
{
  uint8_t in : 1;                               /* Input flag */
  uint8_t out : 1;                              /* Output flag */
  uint8_t polInv : 1;                           /* polarity inversion flag */
  uint8_t cfg : 1;                              /* Configuration change flag */
} syncFlags_t;

typedef union 
{
  syncFlags_t flgs;                             /* Synchronization flags */
  uint8_t raw;                                  /* Raw value (for ease of determining if there is at least a single true flag) */
} syncFlags_u;

/**
 * @brief PCA registers
 * 
 */
typedef struct
{
  regPair_u in;                                 /* Input registers */
  regPair_u out;                                /* Output registers */
  regPair_u polInv;                             /* Polarity inversion registers*/
  regPair_u cfg;                                /* Configuration registers*/
} pcaRegs_t;

/**
 * @brief PCA driver class, internal
 * 
 */
typedef struct 
{
  pcaRegs_t regs;                               /* Registers of the PCM */
  syncFlags_u regsSyncFlgs;                     /* Flags indicating wheter to synchronize a particular register or not when calling run */
  pcahal_t hal;
} pca_drvr_t;

static pca_drvr_t pcaDrvrs[MAX_PCM_DRVR]; /* Static array for  */
static uint8_t m_initd_pca = 0;

/**
 * @brief Write a register
 * @param hal the hardware abstraction layer
 * @param reg the register to write
 * @param val the value to write to the register
 * @return returns 0 if the write failed, 1 otherwise
*/
static uint8_t pca_writeReg(pcahal_t *hal, uint8_t reg, uint16_t val)
{
  static uint8_t data[3] = {0};
  data[0] = reg;
  data[1] = (uint8_t)(val & 0x00FF);
  data[2] = (uint8_t)(val >> 8U);
  return pcahal_write(hal, data, 3U);
}

/**
 * @brief read a register
 * @param hal the hardware abstraction layer
 * @param reg the register to read
 * @param pval pointer to the variable where the register value will be stored
 * @return returns 0 if the read failed, 1 otherwise
*/
static uint8_t pca_readReg(pcahal_t *hal, uint8_t reg, uint16_t *pval)
{
  static uint8_t data[3] = {0};
  data[0] = reg;
  data[1] = *pval & 0x00FF;
  data[2] = *pval >> 8;
  if(!pcahal_read(hal, data, 3U))
    return 0;

  *pval = (data[1]) | data[2] << 8;
  return 1;
}

/**
 * @brief Initialize the PCA driver
 * @param self \ref pca_t driver object
 * @return returns 0 if initialization failed, 1 otherwise
*/
uint8_t pca_init(pca_t *self)
{
  if(m_initd_pca > MAX_PCM_DRVR){
    return 0;
  }
  /* Initialize i2c */
  if(!pcahal_init())
    return 0;
  pcaDrvrs[m_initd_pca].hal.address = self->i2c_addr;
  if(!pcahal_ctor(&pcaDrvrs[m_initd_pca].hal))
    return 0;
  pcaDrvrs[m_initd_pca].regsSyncFlgs.raw = 0;
  pcaDrvrs[m_initd_pca].regs.in.u16_regPair = 0xFFFF;     /* At device reset, all pins are configured as inputs, with high value pull-up to vdd */
  pcaDrvrs[m_initd_pca].regs.out.u16_regPair = 0xFFFF;    /* 1  by default */
  pcaDrvrs[m_initd_pca].regs.polInv.u16_regPair = 0x0000; /* 0 by default */
  pcaDrvrs[m_initd_pca].regs.cfg.u16_regPair = 0xFFFF;    /* Inputs configured by default */

  /* Hack to avoid compiler error on const */
  self->p_pvt = &pcaDrvrs[m_initd_pca];
  self->initd = 1;
  m_initd_pca++;
  return 1;
}

/**
 * @brief Run the PCA
 * @param self \ref pca_t driver object
 * @return returns 0 if pin number is invalid, 1 otherwise
*/

uint8_t pca_cfgPin(pca_t *self, uint8_t pin, ePCA_PM pm)
{
  pca_drvr_t *pdrvr = self->p_pvt;
  if(pin > 15)
    return 0;
  if(pm == ePCA_PM_INPUT) {
    pdrvr->regs.cfg.u16_regPair |= (1 << pin);
  } else {
    /* Set corresponding output pin to zero as well, so that pin is not high when output is configured */
    pdrvr->regs.out.u16_regPair &= ~(1 << pin);
    pdrvr->regs.cfg.u16_regPair &= ~(1 << pin);
  }
  pdrvr->regsSyncFlgs.flgs.cfg = 1;
  return 1;
}

/**
 * @brief Write a value to a pin
 * @param self \ref pca_t driver object
 * @param pin pin number
 * @param val the value of the pin
 * @return returns 0 if pin number is invalid, 1 otherwise
*/
uint8_t pca_wrtPin(pca_t *self, uint8_t pin, ePCA_PIN_VAL val)
{
  pca_drvr_t *pdrvr = self->p_pvt;
  if(pin > 15)
    return 0;
  if(val == PCA_PIN_HIGH)
    pdrvr->regs.out.u16_regPair |= (1 << pin);
  else
    pdrvr->regs.out.u16_regPair &= ~(1 << pin);
  pdrvr->regsSyncFlgs.flgs.out = 1;
  return 1;
}

/**
 * @brief Read the state of a pin
 * @param self \ref pca_t driver object
 * @param pin pin number
 * @param val the value of the pin
 * @return always returns 1
*/
uint8_t pca_rdPin(pca_t *self, uint8_t pin, ePCA_PIN_VAL *val)
{
  pca_drvr_t *pdrvr = self->p_pvt;
  if(pin > 15)
    return 0;
  if(pdrvr->regs.in.u16_regPair & (1 << pin))
    *val = PCA_PIN_HIGH;
  else
    *val = PCA_PIN_LOW;
  return 1;
}

uint8_t pca_setPol(pca_t *self, uint8_t pin, ePCA_POL pol)
{
  pca_drvr_t *pdrvr = self->p_pvt;
  if(pin > 15)
    return 0;
  if(pol == PCA_POL_NORM)
    pdrvr->regs.polInv.u16_regPair &= ~(1 << pin);
  else
    pdrvr->regs.polInv.u16_regPair |= (1 << pin);
  pdrvr->regsSyncFlgs.flgs.polInv = 1;
  return 1;
}

/**
 * @brief Synchronize the internal register states with the current states of the PCA hardware.
 * @detail This function checks the internal registers, then synchronizes the hardware with the internal registers. 
 * if there is at least one flag set to true, the hardware is updated.
 * @param self 
 * @return uint8_t 
 */
uint8_t pca_sync(pca_t *self)
{
  pca_drvr_t *pdrvr = self->p_pvt;
  uint8_t success = 1;
  uint8_t inputs = 0;
  uint16_t regval = 0;
  if(pdrvr->regsSyncFlgs.raw)
  {
    /* Output */
    if(pdrvr->regsSyncFlgs.flgs.out)
    {
      success &= pca_writeReg(&pdrvr->hal, PCA_REG_OUTPUT, pdrvr->regs.out.u16_regPair);
    }
    pcahal_delay_ms(1);
    /* Configuration */
    if (pdrvr->regsSyncFlgs.flgs.cfg) {
      success &= pca_writeReg(&pdrvr->hal, PCA_REG_CFG, pdrvr->regs.cfg.u16_regPair);
    }
    pcahal_delay_ms(1);
    /* Polarity inversion */
    if (pdrvr->regsSyncFlgs.flgs.polInv) {
      success &= pca_writeReg(&pdrvr->hal, PCA_REG_POL_INV, pdrvr->regs.polInv.u16_regPair);
    }
      /* Read inputs if read attempt was made */
    if(pdrvr->regsSyncFlgs.flgs.in)
    {
      inputs = 1;
      success &= pca_readReg(&pdrvr->hal, PCA_REG_INPUT, &pdrvr->regs.in.u16_regPair);
    }
    /* Clear all flags */
    pdrvr->regsSyncFlgs.raw = 0;
    pdrvr->regsSyncFlgs.flgs.in = inputs;       /* Reset input reads, if inputs are configured */
  }
  return success;
}

uint8_t pca_stop(pca_t *self)
{
  pca_drvr_t *pdrvr = self->p_pvt;
  pcahal_abortTx(&pdrvr->hal);
  return 0;
}