/**
* Copyright (c) 2024 JThießen. All rights reserved.
*
*
* @file       lsm303dlh.h
* @date       2024-04-24
* @version    v1.0.0
*
*/

#ifndef _LSM303DHL_H
#define _LSM303DHL_H

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

/********************************************************************/
/* header files */

#include <stdint.h>
#include <stddef.h>

/****************************************************************************/
/*! @name       Common macros                                               */
/****************************************************************************/


/****************************************************************************/
/*! @name        Compiler switch macros Definitions                         */
/****************************************************************************/


/******************************************************************************/
/*! @name        General Macro Definitions                */
/******************************************************************************/
/*! @name API success code */
#define LSM303DLH_OK                                 INT8_C(0)

/*! @name To define TRUE or FALSE */


/*! @name API error codes */
#define LSM303DLH_E_NULL_PTR                         INT8_C(-1)
#define LSM303DLH_E_DEV_NOT_FOUND                    INT8_C(-2)
#define LSM303DLH_E_INVALID_CONFIG                   INT8_C(-3)
#define LSM303DLH_E_COM_FAIL                         INT8_C(-4)

/*! @name API warning codes */


/*! @name I2C ADDRESS       */

//For linear acceleration the default (factory) 7-bit slave address is 0011001b.
//Read 0011001 1 00110011 (33h)
//Write 0011001 0 00110010 (32h)
#define LSM303DLH_ACC_I2C_ADDRESS                UINT8_C(0x32)

//For magnetic sensors the default (factory) 7-bit slave address is 0011110xb.
//Read 0011110 1 00111101 (3Dh)
//Write 0011110 0 00111100 (3Ch)
#define LSM303DLH_MAG_I2C_ADDRESS                UINT8_C(0x3C)


#define LSM303DLH_CTRL_REG1_A               0x20
#define LSM303DLH_CTRL_REG2_A               0x21
#define LSM303DLH_CTRL_REG3_A               0x22
#define LSM303DLH_CTRL_REG4_A               0x23
#define LSM303DLH_CTRL_REG5_A               0x24
#define LSM303DLH_CTRL_REG6_A               0x25
#define LSM303DLH_REFERENCE_A               0x26
#define LSM303DLH_STATUS_REG_A              0x27
#define LSM303DLH_OUT_X_L_A                 0x28
#define LSM303DLH_OUT_X_H_A                 0x29
#define LSM303DLH_OUT_Y_L_A                 0x2A
#define LSM303DLH_OUT_Y_H_A                 0x2B
#define LSM303DLH_OUT_Z_L_A                 0x2C
#define LSM303DLH_OUT_Z_H_A                 0x2D
#define LSM303DLH_FIFO_CTRL_REG_A           0x2E
#define LSM303DLH_FIFO_SRC_REG_A            0x2F
#define LSM303DLH_INT1_CFG_A                0x30
#define LSM303DLH_INT1_SRC_A                0x31
#define LSM303DLH_INT1_THS_A                0x32
#define LSM303DLH_INT1_DURATION_A           0x33
#define LSM303DLH_INT2_CFG_A                0x34
#define LSM303DLH_INT2_SRC_A                0x35
#define LSM303DLH_INT2_THS_A                0x36
#define LSM303DLH_INT2_DURATION_A           0x37
#define LSM303DLH_CLICK_CFG_A               0x38
#define LSM303DLH_CLICK_SRC_A               0x39
#define LSM303DLH_CLICK_THS_A               0x3A
#define LSM303DLH_TIME_LIMIT_A              0x3B
#define LSM303DLH_TIME_LATENCY_A            0x3C
#define LSM303DLH_TIME_WINDOW_A             0x3D

#define LSM303DLH_CRA_REG_M                 0x00
#define LSM303DLH_CRB_REG_M                 0x01
#define LSM303DLH_MR_REG_M                  0x02
#define LSM303DLH_OUT_X_H_M                 0x03
#define LSM303DLH_OUT_X_L_M                 0x04
#define LSM303DLH_OUT_Z_H_M                 0x05
#define LSM303DLH_OUT_Z_L_M                 0x06
#define LSM303DLH_OUT_Y_H_M                 0x07
#define LSM303DLH_OUT_Y_L_M                 0x08
#define LSM303DLH_SR_REG_M                  0x09
#define LSM303DLH_IRA_REG_M                 0x0A
#define LSM303DLH_IRB_REG_M                 0x0B
#define LSM303DLH_IRC_REG_M                 0x0C

#define LSM303DLH_TEMP_OUT_H_M              0x31
#define LSM303DLH_TEMP_OUT_L_M              0x32

/******************************************************************************/
/*! @name           Function Pointers                             */
/******************************************************************************/
typedef int8_t (*lsm303dlh_read_fptr_t)(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
typedef int8_t (*lsm303dlh_write_fptr_t)(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
typedef void (*lsm303dlh_delay_us_fptr_t)(uint32_t period);

/******************************************************************************/
/*!  @name         Structure Declarations                             */
/******************************************************************************/

typedef enum
{
	LSM303DLH_ACC_RATE_0	= 0x00,		// PowerDown
    LSM303DLH_ACC_RATE_1 	= 0x01,		//
    LSM303DLH_ACC_RATE_10  	= 0x02,		//
    LSM303DLH_ACC_RATE_25  	= 0x03,		//
    LSM303DLH_ACC_RATE_50  	= 0x04,		//
    LSM303DLH_ACC_RATE_100  = 0x05,		//
    LSM303DLH_ACC_RATE_200  = 0x06,		// 		//
    LSM303DLH_ACC_RATE_400  = 0x07,		//
    LSM303DLH_ACC_RATE_1620 = 0x08,		// LowPower Mode
	LSM303DLH_ACC_RATE_5376 = 0x09		// Normal 1344 / LowPow 5376
} lsm303dlh_acc_datarate_t;

typedef enum
{
    LSM303DLH_ACC_NORMAL_MODE  = 0,
    LSM303DLH_ACC_LOW_POWER    = 1
} lsm303dlh_acc_mode_t;

typedef enum
{
    LSM303DLH_CONTINUOUS_MODE  = 0,
    LSM303DLH_SINGLE_TRIGGER   = 1,
    LSM303DLH_POWER_DOWN       = 2
} lsm303dlh_mag_mode_t;

typedef enum
{
    LSM303DLH_MAG_G_1_3 = 1,
    LSM303DLH_MAG_G_1_9 = 2,
    LSM303DLH_MAG_G_2_5 = 3,
    LSM303DLH_MAG_G_4_0 = 4,
    LSM303DLH_MAG_G_4_7 = 5,
    LSM303DLH_MAG_G_5_6 = 6,
    LSM303DLH_MAG_G_8_1 = 7
} lsm303dlh_mag_gain_t;

typedef enum
{
    LSM303DLH_MAG_RATE_0_7  = 0x00,  // 0.75 Hz
    LSM303DLH_MAG_RATE_1_5  = 0x01,  // 1.5 Hz
    LSM303DLH_MAG_RATE_3_0  = 0x02,  // 3.0 Hz
    LSM303DLH_MAG_RATE_7_5  = 0x03,  // 7.5 Hz
    LSM303DLH_MAG_RATE_15   = 0x04,  // 15 Hz
    LSM303DLH_MAG_RATE_30   = 0x05,  // 30 Hz
    LSM303DLH_MAG_RATE_75   = 0x06,  // 75 Hz
    LSM303DLH_MAG_RATE_220  = 0x07   // 200 Hz
} lsm303dlh_mag_datarate_t;

typedef enum
{
    LSM303DLH_HIGH_RESOLUTION  = 0,
    LSM303DLH_LOW_POWER        = 1,
} LSM303DLH_lp_m_t;

struct lsm303dlh_acc_data
{
    int16_t x;
    int16_t y;
    int16_t z;
};

struct lsm303dlh_mag_data
{
    int16_t x;
    int16_t y;
    int16_t z;
};

typedef struct
{
  int32_t x;
  int32_t y;
  int32_t z;
} LSM303DLH_Axes_t;

/*!
 * @brief lsm303dlh device structure
 */
struct lsm303dlh_dev
{
    uint8_t chip_id;

    uint8_t dev_addr;

    /*! Variable that holds result of read/write function */
    int8_t rslt;

    /*! Bus read function pointer */
    lsm303dlh_read_fptr_t read;

    /*! Bus write function pointer */
    lsm303dlh_write_fptr_t write;

    /*! delay(in us) function pointer */
    lsm303dlh_delay_us_fptr_t delay_us;

};

/********************************************************************/
/* (extern) variable declarations */
/********************************************************************/


/* function prototype declarations */

int8_t lsm303dlh_acc_init(lsm303dlh_acc_mode_t mode, struct lsm303dlh_dev *dev);
int8_t lsm303dlh_acc_set_regs(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, struct lsm303dlh_dev *dev);
int8_t lsm303dlh_acc_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, struct lsm303dlh_dev *dev);
int8_t lsm303dlh_acc_read_data(struct lsm303dlh_acc_data *acc_data, struct lsm303dlh_dev *dev);

int8_t lsm303dlh_mag_init(lsm303dlh_mag_mode_t mode, struct lsm303dlh_dev *dev);
int8_t lsm303dlh_mag_pow_mode(lsm303dlh_mag_mode_t mode, struct lsm303dlh_dev *dev);
int8_t lsm303dlh_mag_set_regs(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, struct lsm303dlh_dev *dev);
int8_t lsm303dlh_mag_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, struct lsm303dlh_dev *dev);

int8_t lsm303dlh_mag_enableTemperature(uint8_t enable, struct lsm303dlh_dev *dev);
int8_t lsm303dlh_mag_setGain(lsm303dlh_mag_gain_t gain, struct lsm303dlh_dev *dev);
int8_t lsm303dlh_mag_setDatarate(lsm303dlh_mag_datarate_t rate, struct lsm303dlh_dev *dev);

int8_t lsm303dlh_mag_read_data(struct lsm303dlh_mag_data *mag_data, struct lsm303dlh_dev *dev);
int8_t lsm303dlh_temp_read_data(uint16_t *temperature, struct lsm303dlh_dev *dev);

int8_t lsm303dlh_getDataReadyStatus(uint8_t *ready, struct lsm303dlh_dev *dev);

#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif /* _lsm303dlh_H */
