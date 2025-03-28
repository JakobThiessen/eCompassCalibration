/**
* Copyright (c) 2024 JThießen. All rights reserved.
*
*
* @file       lsm303agr.h
* @date       2024-04-24
* @version    v1.0.0
*
*/

#ifndef _LSM303AGR_H
#define _LSM303AGR_H

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
#define LSM303AGR_OK                                 INT8_C(0)

/*! @name To define TRUE or FALSE */


/*! @name API error codes */
#define LSM303AGR_E_NULL_PTR                         INT8_C(-1)
#define LSM303AGR_E_DEV_NOT_FOUND                    INT8_C(-2)
#define LSM303AGR_E_INVALID_CONFIG                   INT8_C(-3)
#define LSM303AGR_E_COM_FAIL                         INT8_C(-4)

/*! @name API warning codes */


/*! @name I2C ADDRESS       */

//For linear acceleration the default (factory) 7-bit slave address is 0011001b.
//Read 0011001 1 00110011 (33h)
//Write 0011001 0 00110010 (32h)
#define LSM303AGR_ACC_I2C_ADDRESS                UINT8_C(0x32)

//For magnetic sensors the default (factory) 7-bit slave address is 0011110xb.
//Read 0011110 1 00111101 (3Dh)
//Write 0011110 0 00111100 (3Ch)
#define LSM303AGR_MAG_I2C_ADDRESS                UINT8_C(0x3C)

#define LSM303AGR_STATUS_REG_AUX_A	0x07
#define LSM303AGR_OUT_TEMP_L_A		0x0C
#define LSM303AGR_OUT_TEMP_H_A		0x0D
#define	LSM303AGR_INT_COUNTER_REG_A	0x0E
#define	LSM303AGR_WHO_AM_I_A		0x0F
#define	LSM303AGR_TEMP_CFG_REG_A	0x1F
#define	LSM303AGR_CTRL_REG1_A		0x20
#define	LSM303AGR_CTRL_REG2_A		0x21
#define	LSM303AGR_CTRL_REG3_A		0x22
#define	LSM303AGR_CTRL_REG4_A		0x23
#define	LSM303AGR_CTRL_REG5_A		0x24
#define	LSM303AGR_CTRL_REG6_A		0x25
#define	LSM303AGR_DATACAPTURE_A		0x26
#define	LSM303AGR_STATUS_REG_A		0x27
#define	LSM303AGR_OUT_X_L_A			0x28
#define	LSM303AGR_OUT_X_H_A			0x29
#define	LSM303AGR_OUT_Y_L_A			0x2A
#define	LSM303AGR_OUT_Y_H_A			0x2B
#define	LSM303AGR_OUT_Z_L_A			0x2C
#define	LSM303AGR_OUT_Z_H_A			0x2D
#define	LSM303AGR_FIFO_CTRL_REG_A	0x2E
#define	LSM303AGR_FIFO_SRC_REG_A	0x2F
#define	LSM303AGR_INT1_CFG_A		0x30
#define	LSM303AGR_INT1_SRC_A		0x31
#define	LSM303AGR_INT1_THS_A		0x32
#define	LSM303AGR_INT1_DURATION_A	0x33
#define	LSM303AGR_INT2_CFG_A		0x34
#define	LSM303AGR_INT2_SRC_A		0x35
#define	LSM303AGR_INT2_THS_A		0x36
#define	LSM303AGR_INT2_DURATION_A	0x37
#define	LSM303AGR_CLICK_CFG_A		0x38
#define	LSM303AGR_CLICK_SRC_A		0x39
#define	LSM303AGR_CLICK_THS_A		0x3A
#define	LSM303AGR_TIME_LIMIT_A		0x3B
#define	LSM303AGR_TIME_LATENCY_A	0x3C
#define	LSM303AGR_TIME_WINDOW_A		0x3D
#define	LSM303AGR_Act_THS_A			0x3E
#define	LSM303AGR_Act_DUR_A			0x3F

#define	LSM303AGR_OFFSET_X_REG_L_M	0x45
#define	LSM303AGR_OFFSET_X_REG_H_M	0x46
#define	LSM303AGR_OFFSET_Y_REG_L_M	0x47
#define	LSM303AGR_OFFSET_Y_REG_H_M	0x48
#define	LSM303AGR_OFFSET_Z_REG_L_M	0x49
#define	LSM303AGR_OFFSET_Z_REG_H_M	0x4A
#define	LSM303AGR_WHO_AM_I_M		0x4F
#define	LSM303AGR_CFG_REG_A_M		0x60
#define	LSM303AGR_CFG_REG_B_M		0x61
#define	LSM303AGR_CFG_REG_C_M		0x62
#define	LSM303AGR_INT_CRTL_REG_M	0x63
#define	LSM303AGR_INT_SOURCE_REG_M	0x64
#define	LSM303AGR_INT_THS_L_REG_M	0x65
#define	LSM303AGR_INT_THS_H_REG_M	0x66
#define	LSM303AGR_STATUS_REG_M		0x67
#define	LSM303AGR_OUTX_L_REG_M		0x68
#define	LSM303AGR_OUTX_H_REG_M		0x69
#define	LSM303AGR_OUTY_L_REG_M		0x6A
#define	LSM303AGR_OUTY_H_REG_M		0x6B
#define	LSM303AGR_OUTZ_L_REG_M		0x6C
#define	LSM303AGR_OUTZ_H_REG_M		0x6D


#define LSM303AGR_WHO_AM_I_A_VALUE	0x33
#define LSM303AGR_WHO_AM_I_M_VALUE	0x40

/******************************************************************************/
/*! @name           Function Pointers                             */
/******************************************************************************/
typedef int8_t (*lsm303agr_read_fptr_t)(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
typedef int8_t (*lsm303agr_write_fptr_t)(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
typedef void (*lsm303agr_delay_us_fptr_t)(uint32_t period);

/******************************************************************************/
/*!  @name         Structure Declarations                             */
/******************************************************************************/

typedef enum
{
    LSM303AGR_TEMP_DISABLE  = 0x00,
    LSM303AGR_TEMP_EBABLE   = 0xC0
} lsm303agr_temp_mode_t;

typedef enum
{
	LSM303AGR_ACC_RATE_0	= 0x00,		// PowerDown
    LSM303AGR_ACC_RATE_1 	= 0x01,		//
    LSM303AGR_ACC_RATE_10  	= 0x02,		//
    LSM303AGR_ACC_RATE_25  	= 0x03,		//
    LSM303AGR_ACC_RATE_50  	= 0x04,		//
    LSM303AGR_ACC_RATE_100  = 0x05,		//
    LSM303AGR_ACC_RATE_200  = 0x06,		// 		//
    LSM303AGR_ACC_RATE_400  = 0x07,		//
    LSM303AGR_ACC_RATE_1620 = 0x08,		// LowPower Mode
	LSM303AGR_ACC_RATE_5376 = 0x09		// Normal 1344 / LowPow 5376
} lsm303agr_acc_datarate_t;

typedef enum
{
    LSM303AGR_ACC_NORMAL_MODE  = 0,
    LSM303AGR_ACC_LOW_POWER    = 1
} lsm303agr_acc_mode_t;



/* enum for Magnetometer */

typedef enum
{
	LSM303AGR_MAG_ODR_10HZ	= 0x00,		//
    LSM303AGR_MAG_ODR_20HZ_ = 0x01,		//
    LSM303AGR_MAG_ODR_50HZ  = 0x02,		//
	LSM303AGR_MAG_ODR_100HZ	= 0x02		//
} lsm303agr_mag_datarate_t;

typedef enum
{
    LSM303AGR_CONTINUOUS  	= 0,
    LSM303AGR_SINGLE   		= 1,
    LSM303AGR_IDLE_1       	= 2,
	LSM303AGR_IDLE_2		= 3
} lsm303agr_mag_mode_t;


typedef enum
{
    LSM303AGR_HIGH_RESOLUTION  = 0,
    LSM303AGR_LOW_POWER        = 1,
} LSM303AGR_lp_m_t;

struct lsm303agr_acc_data
{
    int16_t x;
    int16_t y;
    int16_t z;
};

struct lsm303agr_mag_data
{
    int32_t x;
    int32_t y;
    int32_t z;
};

struct lsm303agr_mag_data_raw
{
    int16_t x;
    int16_t y;
    int16_t z;
};

struct lsm303agr_mag_offset
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
} LSM303AGR_Axes_t;

/*!
 * @brief lsm303agr device structure
 */
struct lsm303agr_dev
{
    uint8_t dev_addr_acc;
    uint8_t dev_addr_mag;

    /*! Variable that holds result of read/write function */
    int8_t rslt;

    /*! Bus read function pointer */
    lsm303agr_read_fptr_t read;

    /*! Bus write function pointer */
    lsm303agr_write_fptr_t write;

    /*! delay(in us) function pointer */
    lsm303agr_delay_us_fptr_t delay_us;

};

/********************************************************************/
/* (extern) variable declarations */
/********************************************************************/


/* function prototype declarations */
int8_t lsm303agr_mag_enableTemperature(uint8_t enable, struct lsm303agr_dev *dev);
int8_t lsm303agr_temp_read_data(uint16_t *temperature, struct lsm303agr_dev *dev);

int8_t lsm303agr_acc_init(lsm303agr_acc_mode_t mode, struct lsm303agr_dev *dev);
int8_t lsm303agr_acc_set_regs(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, struct lsm303agr_dev *dev);
int8_t lsm303agr_acc_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, struct lsm303agr_dev *dev);
int8_t lsm303agr_acc_get_DataReadyStatus(uint8_t *data_rdy, struct lsm303agr_dev *dev);
int8_t lsm303agr_acc_read_data(struct lsm303agr_acc_data *acc_data, struct lsm303agr_dev *dev);

int8_t lsm303agr_mag_init(lsm303agr_mag_mode_t mode, struct lsm303agr_dev *dev);
int8_t lsm303agr_mag_pow_mode(lsm303agr_mag_mode_t mode, struct lsm303agr_dev *dev);
int8_t lsm303agr_mag_set_regs(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, struct lsm303agr_dev *dev);
int8_t lsm303agr_mag_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, struct lsm303agr_dev *dev);

int8_t lsm303agr_mag_setDatarate(lsm303agr_mag_datarate_t rate, struct lsm303agr_dev *dev);
int8_t lsm303agr_mag_read_data(struct lsm303agr_mag_data *mag_data, struct lsm303agr_dev *dev);
int8_t lsm303agr_mag_read_data_raw(struct lsm303agr_mag_data_raw *mag_data_raw, struct lsm303agr_dev *dev);

int8_t lsm303agr_mag_read_offset(struct lsm303agr_mag_offset *offset, struct lsm303agr_dev *dev);
int8_t lsm303agr_mag_write_offset(struct lsm303agr_mag_offset *offset, struct lsm303agr_dev *dev);

#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif /* _lsm303agr_H */
