#include "lsm303dlh.h"

/************************** Internal macros *******************************/

/********************** Static function declarations ************************/

/********************** Global function definitions ************************/

int8_t lsm303dlh_acc_init(lsm303dlh_acc_mode_t mode, struct lsm303dlh_dev *dev)
{
    /* Write data from the reg_addr */
	uint8_t cfg = (LSM303DLH_ACC_RATE_400 << 4) | (mode << 3) | 0x07; // XYZ enabled
    dev->rslt = dev->write(dev->dev_addr, LSM303DLH_CTRL_REG1_A, &cfg, 1);

    return dev->rslt;
}

/*!
 * @brief This API writes the given data to the register address
 * of the sensor.
 */
int8_t lsm303dlh_acc_set_regs(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, struct lsm303dlh_dev *dev)
{
    /* Write data from the reg_addr */
    dev->rslt = dev->write(dev->dev_addr, reg_addr, reg_data, len);
    return dev->rslt;
}

/*!
 * @brief This API reads the data from the given register address of the sensor.
 */
int8_t lsm303dlh_acc_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, struct lsm303dlh_dev *dev)
{
    /* Read the data from the reg_addr */
    dev->rslt = dev->read(dev->dev_addr, reg_addr, reg_data, len);
    return dev->rslt;
}

int8_t lsm303dlh_acc_read_data(struct lsm303dlh_acc_data *acc_data, struct lsm303dlh_dev *dev)
{
	uint8_t data[6];
	dev->rslt = dev->read(dev->dev_addr, LSM303DLH_OUT_X_L_A, data, 6);

	acc_data->x = (int16_t)( ((uint16_t)data[1] << 8) | (uint16_t)data[0] );
	acc_data->y = (int16_t)( ((uint16_t)data[2] << 8) | (uint16_t)data[3] );
	acc_data->z = (int16_t)( ((uint16_t)data[4] << 8) | (uint16_t)data[5] );

	return dev->rslt;
}


int8_t lsm303dlh_mag_init(lsm303dlh_mag_mode_t mode, struct lsm303dlh_dev *dev)
{
	uint8_t cfg = 0x00;

	// Temp_EN; Dataoutputrate = 75Hz
	//cfg = 0x98;
	//dev->rslt = dev->write(dev->dev_addr, LSM303DLH_CRA_REG_M, &cfg, 1);

	// Gain 1.3 Gauss
	//cfg = 0x20;
	//dev->rslt += dev->write(dev->dev_addr, LSM303DLH_CRB_REG_M, &cfg, 1);

	// POwer Mode
	dev->rslt += lsm303dlh_mag_pow_mode(mode, dev);

    return dev->rslt;
}

int8_t lsm303dlh_mag_pow_mode(lsm303dlh_mag_mode_t mode, struct lsm303dlh_dev *dev)
{
    dev->rslt = dev->write(dev->dev_addr, LSM303DLH_MR_REG_M, &mode, 1);
    return dev->rslt;
}

int8_t lsm303dlh_mag_set_regs(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, struct lsm303dlh_dev *dev)
{
    /* Write data from the reg_addr */
    dev->rslt = dev->write(dev->dev_addr, reg_addr, reg_data, len);
    return dev->rslt;
}

int8_t lsm303dlh_mag_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, struct lsm303dlh_dev *dev)
{
    /* Read the data from the reg_addr */
    dev->rslt = dev->read(dev->dev_addr, reg_addr, reg_data, len);
    return dev->rslt;
}

int8_t lsm303dlh_mag_enableTemperature(uint8_t enable, struct lsm303dlh_dev *dev)
{
    uint8_t data = 0;
    dev->rslt = dev->read(dev->dev_addr, LSM303DLH_CRA_REG_M, &data, 1);
    if(enable)  data = data | 0x80;
    dev->rslt = dev->write(dev->dev_addr, LSM303DLH_CRA_REG_M, &data, 1);

    return dev->rslt;
}

int8_t lsm303dlh_mag_setDatarate(lsm303dlh_mag_datarate_t rate, struct lsm303dlh_dev *dev)
{
    uint8_t data = 0;
    dev->rslt = dev->read(dev->dev_addr, LSM303DLH_CRA_REG_M, &data, 1);
    data &= 0x80;
    data = data | (rate << 2);
    dev->rslt = dev->write(dev->dev_addr, LSM303DLH_CRA_REG_M, &data, 1);

    return dev->rslt;
}

int8_t lsm303dlh_mag_setGain(lsm303dlh_mag_gain_t gain, struct lsm303dlh_dev *dev)
{
	gain = gain << 5;
    dev->rslt = dev->write(dev->dev_addr, LSM303DLH_CRB_REG_M, &gain, 1);
    return dev->rslt;
}

int8_t lsm303dlh_mag_read_data(struct lsm303dlh_mag_data *mag_data, struct lsm303dlh_dev *dev)
{
    uint8_t data[10];
    dev->rslt = dev->read(dev->dev_addr, LSM303DLH_OUT_X_H_M, data, 10);

    mag_data->x = (int16_t)( ((uint16_t)data[0] << 8) | (uint16_t)data[1] );
    mag_data->z = (int16_t)( ((uint16_t)data[2] << 8) | (uint16_t)data[3] );
    mag_data->y = (int16_t)( ((uint16_t)data[4] << 8) | (uint16_t)data[5] );
    
    return dev->rslt;
}

int8_t lsm303dlh_temp_read_data(uint16_t *temperature, struct lsm303dlh_dev *dev)
{
    uint8_t reg_data[2] = {0, 0};
    dev->rslt = dev->read(dev->dev_addr, LSM303DLH_TEMP_OUT_H_M, reg_data, 2);

    *temperature = ( ((uint16_t)reg_data[0] << 8) | (uint16_t)reg_data[1] );
    return dev->rslt;
}

int8_t lsm303dlh_getDataReadyStatus(uint8_t *ready, struct lsm303dlh_dev *dev)
{
    dev->rslt = dev->read(dev->dev_addr, LSM303DLH_SR_REG_M, ready, 1);

    *ready = *ready & 0x01;

    return dev->rslt;
}
