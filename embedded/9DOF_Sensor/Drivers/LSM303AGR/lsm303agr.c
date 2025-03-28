#include "lsm303agr.h"

/************************** Internal macros *******************************/

/********************** Static function declarations ************************/

/********************** Global function definitions ************************/
int8_t lsm303agr_acc_init(lsm303agr_acc_mode_t mode, struct lsm303agr_dev *dev)
{
	uint8_t cfg = 0;
	dev->rslt += dev->read(dev->dev_addr_acc, LSM303AGR_WHO_AM_I_A, &cfg, 1);
	if(LSM303AGR_WHO_AM_I_A_VALUE != cfg)
	{
		dev->rslt = LSM303AGR_E_DEV_NOT_FOUND;
	}
	else
	{
		/* Write data from the reg_addr */
		cfg = (LSM303AGR_ACC_RATE_100 << 4) | (mode << 3) | 0x07; // XYZ enabled
		dev->rslt = dev->write(dev->dev_addr_acc, LSM303AGR_CTRL_REG1_A, &cfg, 1);

		cfg = 0;
		if(mode == 0)	cfg = 0x08;	//HighResolution mode enable;
		dev->rslt = dev->write(dev->dev_addr_acc, LSM303AGR_CTRL_REG4_A, &cfg, 1);

		lsm303agr_mag_enableTemperature(1, dev);

		cfg = 0x01;
		dev->rslt = dev->write(dev->dev_addr_acc, LSM303AGR_Act_THS_A, &cfg, 1);
		dev->rslt = dev->write(dev->dev_addr_acc, LSM303AGR_Act_DUR_A, &cfg, 1);
	}
    return dev->rslt;
}

int8_t lsm303agr_mag_enableTemperature(uint8_t enable, struct lsm303agr_dev *dev)
{
    uint8_t data = 0;
    if(enable > 0)	data = 0xC0;
    dev->rslt = dev->write(dev->dev_addr_acc, LSM303AGR_TEMP_CFG_REG_A, &data, 1);

    return dev->rslt;
}

int8_t lsm303agr_temp_read_data(uint16_t *temperature, struct lsm303agr_dev *dev)
{
    uint8_t reg_data[2] = {0, 0};
    dev->rslt = dev->read(dev->dev_addr_acc, LSM303AGR_OUT_TEMP_L_A, reg_data, 2);

    *temperature = (uint16_t)( ((uint16_t)reg_data[1] << 8) | (uint16_t)reg_data[0] );
    return dev->rslt;
}

/*!
 * @brief This API writes the given data to the register address
 * of the sensor.
 */
int8_t lsm303agr_acc_set_regs(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, struct lsm303agr_dev *dev)
{
    /* Write data from the reg_addr */
    dev->rslt = dev->write(dev->dev_addr_acc, reg_addr, reg_data, len);
    return dev->rslt;
}

/*!
 * @brief This API reads the data from the given register address of the sensor.
 */
int8_t lsm303agr_acc_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, struct lsm303agr_dev *dev)
{
    /* Read the data from the reg_addr */
    dev->rslt = dev->read(dev->dev_addr_acc, reg_addr, reg_data, len);
    return dev->rslt;
}

int8_t lsm303agr_acc_get_DataReadyStatus(uint8_t *data_rdy, struct lsm303agr_dev *dev)
{
	uint8_t data;
	dev->rslt = dev->read(dev->dev_addr_acc, LSM303AGR_STATUS_REG_A, &data, 1);
	*data_rdy = data & 0x04;
	return dev->rslt;
}

int8_t lsm303agr_acc_read_data(struct lsm303agr_acc_data *acc_data, struct lsm303agr_dev *dev)
{
	uint8_t data[6];
	dev->rslt = dev->read(dev->dev_addr_acc, LSM303AGR_OUT_X_L_A, data, 6);

	acc_data->x = (int16_t)( ((uint16_t)data[1] << 8) | (uint16_t)data[0] ) / 16;
	acc_data->y = (int16_t)( ((uint16_t)data[2] << 8) | (uint16_t)data[3] ) / 16;
	acc_data->z = (int16_t)( ((uint16_t)data[4] << 8) | (uint16_t)data[5] ) / 16;

	return dev->rslt;
}


int8_t lsm303agr_mag_init(lsm303agr_mag_mode_t mode, struct lsm303agr_dev *dev)
{
	uint8_t cfg = 0x00;

	dev->rslt += dev->read(dev->dev_addr_mag, LSM303AGR_WHO_AM_I_M, &cfg, 1);
	if(LSM303AGR_WHO_AM_I_M_VALUE != cfg)
	{
		dev->rslt = LSM303AGR_E_DEV_NOT_FOUND;
	}
	else
	{
		mode = 0x80 | mode;	// set temperature compensation
		mode &= 0x8F;
		dev->rslt += lsm303agr_mag_pow_mode(mode, dev);
	}

    return dev->rslt;
}

int8_t lsm303agr_mag_pow_mode(lsm303agr_mag_mode_t mode, struct lsm303agr_dev *dev)
{
    dev->rslt = dev->write(dev->dev_addr_mag, LSM303AGR_CFG_REG_A_M, &mode, 1);
    return dev->rslt;
}

int8_t lsm303agr_mag_set_regs(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, struct lsm303agr_dev *dev)
{
    /* Write data from the reg_addr */
    dev->rslt = dev->write(dev->dev_addr_mag, reg_addr, reg_data, len);
    return dev->rslt;
}

int8_t lsm303agr_mag_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, struct lsm303agr_dev *dev)
{
    /* Read the data from the reg_addr */
    dev->rslt = dev->read(dev->dev_addr_mag, reg_addr, reg_data, len);
    return dev->rslt;
}

int8_t lsm303agr_mag_setDatarate(lsm303agr_mag_datarate_t rate, struct lsm303agr_dev *dev)
{
    uint8_t data = 0;
    dev->rslt = dev->read(dev->dev_addr_mag, LSM303AGR_CFG_REG_A_M, &data, 1);
    data &= 0xF3;
    data = data | (rate << 2);
    dev->rslt = dev->write(dev->dev_addr_mag, LSM303AGR_CFG_REG_A_M, &data, 1);

    return dev->rslt;
}

int8_t lsm303agr_mag_read_data(struct lsm303agr_mag_data *mag_data, struct lsm303agr_dev *dev)
{
    uint8_t data[6];
    dev->rslt = dev->read(dev->dev_addr_mag, LSM303AGR_OUTX_L_REG_M, data, 6);

    mag_data->x = (int32_t)( ((uint16_t)data[1] << 8) | (uint16_t)data[0] );
    mag_data->y = (int32_t)( ((uint16_t)data[3] << 8) | (uint16_t)data[2] );
    mag_data->z = (int32_t)( ((uint16_t)data[5] << 8) | (uint16_t)data[4] );

    float sensitivity = 1.5f;

    mag_data->x = mag_data->x * sensitivity;
	mag_data->y = mag_data->y * sensitivity;
	mag_data->z = mag_data->z * sensitivity;
    
    return dev->rslt;
}

int8_t lsm303agr_mag_read_data_raw(struct lsm303agr_mag_data_raw *mag_data_raw, struct lsm303agr_dev *dev)
{
    uint8_t data[6];
    dev->rslt = dev->read(dev->dev_addr_mag, LSM303AGR_OUTX_L_REG_M, data, 6);

    mag_data_raw->x = (int16_t)( ((uint16_t)data[1] << 8) | (uint16_t)data[0] );
    mag_data_raw->y = (int16_t)( ((uint16_t)data[3] << 8) | (uint16_t)data[2] );
    mag_data_raw->z = (int16_t)( ((uint16_t)data[5] << 8) | (uint16_t)data[4] );

    return dev->rslt;
}

int8_t lsm303agr_mag_read_offset(struct lsm303agr_mag_offset *offset, struct lsm303agr_dev *dev)
{
	uint8_t data[6];
	dev->rslt = dev->read(dev->dev_addr_mag, LSM303AGR_OFFSET_X_REG_L_M, data, 6);

	offset->x = (int16_t)( ((uint16_t)data[1] << 8) | (uint16_t)data[0] );
	offset->y = (int16_t)( ((uint16_t)data[3] << 8) | (uint16_t)data[2] );
	offset->z = (int16_t)( ((uint16_t)data[5] << 8) | (uint16_t)data[4] );

	return dev->rslt;
}

int8_t lsm303agr_mag_write_offset(struct lsm303agr_mag_offset *offset, struct lsm303agr_dev *dev)
{
	uint8_t data[6];
	dev->rslt = dev->read(dev->dev_addr_mag, LSM303AGR_OFFSET_X_REG_L_M, data, 6);

	offset->x = (int16_t)( ((uint16_t)data[1] << 8) | (uint16_t)data[0] );
	offset->y = (int16_t)( ((uint16_t)data[3] << 8) | (uint16_t)data[2] );
	offset->z = (int16_t)( ((uint16_t)data[5] << 8) | (uint16_t)data[4] );

	return dev->rslt;
}
