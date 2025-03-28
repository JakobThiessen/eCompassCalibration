/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "../../Drivers/LSM303AGR/lsm303agr.h"
#include "../../Drivers/BMI323_SensorAPI-main/bmi323.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*! Earth's gravity in m/s^2 */
#define GRAVITY_EARTH  (9.80665f)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_DRD_FS;

/*! Variable that holds the I2C device address or SPI chip selection */
static uint8_t dev_addr;

/* USER CODE BEGIN PV */
static struct lsm303agr_dev Sensor;
static struct bmi3_dev acc_dev;
static struct bmi3_sensor_data acc_data;
/* Structure to define accelerometer configuration. */
static struct bmi3_sens_config acc_config;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ICACHE_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/*!
 *  @brief This internal API is used to set configurations for accel.
 *
 *  @param[in] dev       : Structure instance of bmi3_dev.
 *
 *  @return Status of execution.
 */
static int8_t set_accel_config(struct bmi3_dev *dev);

/*!
 *  @brief This internal function converts lsb to meter per second squared for 16 bit accelerometer for
 *  range 2G, 4G, 8G or 16G.
 *
 *  @param[in] val       : LSB from each axis.
 *  @param[in] g_range   : Gravity range.
 *  @param[in] bit_width : Resolution for accel.
 *
 *  @return Accel values in meter per second squared.
 */
static float lsb_to_mps2(int16_t val, int8_t g_range, uint8_t bit_width);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int8_t i2c_1_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data,
		uint16_t len) {
	int8_t result;

	result = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) dev_addr, &reg_addr, 1,
			10);
	result += HAL_I2C_Master_Receive(&hi2c1, (uint16_t) dev_addr, data, len,
			10);
	return result;
}

int8_t i2c_1_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data,
		uint16_t len) {
	int8_t result;
	uint8_t buffer[128];

	buffer[0] = reg_addr;
	memcpy(&buffer[1], data, len);

	result = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) dev_addr, buffer,
			len + 1, 100);
	return result;
}

//BMI3_INTF_RET_TYPE (*bmi3_read_fptr_t)(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
//BMI3_INTF_RET_TYPE (*bmi3_write_fptr_t)(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);

int8_t i2c_1_read_bmi(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr) {
	int8_t result;

    uint8_t device_addr = *(uint8_t*)intf_ptr;
    (void)intf_ptr;

	result = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) device_addr, &reg_addr, 1, 10);
	result += HAL_I2C_Master_Receive(&hi2c1, (uint16_t) device_addr, data, len, 10);
	return result;
}

int8_t i2c_1_write_bmi(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr) {
	int8_t result;
	uint8_t buffer[128];

    uint8_t device_addr = *(uint8_t*)intf_ptr;
    (void)intf_ptr;

	buffer[0] = reg_addr;
	memcpy(&buffer[1], data, len);

	result = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) device_addr, buffer, len + 1, 100);
	return result;
}

//typedef void (*bmi3_delay_us_fptr_t)(uint32_t period, void *intf_ptr);
void custom_us_delay(uint32_t period, void *intf_ptr)
{
	HAL_Delay(period/1000);
}

/*!
 * @brief This internal API is used to set configurations for accel.
 */
static int8_t set_accel_config(struct bmi3_dev *dev)
{
    /* Status of API are returned to this variable. */
    int8_t rslt;

    /* Structure to define accelerometer configuration. */
    struct bmi3_sens_config config;

    /* Structure to map interrupt */
    struct bmi3_map_int map_int = { 0 };

    /* Configure the type of feature. */
    config.type = BMI323_ACCEL;

    /* Get default configurations for the type of feature selected. */
    rslt = bmi323_get_sensor_config(&config, 1, dev);
    //bmi3_error_codes_print_result("bmi323_get_sensor_config", rslt);

    if (rslt == BMI323_OK)
    {
        map_int.acc_drdy_int = BMI3_INT1;

        /* Map data ready interrupt to interrupt pin. */
        rslt = bmi323_map_interrupt(map_int, dev);
        //bmi3_error_codes_print_result("bmi323_map_interrupt", rslt);

        if (rslt == BMI323_OK)
        {
            /* NOTE: The user can change the following configuration parameters according to their requirement. */
            /* Output Data Rate. By default ODR is set as 100Hz for accel. */
            config.cfg.acc.odr = BMI3_ACC_ODR_100HZ;

            /* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G). */
            config.cfg.acc.range = BMI3_ACC_RANGE_2G;

            /* The Accel bandwidth coefficient defines the 3 dB cutoff frequency in relation to the ODR. */
            config.cfg.acc.bwp = BMI3_ACC_BW_ODR_QUARTER;

            /* Set number of average samples for accel. */
            config.cfg.acc.avg_num = BMI3_ACC_AVG64;

            /* Enable the accel mode where averaging of samples
             * will be done based on above set bandwidth and ODR.
             * Note : By default accel is disabled. The accel will get enable by selecting the mode.
             */
            config.cfg.acc.acc_mode = BMI3_ACC_MODE_NORMAL;

            /* Set the accel configurations. */
            rslt = bmi323_set_sensor_config(&config, 1, dev);
            //bmi3_error_codes_print_result("bmi323_set_sensor_config", rslt);
        }
    }

    return rslt;
}

/*!
 * @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 * range 2G, 4G, 8G or 16G.
 */
static float lsb_to_mps2(int16_t val, int8_t g_range, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (GRAVITY_EARTH * val * g_range) / half_scale;
}


/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */
	uint8_t data[100];
	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_ICACHE_Init();
	MX_USART1_UART_Init();
	MX_USART3_UART_Init();
	MX_USB_PCD_Init();
	MX_I2C1_Init();
	/* USER CODE BEGIN 2 */

	Sensor.dev_addr_mag = LSM303AGR_MAG_I2C_ADDRESS;
	Sensor.dev_addr_acc = LSM303AGR_ACC_I2C_ADDRESS;
	Sensor.read = i2c_1_read;
	Sensor.write = i2c_1_write;

	struct lsm303agr_mag_data_raw magData_raw;
	struct lsm303agr_mag_offset magOffsetData;
	struct lsm303agr_acc_data accData;

	int8_t res = lsm303agr_mag_init(LSM303AGR_CONTINUOUS, &Sensor);
	sprintf((char*) data, "init mag: ok...: %d\n\r", res);
	//HAL_UART_Transmit(&huart3, data, (uint16_t)strlen((char*)data), 10);
	res = lsm303agr_mag_setDatarate(LSM303AGR_MAG_ODR_100HZ, &Sensor);

	res = lsm303agr_mag_read_offset(&magOffsetData, &Sensor);

	sprintf((char*) data, "MAG offset: [x,y,z]: %d, %d, %d\n\r", magOffsetData.x, magOffsetData.y, magOffsetData.z);
	HAL_UART_Transmit(&huart3, data, (uint16_t)strlen((char*)data), 10);


	magOffsetData.x = -88;
	magOffsetData.y = 136;
	magOffsetData.z = 23;

	res = lsm303agr_acc_init(LSM303AGR_ACC_NORMAL_MODE, &Sensor);

	sprintf((char*) data, "init acc: ok...: %d\n\r", res);
	HAL_UART_Transmit(&huart3, data, (uint16_t)strlen((char*)data), 10);

	/* Select accel sensor. */
	acc_data.type = BMI323_ACCEL;
	dev_addr = 0xD0; //BMI3_ADDR_I2C_PRIM;
	/* Assign device address to interface pointer */
	acc_dev.intf_ptr = &dev_addr;

	acc_dev.read = i2c_1_read_bmi;
    acc_dev.write = i2c_1_write_bmi;
    acc_dev.delay_us = custom_us_delay;
    acc_dev.intf = BMI3_I2C_INTF;


	int8_t rslt_1 = bmi323_init(&acc_dev);

	if (rslt_1 == BMI323_OK) {
		/* Accel configuration settings. */
		rslt_1 = set_accel_config(&acc_dev);

		if (rslt_1 == BMI323_OK) {
			rslt_1 = bmi323_get_sensor_config(&acc_config, 1, &acc_dev);
			//bmi3_error_codes_print_result("bmi323_get_sensor_config", rslt);
		}
	}

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	sprintf((char*) data, "Begin Search I2C: \n\r");
	HAL_UART_Transmit(&huart3, data, (uint16_t) strlen((char*) data), 10);

	for (uint8_t i = 1; i < 250; i++) {
		uint8_t temp = 0;
		int8_t result = 0;

		result = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t) i, &temp, 1, 100);

		if (result == 0) {
			sprintf((char*) data, "0x%02X ", i);
		} else {
			sprintf((char*) data, "  .  ");
		}
		HAL_UART_Transmit(&huart3, data, (uint16_t) strlen((char*) data), 10);

		if (i % 8 == 0 && i != 0)
			HAL_UART_Transmit(&huart3, "\n\r", 2, 10);
	}
	sprintf((char*) data, "\n\rEnd Search I2C: \n\r");
	HAL_UART_Transmit(&huart3, data, (uint16_t) strlen((char*) data), 10);

	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		uint16_t temperature;
		int8_t rslt = 0;

		rslt = lsm303agr_mag_read_data_raw(&magData_raw, &Sensor);

		uint8_t data_ready_status = 0;
		lsm303agr_acc_get_DataReadyStatus(&data_ready_status, &Sensor);
		if (data_ready_status != 0) {
			lsm303agr_acc_read_data(&accData, &Sensor);
			rslt += lsm303agr_temp_read_data(&temperature, &Sensor);
		}

	    float x = 0, y = 0, z = 0;
	    /* Initialize the interrupt status of accel. */
	    uint16_t int_status = 0;

		/* To get the status of accel data ready interrupt. */
		rslt = bmi323_get_int1_status(&int_status, &acc_dev);
		//bmi3_error_codes_print_result("bmi323_get_int1_status", rslt);

		/* To check the accel data ready interrupt status and print the status for 100 samples. */
		if (int_status & BMI3_INT_STATUS_ACC_DRDY)
		{
			/* Get accelerometer data for x, y and z axis. */
			rslt = bmi323_get_sensor_data(&acc_data, 1, &acc_dev);
			//bmi3_error_codes_print_result("Get sensor data", rslt);

			/* Converting lsb to meter per second squared for 16 bit accelerometer at 2G range. */
			x = lsb_to_mps2(acc_data.sens_data.acc.x, 2, acc_dev.resolution);
			y = lsb_to_mps2(acc_data.sens_data.acc.y, 2, acc_dev.resolution);
			z = lsb_to_mps2(acc_data.sens_data.acc.z, 2, acc_dev.resolution);
/*
				   sensor_data.sens_data.acc.x,
				   sensor_data.sens_data.acc.y,
				   sensor_data.sens_data.acc.z,
				   x,
				   y,
				   z;*/
		}


		//sprintf((char*)data, "ACC: %d %d %d %05d %05d %05d TMP: %05d\n\r", accData.x, accData.y, accData.z, magData.x, magData.y, magData.z, temperature );
		sprintf((char*) data,
				"RAW: %06d, %06d, %06d, %05d, %05d, %05d, %05d, %05d, %05d, %05d, %.03f, %.03f, %.03f\n\r",
				accData.x, accData.y, accData.z, 0, 0, 0, magData_raw.x, magData_raw.y,
				magData_raw.z, temperature, x, y, z);
		HAL_UART_Transmit(&huart3, data, (uint16_t) strlen((char*) data), 10);
		HAL_Delay(100);
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

	while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
	}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48
			| RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLL1_SOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 12;
	RCC_OscInitStruct.PLL.PLLN = 250;
	RCC_OscInitStruct.PLL.PLLP = 2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1_VCIRANGE_1;
	RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
	RCC_OscInitStruct.PLL.PLLFRACN = 0;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_PCLK3;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x60808CD3;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief ICACHE Initialization Function
 * @param None
 * @retval None
 */
static void MX_ICACHE_Init(void) {

	/* USER CODE BEGIN ICACHE_Init 0 */

	/* USER CODE END ICACHE_Init 0 */

	/* USER CODE BEGIN ICACHE_Init 1 */

	/* USER CODE END ICACHE_Init 1 */

	/** Enable instruction cache (default 2-ways set associative cache)
	 */
	if (HAL_ICACHE_Enable() != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ICACHE_Init 2 */

	/* USER CODE END ICACHE_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart3) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * @brief USB Initialization Function
 * @param None
 * @retval None
 */
static void MX_USB_PCD_Init(void) {

	/* USER CODE BEGIN USB_Init 0 */

	/* USER CODE END USB_Init 0 */

	/* USER CODE BEGIN USB_Init 1 */

	/* USER CODE END USB_Init 1 */
	hpcd_USB_DRD_FS.Instance = USB_DRD_FS;
	hpcd_USB_DRD_FS.Init.dev_endpoints = 8;
	hpcd_USB_DRD_FS.Init.speed = USBD_FS_SPEED;
	hpcd_USB_DRD_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
	hpcd_USB_DRD_FS.Init.Sof_enable = DISABLE;
	hpcd_USB_DRD_FS.Init.low_power_enable = DISABLE;
	hpcd_USB_DRD_FS.Init.lpm_enable = DISABLE;
	hpcd_USB_DRD_FS.Init.battery_charging_enable = DISABLE;
	hpcd_USB_DRD_FS.Init.vbus_sensing_enable = DISABLE;
	hpcd_USB_DRD_FS.Init.bulk_doublebuffer_enable = DISABLE;
	hpcd_USB_DRD_FS.Init.iso_singlebuffer_enable = DISABLE;
	if (HAL_PCD_Init(&hpcd_USB_DRD_FS) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USB_Init 2 */

	/* USER CODE END USB_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(USB_FS_PWR_EN_GPIO_Port, USB_FS_PWR_EN_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_BLUE_USER_BUTTON_Pin */
	GPIO_InitStruct.Pin = B1_BLUE_USER_BUTTON_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_BLUE_USER_BUTTON_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : USER_LED_Pin */
	GPIO_InitStruct.Pin = USER_LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(USER_LED_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : USB_FS_VBUS_Pin */
	GPIO_InitStruct.Pin = USB_FS_VBUS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(USB_FS_VBUS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : USB_FS_PWR_EN_Pin */
	GPIO_InitStruct.Pin = USB_FS_PWR_EN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(USB_FS_PWR_EN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : USB_FS_OVCR_Pin */
	GPIO_InitStruct.Pin = USB_FS_OVCR_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(USB_FS_OVCR_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
