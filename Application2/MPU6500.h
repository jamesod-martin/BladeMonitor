/*
 * MPU6500.h
 *
 *  Created on: Jul 30, 2015
 *      Author: jamesod
 */

#ifndef MPU6500_H_
#define MPU6500_H_

#include "Board.h"
#include <xdc/std.h>
#include <ti/drivers/i2c/I2CCC26XX.h>
#include <ti/drivers/I2C.h>
#include "bsp_i2c.h"
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>

// MPU6500 GPIO interface
#define AD0 24 //Same pin as SPI_RxD. Must be set to known state in I2C mode. Is GP-114=MRAA 24
			   //Will try to use MRAA spi mode anyways....
#define CS0 23  //Chip Select zero
#define MPU6500_I2C_ADD1 0b1101000	// 68 I2C address. Last bit depends on state of AD0
#define MPU6500_I2C_ADD2 0b1101001

//MPU6500 Registers
#define	SELF_TEST_X_GYRO 	0x00	//	R/W 	xg_st_data [7:0]
#define	SELF_TEST_Y_GYRO 	0x1		//	R/W 	yg_st_data [7:0]
#define	SELF_TEST_Z_GYRO 	0x2		//	R/W 	zg_st_data [7:0]
#define	SELF_TEST_X_ACCEL 	0x0D 	//	R/W 	XA_ST_DATA [7:0]
#define	SELF_TEST_Y_ACCEL 	0x0E 	//	R/W 	YA_ST_DATA [7:0]
#define	SELF_TEST_Z_ACCEL 	0x0F 	//	R/W 	ZA_ST_DATA [7:0]
#define	XG_OFFSET_H 		0x13	//	R/W 	X_OFFS_USR [15:8]
#define	XG_OFFSET_L 		0x14	//	R/W 	X_OFFS_USR [7:0]
#define	YG_OFFSET_H 		0x15	//	R/W 	Y_OFFS_USR [15:8]
#define	YG_OFFSET_L 		0x16	//	R/W 	Y_OFFS_USR [7:0]
#define	ZG_OFFSET_H 		0x17	//	R/W 	Z_OFFS_USR [15:8]
#define	ZG_OFFSET_L 		0x18	//	R/W 	Z_OFFS_USR [7:0]
#define	SMPLRT_DIV 			0x19	//	R/W 	SMPLRT_DIV[7:0]
#define	MPU6500_CONFIG		0x1A 	//	R/W
#define	GYRO_CONFIG 		0x1B 	//	R/W
#define	ACCEL_CONFIG 		0x1C 	//	R/W
#define	ACCEL_CONFIG2 		0x1D 	//	R/W
#define	LP_ACCEL_ODR 		0x1E 	//	R/W
#define	WOM_THR 			0x1F 	//	R/W
#define	FIFO_EN 			0x23	//	R/W
#define	I2C_MST_CTRL 		0x24	//	R/W
#define	I2C_SLV0_ADDR 		0x25	//	R/W
#define	I2C_SLV0_REG 		0x26	//	R/W
#define	I2C_SLV0_CTRL 		0x27	//	R/W
#define	I2C_SLV1_ADDR 		0x28	//	R/W
#define	I2C_SLV1_REG 		0x29	//	R/W
#define	I2C_SLV1_CTRL 		0x2A 	//	R/W
#define	I2C_SLV2_ADDR 		0x2B 	//	R/W
#define	I2C_SLV2_REG 		0x2C 	//	R/W
#define	I2C_SLV2_CTRL 		0x2D 	//	R/W
#define	I2C_SLV3_ADDR 		0x2E 	//	R/W
#define	I2C_SLV3_REG 		0x2F 	//	R/W
#define	I2C_SLV3_CTRL 		0x30	//	R/W
#define	I2C_SLV4_ADDR 		0x31	//	R/W
#define	I2C_SLV4_REG 		0x32	//	R/W
#define	I2C_SLV4_DO 		0x33	//	R/W
#define	I2C_SLV4_CTRL 		0x34	//	R/W
#define	I2C_SLV4_DI 		0x35	//	R
#define	I2C_MST_STATUS 		0x36	//	R
#define	INT_PIN_CFG 		0x37	//	R/W
#define	INT_ENABLE 			0x38	//	R/W
#define	INT_STATUS 			0x3A 	//	R
#define	ACCEL_XOUT_H 		0x3B 	//	R 	ACCEL_XOUT_H[15:8]
#define	ACCEL_XOUT_L 		0x3C 	//	R 	ACCEL_XOUT_L[7:0]
#define	ACCEL_YOUT_H 		0x3D 	//	R 	ACCEL_YOUT_H[15:8]
#define	ACCEL_YOUT_L 		0x3E 	//	R 	ACCEL_YOUT_L[7:0]
#define	ACCEL_ZOUT_H 		0x3F 	//	R 	ACCEL_ZOUT_H[15:8]
#define	ACCEL_ZOUT_L 		0x40	//	R 	ACCEL_ZOUT_L[7:0]
#define	TEMP_OUT_H 			0x41		//	R 	TEMP_OUT_H[15:8]
#define	TEMP_OUT_L 			0x42	//	R 	TEMP_OUT_L[7:0]
#define	GYRO_XOUT_H 		0x43	//	R 	GYRO_XOUT_H[15:8]
#define	GYRO_XOUT_L 		0x44	//	R 	GYRO_XOUT_L[7:0]
#define	GYRO_YOUT_H 		0x45	//	R 	GYRO_YOUT_H[15:8]
#define	GYRO_YOUT_L 		0x46	//	R 	GYRO_YOUT_L[7:0]
#define	GYRO_ZOUT_H 		0x47	//	R 	GYRO_ZOUT_H[15:8]
#define	GYRO_ZOUT_L 		0x48	//	R 	GYRO_ZOUT_L[7:0]
#define	EXT_SENS_DATA_00 	0x49	//	R 	EXT_SENS_DATA_00[7:0]
#define	EXT_SENS_DATA_01 	0x4A 	//	R 	EXT_SENS_DATA_01[7:0]
#define	EXT_SENS_DATA_02 	0x4B 	//	R 	EXT_SENS_DATA_02[7:0]
#define	EXT_SENS_DATA_03 	0x4C 	//	R 	EXT_SENS_DATA_03[7:0]
#define	EXT_SENS_DATA_04 	0x4D 	//	R 	EXT_SENS_DATA_04[7:0]
#define	EXT_SENS_DATA_05 	0x4E 	//	R 	EXT_SENS_DATA_05[7:0]
#define	EXT_SENS_DATA_06 	0x4F 	//	R 	EXT_SENS_DATA_06[7:0]
#define	EXT_SENS_DATA_07 	0x50	//	R 	EXT_SENS_DATA_07[7:0]
#define	EXT_SENS_DATA_08 	0x51	//	R 	EXT_SENS_DATA_08[7:0]
#define	EXT_SENS_DATA_09 	0x52	//	R 	EXT_SENS_DATA_09[7:0]
#define	EXT_SENS_DATA_10 	0x53	//	R 	EXT_SENS_DATA_10[7:0]
#define	EXT_SENS_DATA_11 	0x54	//	R 	EXT_SENS_DATA_11[7:0]
#define	EXT_SENS_DATA_12 	0x55	//	R 	EXT_SENS_DATA_12[7:0]
#define	EXT_SENS_DATA_13 	0x56	//	R 	EXT_SENS_DATA_13[7:0]
#define	EXT_SENS_DATA_14 	0x57	//	R 	EXT_SENS_DATA_14[7:0]
#define	EXT_SENS_DATA_15 	0x58	//	R 	EXT_SENS_DATA_15[7:0]
#define	EXT_SENS_DATA_16 	0x59	//	R 	EXT_SENS_DATA_16[7:0]
#define	EXT_SENS_DATA_17 	0x5A 	//	R 	EXT_SENS_DATA_17[7:0]
#define	EXT_SENS_DATA_18 	0x5B 	//	R 	EXT_SENS_DATA_18[7:0]
#define	EXT_SENS_DATA_19 	0x5C 	//	R 	EXT_SENS_DATA_19[7:0]
#define	EXT_SENS_DATA_20 	0x5D 	//	R 	EXT_SENS_DATA_20[7:0]
#define	EXT_SENS_DATA_21 	0x5E 	//	R 	EXT_SENS_DATA_21[7:0]
#define	EXT_SENS_DATA_22 	0x5F 	//	R 	EXT_SENS_DATA_22[7:0]
#define	EXT_SENS_DATA_23 	0x60	//	R 	EXT_SENS_DATA_23[7:0]
#define	I2C_SLV0_DO 		0x63	//	R/W 	I2C_SLV0_DO[7:0]
#define	I2C_SLV1_DO 		0x64	//	R/W 	I2C_SLV1_DO[7:0]
#define	I2C_SLV2_DO 		0x65	//	R/W 	I2C_SLV2_DO[7:0]
#define	I2C_SLV3_DO 		0x66	//	R/W
#define	I2C_MST_DELAY_CTRL 	0x67	//	R/W
#define	SIGNAL_PATH_RESET 	0x68	//	R/W
#define	MOT_DETECT_CTRL 	0x69	//
#define	USER_CTRL 			0x6A 	//	R/W
#define	PWR_MGMT_1 			0x6B 	//	R/W
#define	PWR_MGMT_2 			0x6C 	//	R/W
#define	FIFO_COUNTH 		0x72	//	R/W
#define	FIFO_COUNTL 		0x73	//	R/W
#define	FIFO_R_W 			0x74	//	R/W
#define	WHO_AM_I 			0x75	//	R
#define	XA_OFFSET_H 		0x77	//	R/W
#define	XA_OFFSET_L 		0x78	//	R/W
#define	YA_OFFSET_H 		0x7A 	//	R/W
#define	YA_OFFSET_L 		0x7B 	//	R/W
#define	ZA_OFFSET_H 		0x7D 	//	R/W
#define	ZA_OFFSET_L 		0x7E 	//	R/W

//MPU6500 configuration values
#define SLEEP 				0x40
#define H_RESET 			0x80
#define CLKSEL 				0x07
#define CLK_SEL_PLLGYROX 	0x01
#define CLK_SEL_PLLGYROZ 	0x03
#define EXT_SYNC_GYROX 		0x02
#define FS_250DPS           0x00
#define FS_500DPS           0x08
#define FS_1000DPS          0x10
#define FS_2000DPS          0x18
#define FS_2G               0x00
#define FS_4G               0x08
#define FS_8G               0x10
#define FS_16G              0x18
#define FS_MASK             0x18
#define DLPF_CFG_256HZ_NOLPF2  0x00
#define DLPF_CFG_188HZ         0x01
#define DLPF_CFG_98HZ          0x02
#define DLPF_CFG_42HZ          0x03
#define DLPF_CFG_20HZ          0x04
#define DLPF_CFG_10HZ          0x05
#define DLPF_CFG_5HZ           0x06
#define DLPF_CFG_2100HZ_NOLPF  0x07
#define DLPF_CFG_MASK          0x07
#define INT_ANYRD_2CLEAR        0x10
#define RAW_RDY_EN              0x01
#define I2C_IF_DIS              0x10

#define MAX_WRITE_LEN 4
#define MAX_READ_LEN 4

/* Delay */
#ifdef TI_DRIVERS_I2C_INCLUDED
#define delay_ms(i) Task_sleep( ((i) * 1000) / Clock_tickPeriod )
#define MS_2_TICKS(ms) ( ((ms) * 1000) / Clock_tickPeriod )
#else
#define delay_ms(i) ( CPUdelay(8000*(i)) )
#endif


uint8_t MPU6500Init();
uint8_t GetAccelerometerData(int16_t * data);

#endif /* MPU6500_H_ */
