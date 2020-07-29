//status 27 july 2020
//gps issue appears resolved? but system crashes after only 1-2 pps... pps pulses are now shorter
//10ms as opposed to 100ms. This might be an issue.
//check activities within the pps interrupt next time.

//status 20th july 2020
//code is fully operational - sensors and cosmic rays are output, together with GPS data
//picked up without issue by the python daemons on the Pi
//things which still don't work 100% are:
//DMA transfers - switched back to conventional for now, DMA usart was causing crashes/hanging
//DMA interrupts are not handled - might be a cause of this, but handling/interrupts isn't required.
//Strig A and Strig B signals for calibration don't report in, consider changing the types to volatile
//code is rather dirty and needs a good clean up!
//I have a feeling that some events are missed - i.e. every so often data for 1 second doesn't make it in to the output
//but it's hard to test this without statistical analysis on a decent chunk of data
//rates reported are within expectations.
//Calibration values I've been using are DACs at 700-800 (700 > more events)
//HV at 180 (probably works from 200, but we have some margin here).
//after calibration is complete, restarting the detector in software causes a crash
//this doesn't happen when we quit/use the shell scripts
//shell scripts also need tidying: First flash = Erase and flash, then calibration, then any subsequent flashes (flash/upgrade) should be flashes without erase.
//consider modification to print calibration values during start up
//old statuses can be mostly ignored now.

//status 20th july - updated
//doesn't seem to work on units that weren't the sample - crashing.
//found and fixed a bug in the eeprom loading routine, regarding readback values for the key (was wrong variable)
//somehow missing the timer info from the calibration routine
//events/second value counts up (events or seconds?) when running calibration on units that aren't the golden unit.
//more testing required.

//status 12th july.
//eeprom isn't working!? need to fix it. can't read/write reliably.. strange behaviour.
//trying different libraries and changes to .ld file, no success as yet


//july 27th - still random issues with GPS. now unit is working, but crashes on first pps. ?

/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

//status as of 12/07/20 18:09.
//things that work:
//eeprom read/write; there was an issue with the offset, now resolved.
//data input/output seems ok.
//timers sorted, event interrupt not tested (using open dev board)
//next to do: rebuild calibration routines.
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "bmp280/bmp280.h"
#include "lsm9ds1/lsm9ds1_reg.h"
#include "math.h"
#include "stdlib.h"
#include "eeprom.h"

/* USER CODE END Includes */
//__attribute__((__section__(".user_data"))) const uint16_t userConfig[64];

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

#define RMCGGA    "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n" // RCM & GGA
#define ZDA    "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0*29\r\n" // ZDA
#define GGAZDA    "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0*28\r\n" // GGA & ZDA
#define GGA    "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n" // GGA
#define GPSBAUDF "$PMTK251,115200*1F\r\n" //set as a one time thing; then we need to reboot the uart.
#define GPSBAUDM "$PMTK251,19200*25\r\n" //set as a one time thing; then we need to reboot the uart.
#define GPSBAUDS "$PMTK251,9600*17\r\n" //set as a one time thing; then we need to reboot the uart.
#define GPSSETPPS "$PMTK285,1,100*3D\r\n" //set the PPS to work
#define GPSRESET "$PMTK104*37\r\n" //full reset of the GPS unit, all settings 0
#define GPSOFFSET "$PMTK255,1*2D\r\n" //make the nema come after the pps.


#define FMWVERS   "$PMTK605*31\r\n"             // PMTK_Q_RELEASE gets the firmware version
// Sets the update intervall
#define NORMAL    "$PMTK220,1000*1F\r\n"          // PMTK_SET_NMEA_UPDATE_1HZ
// disables updates for the antenna status (only Adafruit ultimate GPS?)
#define NOANTENNA "$PGCMD,33,0*6D\r\n"          // PGCMD_NOAN -> not required for L76

#define dac1_default 750
#define dac2_default 750
#define hv1_default 180
#define hv2_default 180

#define little_g 9.80665



//some eeprom stuff - probably not used? check and delete
#define DATA_EEPROM_BASE_ADDR ((uint32_t)0x08060000) /* Data EEPROM base address */
#define DATA_EEPROM_END_ADDR  ((uint32_t)0x08060080) /* Data EEPROM end address */
uint16_t eeprom_addr_offset = 0x0040;
//eeprom stuff that definitely is used.
uint16_t VirtAddVarTab[NB_OF_VAR] = {0x5000, 0x6000, 0x7000, 0x8000, 0x9000};
uint16_t VarDataTab[NB_OF_VAR] = {0, 0, 0, 0, 0};
uint16_t VarValue,VarDataTmp = 0;
uint32_t eeprom_value=0; //variable for reading/writing from/to eeprom


//we define 32 addresses, each one is 32 bits long
//define for the lsm9ds1 - sensor bus for i2c, specific.
#define SENSOR_BUS hi2c1


ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi2;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

//define bmp280
BMP280_HandleTypedef bmp280;

//print out variables for IMU
float accelx=0;
float accely=0;
float accelz=0;

float magx=0;
float magy=0;
float magz=0;

float pressure, temperature, humidity;

uint16_t size;
uint8_t Data[256];


//calibration variables - used when checking performance during cal mode.
uint16_t cal_events=0; //number of events per second in calibration mode
uint16_t cal_cumulative=0; //cumulative count of events in calibration mode
uint16_t a_events=0; //events on channel a in calibration mode
uint16_t b_events=0; //events on channel b in calibration mode
float rolling_average=0; //moving average of events
uint16_t cal_timer=0; //timer for calibration mode.

//text output; one buffer used at present
//will switch to using both via DMA when DMA is working.
uint8_t TextOutBuf[1024];
uint8_t Bbuffer[1024];
volatile uint8_t toggle = 0; //buffer toggle, not yet used.

//dma handler for usart1
DMA_HandleTypeDef hdma_usart1_tx;
uint32_t oldtimestamp =0;
int16_t convtemp=0;
float float_temp=0;

//timer values for the first interrupt (ch1) - also not sure if used.
uint32_t IC_Value1 = 0;
uint32_t IC_Value2 = 0;
uint32_t Difference = 0;
uint32_t Frequency = 0;
uint8_t Is_First_Captured = 0;  // 0- not captured, 1- captured

uint32_t Evt_stack =0; //number of cosmic ray events this second
uint32_t Evt_timestamps[30]; //space for 30 events per second, no overflow as yet!
uint32_t Evt_total = 0; //total events since start of operation
uint32_t gps_timestamp =0; //value of TIM2 when GPS PPS arrives.

uint8_t data_ready=0; //flag for data ready to send to UART1 via DMA.
uint8_t cal_mode=0; //flag for calibration mode
uint8_t imu_failed=0; //flag for failure of imu/mmu chip. One board had a failed IMU in testing
uint8_t pps_started=0;

//eeprom usage map
/*
 * 0 = is eeprom used? if /=0 then ignore. key
 * 1 = DAC channel 1
 * 2 = DAC channel 2
 * 3 = HV channel 1
 * 4 = HV channel 2
 */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
uint16_t eeprom_read(uint16_t eeprom_address);
static void EXTILine11_Config(void); //the interrupt thingy.
static void EXTILine12_Config(void); //the interrupt thingy.

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void debugPrint(UART_HandleTypeDef *huart, char _out[])
{
	HAL_UART_Transmit(huart, (uint8_t *) _out, strlen(_out), 10);
} 



void debugPrintln(UART_HandleTypeDef *huart, char _out[])
{
	HAL_UART_Transmit(huart, (uint8_t *) _out, strlen(_out), 10);
	char newline[2] = "\r\n";
	HAL_UART_Transmit(huart, (uint8_t *) newline, 2, 10);
} 

//types for accelerometer
typedef union{
	int16_t i16bit[3];
	uint8_t u8bit[6];
} axis3bit16_t;

typedef struct {
	void*   hbus;
	uint8_t i2c_address;
	uint8_t cs_port;
	uint8_t cs_pin;
} sensbus_t;
//manually set the I2C addresses here
static sensbus_t imu_bus = {&SENSOR_BUS,
		0xD5,
		0,
		0};
static sensbus_t mag_bus = {&SENSOR_BUS,
		0x39,
		0,
		0};

static axis3bit16_t data_raw_acceleration;
static axis3bit16_t data_raw_angular_rate;
static axis3bit16_t data_raw_magnetic_field;
static float acceleration_mg[3];

//static float angular_rate_mdps[3];
static float magnetic_field_mgauss[3];
static lsm9ds1_id_t whoamI;
static lsm9ds1_status_t reg;
static uint8_t rst;
static uint8_t tx_buffer[1000];


//instances for the accel and mag
//init the device here;
stmdev_ctx_t dev_ctx_imu;
stmdev_ctx_t dev_ctx_mag;
uint8_t imu_temp[2];

//usart part printf- doesn't seem to work anymore.
//did work when it was first added?
/* We need to implement own __FILE struct */
/* FILE struct is used from __FILE */
struct __FILE {
	int dummy;
};
//declare a file
FILE __stdout;

int fputc(int ch, FILE *f){
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
	return ch;
}

int ferror(FILE *f){
	/* Your implementation of ferror(). */
	return 0;
}

void GPS_repeater(void)
{
	//modify the gps routine so that it runs continuously to allow config changes in realtime.

	//while(1){
		uint8_t buffer[1];
		uint8_t byte;
		//memset(&buffer[0], 0, sizeof(buffer));
		HAL_UART_Receive(&huart2, &buffer, 1, 400); //delay is arbitrary here - less than 1s otherwise interrupt
		HAL_UART_Transmit(&huart1, &buffer, 1, HAL_MAX_DELAY); //delay here has no function, it comes out as soon as it goes in.

		//modified for bidirectional 27/07/20
		//use only when debugging
		//memset(&TextOutBuf[0], 0, sizeof(TextOutBuf));
		//memset(&buffer[0], 0, sizeof(buffer));

		//HAL_UART_Receive(&huart1, &byte, 1, 100);
		//	HAL_UART_Transmit(&huart2, &byte, 1, 10);
	//}

}

void set_HV(uint8_t chan, uint8_t voltage)
{
	debugPrint(&huart1, "Setting HV"); // print
	debugPrint(&huart1, "\r\n"); // print
	//printf("Channel: %d", chan); // print
	//HAL_UART_Transmit(&huart1, chan, sizeof(chan), HAL_MAX_DELAY);
	//printf(
	//debugPrint(&huart1, chan); // print
	//printf(" Voltage: %d", voltage); // print
	//HAL_UART_Transmit(&huart1, voltage, sizeof(voltage), HAL_MAX_DELAY);

	//debugPrint(&huart1, voltage); // print
	//debugPrint(&huart1, "\r\n"); // print
	if (chan == 1)
	{
		HAL_GPIO_WritePin(hvpsu_cs1_GPIO_Port, hvpsu_cs1_Pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(hvpsu_cs2_GPIO_Port, hvpsu_cs2_Pin, GPIO_PIN_RESET);
	}
	//HAL_Delay(100);
	HAL_SPI_Transmit(&hspi2,&voltage,1,100);
	while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(hvpsu_cs1_GPIO_Port, hvpsu_cs1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(hvpsu_cs2_GPIO_Port, hvpsu_cs2_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
	debugPrint(&huart1, "Setting HV completed.\r\n"); // print
}
void set_DAC(uint8_t chan, uint16_t thresh)
{
	debugPrint(&huart1, "Setting DAC"); // print
	debugPrint(&huart1, "\r\n"); // print
	//printf("Channel: %d", chan); // print
	//HAL_UART_Transmit(&huart1, chan, sizeof(chan), HAL_MAX_DELAY);
	//printf(
	//debugPrint(&huart1, chan); // print
	//printf(" Threshold: %d", thresh); // print
	//HAL_UART_Transmit(&huart1, voltage, sizeof(voltage), HAL_MAX_DELAY);

	//debugPrint(&huart1, voltage); // print
	debugPrint(&huart1, "\r\n"); // print
	unsigned char buffer[3];

	buffer[1]=(thresh >> 8);
	buffer[2]=(thresh & 0xFF);
	if (chan == 1)
	{
		buffer[0]=0x00;
		HAL_I2C_Master_Transmit(&hi2c1,0x60<<1,buffer,3,100);


		//HAL_I2C_Mem_Write(&hi2c1, DAC_addr, DAC_ch1, 2, (uint8_t*)(thresh), 2, HAL_MAX_DELAY);
		//HAL_I2C_Master_Transmit(&hi2c1, DAC_addr, (uint8_t)(DAC_ch1), 1, HAL_MAX_DELAY);
	}
	else
	{
		buffer[0]=0x08;
		HAL_I2C_Master_Transmit(&hi2c1,0x60<<1,buffer,3,100);

	}
	debugPrint(&huart1, "Setting DAC completed.\r\n"); // print

}

//global variables for eeprom ops
uint16_t key = 0;
uint16_t DAC_channel1=800;
uint16_t DAC_channel2=800;
uint16_t HV_channel1=190;
uint16_t HV_channel2=190;



void gps_init()
{
	//init GPS to send the right strings only.
	//we do this a bunch of times to be sure it works.
	printf("Start GPS init.\r\n");
	HAL_Delay(5000);
	HAL_UART_Transmit(&huart2, GPSRESET, sizeof(GPSRESET), HAL_MAX_DELAY);
	HAL_Delay(5000);
	HAL_UART_Transmit(&huart2, GPSSETPPS, sizeof(GPSSETPPS), HAL_MAX_DELAY);
	//HAL_Delay(100);
	HAL_Delay(100);
	HAL_UART_Transmit(&huart2, GGAZDA, sizeof(GGAZDA), HAL_MAX_DELAY);
	//HAL_Delay(100);
	HAL_Delay(100);
	//HAL_Delay(1500); //1.5s delay for boot
	HAL_UART_Transmit(&huart2, GPSSETPPS, sizeof(GPSSETPPS), HAL_MAX_DELAY);
	//HAL_Delay(100);
	HAL_Delay(100);
	HAL_UART_Transmit(&huart2, GGAZDA, sizeof(GGAZDA), HAL_MAX_DELAY);
	HAL_Delay(100);
	//HAL_Delay(100);
	HAL_UART_Transmit(&huart2, GGAZDA, sizeof(GGAZDA), HAL_MAX_DELAY);
	HAL_Delay(100);
	HAL_UART_Transmit(&huart2, GPSSETPPS, sizeof(GPSSETPPS), HAL_MAX_DELAY);
	HAL_Delay(100); //1.5s delay for boot
	HAL_UART_Transmit(&huart2, GPSSETPPS, sizeof(GPSSETPPS), HAL_MAX_DELAY);
	HAL_Delay(100);
	HAL_UART_Transmit(&huart2, GPSOFFSET, sizeof(GPSSETPPS), HAL_MAX_DELAY);

	debugPrint(&huart1, "Completed GPS init.\r\n");

}



void bme_readout()
{
	float altitude=0;
	//HAL_Delay(100);
	while (!bmp280_read_float(&bmp280, &temperature, &pressure, &humidity)) {
		size = sprintf((char *)TextOutBuf+strlen(TextOutBuf),
				"Temperature/pressure reading failed\n");
		//HAL_UART_Transmit(&huart1, Data, size, 1000);
		//HAL_Delay(2000);
	}
	//convert pascals to mbar
	pressure = pressure/100;
	altitude = 44330.0f*( 1.0f - pow((pressure/1013.25f), (1.0f/5.255f)))+18;     // Calculate altitude in meters

	size = sprintf((char *)TextOutBuf+strlen(TextOutBuf),"Altitude: %3.6f;\r\nTemperatureCBaro: %3.6f;\r\nPressure: %4.6f;\r\nHumidity: %2.6f;\r\n", altitude, temperature, pressure, humidity);
	//HAL_UART_Transmit(&huart1, Data, size, 1000);

	//size = sprintf((char *)Data,"Pressure: %.2f Pa, Temperature: %.2f C",
	//		pressure, temperature);
	//HAL_UART_Transmit(&huart1, Data, size, 1000);
	//if (bme280p) {
	//		size = sprintf((char *)Data,", Humidity: %.2f\n", humidity);
	//		HAL_UART_Transmit(&huart1, Data, size, 1000);
	//	}

	//else {
	//		size = sprintf((char *)Data, "\n");
	//		HAL_UART_Transmit(&huart1, Data, size, 1000);
	//	}
}
static void platform_init(void)
{
}


static void tx_com(uint8_t *tx_buffer, uint16_t len)
{
	HAL_UART_Transmit(&huart1, tx_buffer, len, 1000);
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
		uint16_t len)
{
	sensbus_t *sensbus = (sensbus_t*)handle;

	if (sensbus->hbus == &hi2c1) {
		HAL_I2C_Mem_Read(sensbus->hbus, sensbus->i2c_address, reg,
				I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
	}
	return 0;
}

static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
		uint16_t len)
{
	sensbus_t *sensbus = (sensbus_t*)handle;

	if (sensbus->hbus == &hi2c1) {
		HAL_I2C_Mem_Write(sensbus->hbus, sensbus->i2c_address, reg,
				I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
	}
	return 0;
}


void lsm9ds1_read_data_polling(void)
{


	debugPrint(&huart1, "Reg writing IMU\r\n");
	/* Initialize inertial sensors (IMU) driver interface */
	dev_ctx_imu.write_reg = platform_write;
	dev_ctx_imu.read_reg = platform_read;
	dev_ctx_imu.handle = (void*)&imu_bus;

	debugPrint(&huart1, "Reg writing mmu\r\n");
	/* Initialize magnetic sensors driver interface */
	dev_ctx_mag.write_reg = platform_write;
	dev_ctx_mag.read_reg = platform_read;
	dev_ctx_mag.handle = (void*)&mag_bus;


	debugPrint(&huart1, "Platform init...\r\n");
	/* Initialize platform specific hardware */
	platform_init();


	debugPrint(&huart1, "Boot delay...\r\n");
	/* Wait sensor boot time */
	HAL_Delay(100);

	/* Check device ID */

	debugPrint(&huart1, "ID check...\r\n");
	lsm9ds1_dev_id_get(&dev_ctx_mag, &dev_ctx_imu, &whoamI);
	debugPrint(&huart1, "ID return...\r\n");
	size = sprintf((char *)Data, "Whoamivals imu: %d, mag: %d \r\n", whoamI.imu, whoamI.mag);
	HAL_UART_Transmit(&huart1, Data, size, 1000);


	if (whoamI.imu != LSM9DS1_IMU_ID || whoamI.mag != LSM9DS1_MAG_ID){
		//while(1){
		/* manage here device not found */
		debugPrint(&huart1, "IMU error. IMU/MAG offline\r\n");
		imu_failed = 1;
		//			printf("address error");
		//}
	}

	/* Restore default configuration */

	debugPrint(&huart1, "Config restore...\r\n");
	lsm9ds1_dev_reset_set(&dev_ctx_mag, &dev_ctx_imu, PROPERTY_ENABLE);
	do {
		lsm9ds1_dev_reset_get(&dev_ctx_mag, &dev_ctx_imu, &rst);
	} while (rst);

	/* Enable Block Data Update */
	lsm9ds1_block_data_update_set(&dev_ctx_mag, &dev_ctx_imu, PROPERTY_ENABLE);


	debugPrint(&huart1, "Set scale...\r\n");
	/* Set full scale */
	lsm9ds1_xl_full_scale_set(&dev_ctx_imu, LSM9DS1_2g);
	lsm9ds1_gy_full_scale_set(&dev_ctx_imu, LSM9DS1_2000dps);
	lsm9ds1_mag_full_scale_set(&dev_ctx_mag, LSM9DS1_16Ga);


	debugPrint(&huart1, "Set bandwidth...\r\n");
	/* Configure filtering chain - See datasheet for filtering chain details */
	/* Accelerometer filtering chain */
	lsm9ds1_xl_filter_aalias_bandwidth_set(&dev_ctx_imu, LSM9DS1_AUTO);
	lsm9ds1_xl_filter_lp_bandwidth_set(&dev_ctx_imu, LSM9DS1_LP_ODR_DIV_50);
	lsm9ds1_xl_filter_out_path_set(&dev_ctx_imu, LSM9DS1_LP_OUT);
	/* Gyroscope filtering chain */
	lsm9ds1_gy_filter_lp_bandwidth_set(&dev_ctx_imu, LSM9DS1_LP_ULTRA_LIGHT);
	lsm9ds1_gy_filter_hp_bandwidth_set(&dev_ctx_imu, LSM9DS1_HP_MEDIUM);
	lsm9ds1_gy_filter_out_path_set(&dev_ctx_imu, LSM9DS1_LPF1_HPF_LPF2_OUT);


	debugPrint(&huart1, "Set outputmode...\r\n");
	/* Set Output Data Rate / Power mode */
	lsm9ds1_imu_data_rate_set(&dev_ctx_imu, LSM9DS1_IMU_59Hz5);
	lsm9ds1_mag_data_rate_set(&dev_ctx_mag, LSM9DS1_MAG_UHP_10Hz);

	/* Read samples in polling mode (no int) */
	//while(1)
	//{
	/* Read device status register */

	debugPrint(&huart1, "Completed.\r\n");
}


void read_imu()
{
	if (imu_failed == 0) {
		lsm9ds1_dev_status_get(&dev_ctx_mag, &dev_ctx_imu, &reg);

		if ( reg.status_imu.xlda && reg.status_imu.gda )
		{
			/* Read imu data */
			memset(data_raw_acceleration.u8bit, 0x00, 3 * sizeof(int16_t));
			memset(data_raw_angular_rate.u8bit, 0x00, 3 * sizeof(int16_t));

			lsm9ds1_acceleration_raw_get(&dev_ctx_imu, data_raw_acceleration.u8bit);
			lsm9ds1_angular_rate_raw_get(&dev_ctx_imu, data_raw_angular_rate.u8bit);

			acceleration_mg[0] = lsm9ds1_from_fs4g_to_mg(data_raw_acceleration.i16bit[0]);
			acceleration_mg[1] = lsm9ds1_from_fs4g_to_mg(data_raw_acceleration.i16bit[1]);
			acceleration_mg[2] = lsm9ds1_from_fs4g_to_mg(data_raw_acceleration.i16bit[2]);

			//angular_rate_mdps[0] = lsm9ds1_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[0]);
			//angular_rate_mdps[1] = lsm9ds1_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[1]);
			//angular_rate_mdps[2] = lsm9ds1_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[2]);

			accelx = ((acceleration_mg[0] /1000)*little_g/2);
			accely = ((acceleration_mg[1] /1000)*little_g/2);
			accelz = ((acceleration_mg[2] /1000)*little_g/2);

			sprintf((char*)TextOutBuf+strlen(TextOutBuf), "AccelX: %2.6f;\r\nAccelY: %2.6f;\r\nAccelZ: %2.6f;\r\n",
					accelx, accely, accelz);
			//angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
			//tx_com(tx_buffer, strlen((char const*)tx_buffer));

			//sprintf(IMUtext, "MagX: %2.6f;\r\nMagY: %2.6f;\r\nMagZ: %2.6f;\r\n", magx, magy, magz);
			//WriteStringToOutputBuff(IMUtext);
			//sprintf(IMUtext, "AccelX: %2.6f;\r\nAccelY: %2.6f;\r\nAccelZ: %2.6f;\r\n",gravx, gravy, gravz);
			//WriteStringToOutputBuff(IMUtext);

		}

		if ( reg.status_mag.zyxda )
		{
			/* Read magnetometer data */
			memset(data_raw_magnetic_field.u8bit, 0x00, 3 * sizeof(int16_t));

			lsm9ds1_magnetic_raw_get(&dev_ctx_mag, data_raw_magnetic_field.u8bit);

			magnetic_field_mgauss[0] = lsm9ds1_from_fs16gauss_to_mG(data_raw_magnetic_field.i16bit[0]);
			magnetic_field_mgauss[1] = lsm9ds1_from_fs16gauss_to_mG(data_raw_magnetic_field.i16bit[1]);
			magnetic_field_mgauss[2] = lsm9ds1_from_fs16gauss_to_mG(data_raw_magnetic_field.i16bit[2]);

			sprintf((char*)TextOutBuf+strlen(TextOutBuf), "MagX: %2.6f;\r\nMagY: %2.6f;\r\nMagZ: %2.6f;\r\n",
					(magnetic_field_mgauss[0]/1000), (magnetic_field_mgauss[1]/1000), (magnetic_field_mgauss[2]/1000));
			//tx_com(tx_buffer, strlen((char const*)tx_buffer));


			//there is a problem with this temperature reading. 160720 - starts reading out 4000? was working before.
			//now we print out the temp
			lsm9ds1_temperature_raw_get(&dev_ctx_imu, imu_temp);
			convtemp = ((imu_temp[1] << 8) + imu_temp[0]);
			float_temp = (convtemp / 16) + 27.5f;
			//sprintf((char*)TextOutBuf+strlen(TextOutBuf), "acceltemp:%3.6f;\r\n",convtemp);

			sprintf((char*)TextOutBuf+strlen(TextOutBuf), "TemperatureCHumid:%3.6f;\r\n",float_temp);
			//tx_com(tx_buffer, strlen((char const*)tx_buffer));

			//temp is working again now? not sure why it broke...

		}
		//}
	}
}



void avg_temp_print(void)
{
	float avg_temp = (temperature+float_temp)/2;
	sprintf((char*)TextOutBuf+strlen(TextOutBuf), "TemperatureC: %3.6f;\r\n",avg_temp);
	//tx_com(tx_buffer, strlen((char const*)tx_buffer));

}

void print_buffer(void)
{
	//TextOutBufSize = strlen(TextOutBuf); //check characters in buffer
	if (strlen(TextOutBuf) >0 )
	{
		//print one if there is a character to print
		//problem here, it only transmits the first letter, over and over
		//see if there's a DMA route?
		//DMA instruction to buffer -> UART would be ideal.
		HAL_UART_Transmit(&huart1, TextOutBuf, 1, 1000);
		//HAL_UART_Transmit(&huart1, TextOutBuf, strlen(TextOutBuf), 1000);
	}
	//if (strlen(TextOutBuf)==0 )
	//{
	//memset(&TextOutBuf[0], 0, sizeof(TextOutBuf));
	//}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	HAL_NVIC_DisableIRQ(TIM2_IRQn); //disable interrupts on call, regardless of type. renable at the end.
	//memset(TextOutBuf,0,strlen(TextOutBuf));

	//debugPrint(&huart1, "TIM2\r\n");

	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if interrput source is channel 1
	{
		pps_started = 1;
		//here we are in the PPS case. Clear the buffer
		memset(TextOutBuf,0,strlen(TextOutBuf));


		//debugPrint(&huart1, "GPSPPS\r\n");
		HAL_GPIO_TogglePin(pwr_led_GPIO_Port,pwr_led_Pin);
		sprintf((char*)TextOutBuf+strlen(TextOutBuf), "PPS: GPS lock:1;\r\n");

		if (cal_mode==1)
		{
			//in calibration mode, put the number of events in a print buffer and print it directly.
			cal_cumulative = cal_cumulative + cal_events; //add in the events this second to the cumulative total
			if (cal_timer <120)
			{
				cal_timer++; //increment the calibration timer (starts at 0 - number of seconds that cal has been running; assume instant start up! could be dangerous so we'll reset every 2 minutes to be sure)
			}
			else
			{
				//reset condition every 2 minutes, and we don't set it to 0 otherwise we'd get a div0
				cal_timer = 1;
				cal_cumulative = cal_events;
			}
			rolling_average = (float)cal_cumulative/(float)cal_timer;

			memset(&TextOutBuf[0], 0, sizeof(TextOutBuf));

			sprintf((char*)TextOutBuf, "Cal - Events: %d, Ch_A: %d, Ch_B: %d, Timer: %d, Rolling average: %f\r\n", cal_events, a_events, b_events, cal_timer, rolling_average);


			//bme_readout();
			//read_imu();
			//avg_temp_print();
			//now reset the counters
			cal_events = 0;
			a_events = 0;
			b_events = 0;
			//HAL_UART_Transmit_DMA(&huart1, TextOutBuf, 1024); //this is the best way to send the data,
			//but probably can't be dynamically sized. must find a way to send it all.
			data_ready=1;

		}
		else
		{
			//normal operation mode
			//debugPrint(&huart1, "PPS\r\n");
			Evt_total = Evt_total+Evt_stack; //increment total events
			//oldtimestamp = gps_timestamp; //backup the old value
			gps_timestamp = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);  // capture the first value

			//HAL_GPIO_WritePin(evt_led_GPIO_Port, evt_led_Pin,  GPIO_PIN_SET);

			//here goes the code to do the readouts; write second
			//readout events
			//readout secondary data
			//now reset the counter to 0;
			//print sensors was here, moving to main loop
			bme_readout();
			read_imu();
			avg_temp_print();

			if (Evt_stack > 0) {
				for (uint8_t prt_ctr=0; prt_ctr<(Evt_stack+1); prt_ctr++)
				{
					sprintf((char*)TextOutBuf+strlen(TextOutBuf), "Event: sub second micros:%d/%d; Event Count: %d\r\n", Evt_timestamps[prt_ctr], gps_timestamp, prt_ctr+1);

					//sprintf((char*)TextOutBuf, "GPS_PPS\r\n");

				}
			}

			//HAL_GPIO_TogglePin(pwr_led_GPIO_Port,pwr_led_Pin);
			//sprintf((char*)TextOutBuf, "GPS_PPS\r\n");
			//HAL_UART_Transmit(&huart1, TextOutBuf, sizeof(TextOutBuf), 1000);
			data_ready=1;
			//reset ctr
			TIM2->CNT = 0; //reset the ctr
			TIM2->CR1 |= 0x01;

			//after we print, set the event stack back to 0;
			Evt_stack=0;

			//TIM2->CCMR1 |= TIM_CCMR1_CC1S_0;
			//TIM2->CCER |= TIM_CCER_CC1E;
			//TIM2->CR1 |= TIM_CR1_CEN;
			//TIM2->SR = ~TIM_SR_CC1IF;
		}

	}
	else
		//here we are in the event case. Which is all other times we execute this routine if channel 1 wasn't used.

		//if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)  // if interrput source is channel 2, cosmic event
	{
		HAL_GPIO_WritePin(evt_led_GPIO_Port,evt_led_Pin, GPIO_PIN_SET);

		//debugPrint(&huart1, "evt\r\n");


		//when we have an event, we read the timer into the nth slot of the stack.
		if (pps_started)
		{
		Evt_timestamps[Evt_stack] = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2);  // capture the first value
		Evt_stack++;
		}
		HAL_GPIO_WritePin(evt_led_GPIO_Port,evt_led_Pin, GPIO_PIN_SET);
		//if (Evt_stack>30) sprintf((char*)TextOutBuf, "Event overflow");

		//HAL_GPIO_WritePin(evt_led_GPIO_Port, evt_led_Pin,  GPIO_PIN_SET); //set event pin, we'll reset it in the main loop after a v. short delay.
		if (cal_mode==1)
		{
			cal_events++;
		}

	}
	//data_ready=1;
	HAL_NVIC_EnableIRQ(TIM2_IRQn); //disable interrupts on call, regardless of type. renable at the end.
	//HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}


void set_cal_interrupt()
//this doesn't work? Don't know why
//it's not essential but would facilitate calibration
{
	debugPrint(&huart1, "Setting interrupts for calibration\r\n");
	EXTILine12_Config();
	EXTILine11_Config();
	debugPrint(&huart1, "Strig_a & Strig_b interrupts set\r\n");
}


void release_cal_interrupt()
//ditto also not working, but only because the interrupts don't bind in the first place.
{
	debugPrint(&huart1, "Releasing interrupts for calibration\r\n");
	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
	debugPrint(&huart1, "Interrupts released.\r\n");
}



void cal_routine()
{
	//this code enables manual calibration over the serial port, requires the flag pin to be set high when booting the STM32.
	//there are scripts in the pi image to do this automatically

	char calstat[2];
	char valstat[4];
	char hv1stat[4];
	char hv2stat[4];
	char dac1stat[5];
	char dac2stat[5];

	uint8_t selector = 0;
	uint16_t settingval = 0;
	uint32_t eepromcomplete = 1;
	//start a while loop here somehow

	if (data_ready)
	{
		sprintf((char*)TextOutBuf, "Cal - CMF events: %d, Events/sec: %d, Ch_A: %d, Ch_B: %d, Timer: %d, average: %f\r\n", cal_cumulative, cal_events, a_events, b_events, cal_timer, rolling_average);
		//bme_readout();
		//read_imu();
		//avg_temp_print();

		HAL_UART_Transmit(&huart1, TextOutBuf, sizeof(TextOutBuf), 1000);
		data_ready=0;
	}

	memset(&calstat[0], 0, sizeof(calstat));
	memset(&valstat[0], 0, sizeof(valstat));
	debugPrint(&huart1, "Calibration Mode for Cosmic Pi V.1.7\r\n");
	debugPrint(&huart1, "Choose option:\r\n");
	debugPrint(&huart1, "1: Set DAC channel 1\r\n");
	debugPrint(&huart1, "2: Set DAC channel 2\r\n");
	debugPrint(&huart1, "3: Set HV channel 1\r\n");
	debugPrint(&huart1, "4: Set HV channel 2\r\n");
	debugPrint(&huart1, "5: Set DAC channels 1 & 2 at the same time\r\n");
	debugPrint(&huart1, "6: Set HV channels 1 & 2 at the same time\r\n");
	debugPrint(&huart1, "7: enable calibration interrupts\r\n");
	debugPrint(&huart1, "8: write to eeprom\r\n");
	debugPrint(&huart1, "9: quit debug mode\r\n");
	debugPrint(&huart1, "0: repeat menu\r\n");

	HAL_UART_Receive(&huart1, (uint8_t *)calstat, 1, 30000); //if it times out, then we need to repeat it.
	debugPrint(&huart1, "\r\n");
	debugPrint(&huart1, "Read input value: ");
	HAL_UART_Transmit(&huart1, (uint8_t *)calstat, 1, 30000);
	debugPrint(&huart1, "\r\n");
	selector = 0;
	selector = atoi(calstat);

	switch (selector)
	{
	case 0:
		debugPrint(&huart1, "Choose option:\r\n");
		debugPrint(&huart1, "1: Set DAC channel 1\r\n");
		debugPrint(&huart1, "2: Set DAC channel 2\r\n");
		debugPrint(&huart1, "3: Set HV channel 1\r\n");
		debugPrint(&huart1, "4: Set HV channel 2\r\n");
		debugPrint(&huart1, "5: Set DAC channels 1 & 2 at the same time\r\n");
		debugPrint(&huart1, "6: Set HV channels 1 & 2 at the same time\r\n");
		debugPrint(&huart1, "7: enable calibration interrupts\r\n");
		debugPrint(&huart1, "8: write to eeprom\r\n");
		debugPrint(&huart1, "9: quit debug mode\r\n");
		//nb when we press 9 and try starting the detector, it almost always crashes.
		//don't know why. a command line reset fixes it.
		debugPrint(&huart1, "0: repeat menu\r\n");
		break;
	case 1:
		memset(&dac1stat[0], 0, sizeof(dac1stat));
		debugPrint(&huart1, "Set DAC channel 1, enter 4 digits from 0000 to 1024\r\n");
		HAL_UART_Receive(&huart1, (uint8_t *)dac1stat, 4, 30000); //if it times out, then we need to repeat it.
		debugPrint(&huart1, "\r\n");
		debugPrint(&huart1, "Read input value: ");
		HAL_UART_Transmit(&huart1, (uint8_t *)dac1stat, 4, 30000);
		debugPrint(&huart1, "\r\n");
		settingval=0;
		DAC_channel1 = atoi(dac1stat);

		size = sprintf((char *)Data, "DAC Channel 1: %d \r\n", DAC_channel1);
		HAL_UART_Transmit(&huart1, Data, size, 1000);

		set_DAC(1,DAC_channel1);
		debugPrint(&huart1, "DAC channel 1 setting completed \r\n");
		break;
	case 2:
		memset(&dac2stat[0], 0, sizeof(dac2stat));
		debugPrint(&huart1, "Set DAC channel 2, enter 4 digits from 0000 to 1024\r\n");
		HAL_UART_Receive(&huart1, (uint8_t *)dac2stat, 4, 30000); //if it times out, then we need to repeat it.
		debugPrint(&huart1, "\r\n");
		debugPrint(&huart1, "Read input value: ");
		HAL_UART_Transmit(&huart1, (uint8_t *)dac2stat, 4, 30000);
		debugPrint(&huart1, "\r\n");
		DAC_channel2 = atoi(dac2stat);
		size = sprintf((char *)Data, "DAC Channel 2: %d \r\n", DAC_channel2);
		HAL_UART_Transmit(&huart1, Data, size, 1000);

		set_DAC(2,DAC_channel2);
		debugPrint(&huart1, "DAC channel 2 setting completed \r\n");
		break;
	case 3:
		memset(&valstat[0], 0, sizeof(valstat));
		debugPrint(&huart1, "Set HV channel 1, enter 3 digits from 000 to 255\r\n");
		HAL_UART_Receive(&huart1, (uint8_t *)hv1stat, 3, 30000); //if it times out, then we need to repeat it.
		debugPrint(&huart1, "\r\n");
		debugPrint(&huart1, "Read input value: ");
		HAL_UART_Transmit(&huart1, (uint8_t *)hv1stat, 3, 30000);
		debugPrint(&huart1, "\r\n");
		HV_channel1 = atoi(hv1stat);
		set_HV(1,HV_channel1);
		debugPrint(&huart1, "HV channel 1 setting completed \r\n");
		break;
	case 4:
		memset(&valstat[0], 0, sizeof(valstat));
		debugPrint(&huart1, "Set HV channel 2, enter 3 digits from 000 to 255\r\n");
		HAL_UART_Receive(&huart1, (uint8_t *)valstat, 3, 30000); //if it times out, then we need to repeat it.
		debugPrint(&huart1, "\r\n");
		debugPrint(&huart1, "Read input value: ");
		HAL_UART_Transmit(&huart1, (uint8_t *)valstat, 3, 30000);
		debugPrint(&huart1, "\r\n");
		settingval=0;
		settingval = atoi(valstat);
		HV_channel2 = settingval;
		set_HV(2,settingval);
		debugPrint(&huart1, "HV channel 2 setting completed \r\n");
		break;
	case 5:
		memset(&dac2stat[0], 0, sizeof(dac2stat));
		debugPrint(&huart1, "Set DAC channels 1 & 2, enter 4 digits from 0000 to 1024\r\n");
		HAL_UART_Receive(&huart1, (uint8_t *)dac2stat, 4, 30000); //if it times out, then we need to repeat it.
		debugPrint(&huart1, "\r\n");
		debugPrint(&huart1, "Read input value: ");
		HAL_UART_Transmit(&huart1, (uint8_t *)dac2stat, 4, 30000);
		debugPrint(&huart1, "\r\n");
		settingval=0;
		settingval = atoi(dac2stat);
		DAC_channel1 = settingval;
		DAC_channel2 = settingval;
		set_DAC(1,settingval);
		set_DAC(2,settingval);
		debugPrint(&huart1, "DAC channels 1 & 2 setting completed \r\n");
		break;
	case 6:
		memset(&valstat[0], 0, sizeof(valstat));
		debugPrint(&huart1, "Set HV channels 1 & 2, enter 3 digits from 000 to 255\r\n");
		HAL_UART_Receive(&huart1, (uint8_t *)valstat, 3, 30000); //if it times out, then we need to repeat it.
		debugPrint(&huart1, "\r\n");
		debugPrint(&huart1, "Read input value: ");
		HAL_UART_Transmit(&huart1, (uint8_t *)valstat, 3, 30000);
		debugPrint(&huart1, "\r\n");
		settingval=0;
		settingval = atoi(valstat);
		HV_channel1 = settingval;
		HV_channel2 = settingval;
		set_HV(1,settingval);
		set_HV(2,settingval);
		debugPrint(&huart1, "HV channels 1 & 2 setting completed \r\n");
		break;
	case 7:
		//set the interrupts for cal... somehow
		set_cal_interrupt();
		break;
	case 8:
		flash_pgm();
		break;
	case 9:
		//turn off the cal interrupts
		release_cal_interrupt();
		cal_mode=0;
		break;

	default: break;

	}
}


void flash_pgm() //dump all current variables into flash, overwrite whatever is in there.
{
	debugPrint(&huart1, "Writing current config values to local flash...\r\n");
	HAL_FLASH_Unlock();

	if( EE_Init() != EE_OK)
	{
		Error_Handler();
	}

	debugPrint(&huart1, "Flash unlock done\r\n");

	debugPrint(&huart1, "writing key (1) to first location.\r\n");
	/* Sequence 1 */
	if((EE_WriteVariable(VirtAddVarTab[0],  (uint16_t)0x01)) != HAL_OK)
	{
		Error_Handler();
	}
	//debugPrint(&huart1, "complete. \r\n");
	debugPrint(&huart1, "Writing DAC Channel 1 value to second location\r\n");
	if((EE_WriteVariable(VirtAddVarTab[1], DAC_channel1 )) != HAL_OK)
	{
		Error_Handler();
	}


	debugPrint(&huart1, "Writing DAC Channel 2 value to second location\r\n");
	if((EE_WriteVariable(VirtAddVarTab[2],  DAC_channel2)) != HAL_OK)
	{
		Error_Handler();
	}


	debugPrint(&huart1, "Writing HV Channel 1 value to third location\r\n");
	if((EE_WriteVariable(VirtAddVarTab[3],  HV_channel1)) != HAL_OK)
	{
		Error_Handler();
	}


	debugPrint(&huart1, "Writing HV Channel 2 value to fourth location\r\n");
	if((EE_WriteVariable(VirtAddVarTab[4],  HV_channel2)) != HAL_OK)
	{
		Error_Handler();
	}


	debugPrint(&huart1, "Write stage done \r\n");

	/* read the last stored variables data*/
	if(EE_ReadVariable(VirtAddVarTab[0], &VarDataTmp) != HAL_OK)
	{
		Error_Handler();
	}
	size = sprintf((char *)Data, "Readback EEPROM location 1: %d \r\n", VarDataTmp);
	HAL_UART_Transmit(&huart1, Data, size, 1000);


	if(EE_ReadVariable(VirtAddVarTab[1], &VarDataTmp) != HAL_OK)
	{
		Error_Handler();
	}
	size = sprintf((char *)Data, "Readback2: %d \r\n", VarDataTmp);
	HAL_UART_Transmit(&huart1, Data, size, 1000);

	if(EE_ReadVariable(VirtAddVarTab[2], &VarDataTmp) != HAL_OK)
	{
		Error_Handler();
	}
	size = sprintf((char *)Data, "Readback3: %d \r\n", VarDataTmp);
	HAL_UART_Transmit(&huart1, Data, size, 1000);

	if(EE_ReadVariable(VirtAddVarTab[3], &VarDataTmp) != HAL_OK)
	{
		Error_Handler();
	}
	size = sprintf((char *)Data, "Readback4: %d \r\n", VarDataTmp);
	HAL_UART_Transmit(&huart1, Data, size, 1000);

	if(EE_ReadVariable(VirtAddVarTab[4], &VarDataTmp) != HAL_OK)
	{
		Error_Handler();
	}
	size = sprintf((char *)Data, "Readback5: %d \r\n", VarDataTmp);
	HAL_UART_Transmit(&huart1, Data, size, 1000);

	debugPrint(&huart1, "Locking flash \r\n");
	HAL_FLASH_Unlock();
	debugPrint(&huart1, "EEPROM programming successfully completed \r\n");

	HAL_Delay(500);

	//got to here on the 15th July.
	//eeprom is now working (read/write)
	//remember to lock the eeprom after read write routines.
	//eeprom read routine needs to be rewritten,
	//dead functions need to be culled.
}

void flash_load()
{
	//routine to load the contents of the flash memory into ram and set the necessary hardware values
	uint16_t localread=0;


	debugPrint(&huart1, "Loading calibration values from flash \r\n");

	if(EE_ReadVariable(VirtAddVarTab[0], &localread) != HAL_OK)
	{
		debugPrint(&huart1, "Flash read failed \r\n");

		Error_Handler();
	}
	size = sprintf((char *)Data, "Readback EEPROM location 1: %d \r\n", localread);
	HAL_UART_Transmit(&huart1, Data, size, 1000);
	if (localread==1)
	{
		debugPrint(&huart1, "Valid values found, loading \r\n");
		if(EE_ReadVariable(VirtAddVarTab[1], &localread) != HAL_OK)
		{
			Error_Handler();
		}
		size = sprintf((char *)Data, "Readback EEPROM location 2: %d \r\n", localread);
		HAL_UART_Transmit(&huart1, Data, size, 1000);
		DAC_channel1=localread;
		if(EE_ReadVariable(VirtAddVarTab[2], &localread) != HAL_OK)
		{
			Error_Handler();
		}
		size = sprintf((char *)Data, "Readback EEPROM location 3: %d \r\n", localread);
		HAL_UART_Transmit(&huart1, Data, size, 1000);
		DAC_channel2=localread;
		if(EE_ReadVariable(VirtAddVarTab[3], &localread) != HAL_OK)
		{
			Error_Handler();
		}
		size = sprintf((char *)Data, "Readback EEPROM location 4: %d \r\n", localread);
		HAL_UART_Transmit(&huart1, Data, size, 1000);
		HV_channel1=localread;
		if(EE_ReadVariable(VirtAddVarTab[4], &localread) != HAL_OK)
		{
			Error_Handler();
		}
		size = sprintf((char *)Data, "Readback EEPROM location 5: %d \r\n", localread);
		HAL_UART_Transmit(&huart1, Data, size, 1000);
		HV_channel2=localread;

		//set the values after loading.
		set_DAC(1, DAC_channel1);
		set_DAC(2, DAC_channel2);
		set_HV(1, HV_channel1);
		set_HV(2, HV_channel2);

	}
	else
	{
		debugPrint(&huart1, "Flash does not contain valid variables \r\n");
		debugPrint(&huart1, "Loading default values, DAC 1 & 2 - 750, HV 1 & 2 - 180 \r\n");
		set_DAC(1,dac1_default);
		set_DAC(2,dac2_default);
		set_HV(1,hv1_default);
		set_HV(2,hv2_default);

	}

}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

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


	MX_USART1_UART_Init();

	//debugPrint(&huart1, "EEPROM init starting...\r\n");

	//HAL_FLASH_Unlock();
	//
	//setup the 64 addresses in eeprom, we only need 5.
	//init the eeprom.
	//if( EE_Init() != EE_OK)
	//  {
	//    Error_Handler();
	//  }
	//enableEEPROMWriting();

	//debugPrint(&huart1, "EEPROM init complete\r\n");



	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_SPI2_Init();
	MX_I2C1_Init();
	HAL_GPIO_WritePin(pwr_led_GPIO_Port, pwr_led_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(evt_led_GPIO_Port, evt_led_Pin,  GPIO_PIN_RESET);

	MX_DMA_Init();
	MX_ADC1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_USART2_UART_Init();

	HAL_Delay(1500);
	/* USER CODE BEGIN 2 */
	//in the main routine turn on the power led. It will toggle every second thereafter via GPS PPS interrupt (TIM2).
	HAL_GPIO_WritePin(pwr_led_GPIO_Port, pwr_led_Pin, GPIO_PIN_SET);

	//set out of range values to prevent spurious interrupts during start up
	set_DAC(1, 900);
	set_DAC(2, 900);
	set_HV(1, 240);
	set_HV(2, 240);


	debugPrint(&huart1, "Cosmic Pi Version 1.7 startup \r\n"); // print

	debugPrint(&huart1, "Initialise IMU...\r\n");
	lsm9ds1_read_data_polling();
	//if the board has a failed IMU, it'll be reported here.

	//setup the pressure sensor
	bmp280_init_default_params(&bmp280.params);
	bmp280.addr = BMP280_I2C_ADDRESS_0;
	bmp280.i2c = &hi2c1;

	while (!bmp280_init(&bmp280, &bmp280.params)) {
		size = sprintf((char *)Data, "BMP280 initialization failed\r\n");
		HAL_UART_Transmit(&huart1, Data, size, 1000);
		HAL_Delay(2000);
	}
	bool bme280p = bmp280.id == BME280_CHIP_ID;
	size = sprintf((char *)Data, "BMP280: found %s\r\n", bme280p ? "BME280" : "BMP280");
	HAL_UART_Transmit(&huart1, Data, size, 1000);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	//set up the GPS - send it custom commands to perform as required.
	debugPrint(&huart1, "Launch GPS init\r\n");
	gps_init();

	//delete this if it isn't used?
	//uint8_t hvmove=0xFF;


	//start the timer TIM2
	debugPrint(&huart1, "timer init - GPS\r\n");
	HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_1); //gps pps timer routine
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1); //gps pps timer routine
	//TIM_Cmd(TIM2, ENABLE);
	TIM2->CR1 |= 0x01;
	//debugPrint(&huart1, "timer init - EVT\r\n");
	HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_2); //gps pps timer routine
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2); //event timer routine
	TIM2->CR1 |= 0x01;
	//debugPrint(&huart1, "Starting timer 2\r\n");
	//HAL_TIM_Base_Start();
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_Base_Start_IT(&htim2);


	//timers are started
	//debugPrint(&huart1, "Timer started\r\n");

	//might not be used?
	//uint32_t readout =0;

	//check if we are in calibration mode, if so run cal routine, otherwise carry on
	if(HAL_GPIO_ReadPin(flag_GPIO_Port, flag_Pin))
	{
		cal_mode = 1;
		HAL_GPIO_WritePin(pwr_led_GPIO_Port, pwr_led_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(evt_led_GPIO_Port, evt_led_Pin, GPIO_PIN_RESET);

	}
	while (cal_mode==1)
	{
		cal_routine(); //it's within a while loop so it can be exited. don't forget to cancel the cal interrupts if exiting
	}
	//assuming we're not doing calibration, start up as normal.
	HAL_NVIC_DisableIRQ(TIM2_IRQn); //disable interrupts on call, regardless of type. renable at the end.

	//load calibration values from eeprom.
	flash_load();


	//HAL_GPIO_WritePin(evt_led_GPIO_Port, evt_led_Pin,  GPIO_PIN_RESET);
	//char in[8];
	//uint16_t firstint=0;
	//uint16_t secondint=0;

	//now we try to read some input;
	//debugPrint(&huart1, "loop\r\n");
	HAL_GPIO_WritePin(pwr_led_GPIO_Port, pwr_led_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(evt_led_GPIO_Port, evt_led_Pin, GPIO_PIN_RESET);


	HAL_NVIC_EnableIRQ(TIM2_IRQn); //disable interrupts on call, regardless of type. renable at the end.

	//HAL_NVIC_EnableIRQ(TIM2_IRQn); //re-enable interrupt TIM2.


	//infinite loop for detector operation.
	while(1){

		//reset the EVT pin each cycle.
		HAL_GPIO_WritePin(evt_led_GPIO_Port, evt_led_Pin,  GPIO_PIN_RESET);
		//collect any characters from the GPS and print
		GPS_repeater();

		if (data_ready) {
			HAL_NVIC_DisableIRQ(TIM2_IRQn); //disable interrupts on call, regardless of type. renable at the end.
			HAL_UART_Transmit(&huart1, TextOutBuf, strlen(TextOutBuf), 100); //send one char at a time when idle.

			HAL_NVIC_EnableIRQ(TIM2_IRQn); //disable interrupts on call, regardless of type. renable at the end.

			data_ready = 0;
		}


	}

	/*
	while (1)
	{
		if (data_ready == 1) {
			//print out the data here, rather than in the interupt.
			size = sprintf((char *)Data, "gps timestamp: %d \r\n", gps_timestamp);
			HAL_UART_Transmit(&huart1, Data, size, 1000);
			data_ready=0;
		}

	}
	 */
	/* USER CODE END 3 */
}

static void EXTILine12_Config(void)
{
	GPIO_InitTypeDef   GPIO_InitStructure;

	debugPrint(&huart1, "enable strig_a interrupt starting...\r\n");


	/* Enable GPIOC clock */
	__HAL_RCC_GPIOC_CLK_ENABLE();

	/* Configure PC11 pin as input floating */
	GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Pin = GPIO_PIN_12;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* Enable and set EXTI Line0 Interrupt to the lowest priority */
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);


	debugPrint(&huart1, "enable strig_a interrupt complete\r\n");

}

static void EXTILine11_Config(void)
{
	GPIO_InitTypeDef   GPIO_InitStructure;

	debugPrint(&huart1, "enable strig_b interrupt starting...\r\n");


	/* Enable GPIOC clock */
	__HAL_RCC_GPIOC_CLK_ENABLE();

	/* Configure PC11 pin as input floating */
	GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Pin = GPIO_PIN_11;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* Enable and set EXTI Line0 Interrupt to the lowest priority */
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);



	debugPrint(&huart1, "enable strig_b interrupt complete\r\n");

}



/**
 * @brief EXTI line detection callbacks
 * @param GPIO_Pin: Specifies the pins connected EXTI line
 * @retval None
 * GPIO_PIN_12
 *
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_12)
	{
		//debugPrint(&huart1, "a_evt\r\n");
		a_events++;
	}
	if(GPIO_Pin == GPIO_PIN_11)
	{
		//debugPrint(&huart1, "b_evt\r\n");
		b_events++;
	}

}



/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 84;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 3;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */
	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void)
{

	/* USER CODE BEGIN SPI2_Init 0 */

	/* USER CODE END SPI2_Init 0 */

	/* USER CODE BEGIN SPI2_Init 1 */

	/* USER CODE END SPI2_Init 1 */
	/* SPI2 parameter configuration*/
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN SPI2_Init 2 */

	/* USER CODE END SPI2_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_IC_InitTypeDef sConfigIC = {0};

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 42500000;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 0;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

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
	huart1.Init.OverSampling = UART_OVERSAMPLING_8;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 9600;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_8;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/** 
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) 
{

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
	/* DMA2_Stream7_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, pwr_led_Pin|evt_led_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(flag_GPIO_Port, flag_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, hvpsu_cs2_Pin|hvpsu_cs1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : hvpsu_cl1_Pin hvpsu_cl2_Pin strigout_b_Pin strigout_a_Pin */
	GPIO_InitStruct.Pin = hvpsu_cl1_Pin|hvpsu_cl2_Pin|strigout_b_Pin|strigout_a_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : pwr_led_Pin evt_led_Pin */
	GPIO_InitStruct.Pin = pwr_led_Pin|evt_led_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : flag_Pin */
	GPIO_InitStruct.Pin = flag_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	//GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(flag_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : hvpsu_cs2_Pin hvpsu_cs1_Pin */
	GPIO_InitStruct.Pin = hvpsu_cs2_Pin|hvpsu_cs1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	while(1)
	{
		/* Toggle LED2 fast */
		//BSP_LED_Toggle(LED2);
		//HAL_Delay(40);
	}
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
