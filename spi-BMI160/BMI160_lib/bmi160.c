/*
****************************************************************************
* Copyright (C) 2016 Institute for Infocomm Research
*
* bmi160.c
* Date: 2016/12/21
* Revision: 1.0.0 $
*
* Usage: Sensor Driver for BMI160 sensor
*
****************************************************************************

! file BMI160
    @brief Sensor driver for BMI160 */

#include "app_timer.h"
#include "app_error.h"
#include "app_util_platform.h"

#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_drv_spi.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_gpio.h"

#include "sdk_config.h"

#include "bmi160.h"


#define BMI160_SPI_MISO_PIN 		28	// 15
#define BMI160_SPI_SS_PIN 			29
#define BMI160_SPI_MOSI_PIN 		4	// 13
#define BMI160_SPI_SCK_PIN 			3

#define BMI160_SPI_ENABLE_PIN		29


static const nrf_drv_spi_t m_spi_master = NRF_DRV_SPI_INSTANCE(0);

#define TX_RX_BUF_LENGTH         16           /**< SPI transaction buffer length. */
// Data buffers.
static uint8_t m_tx_data[TX_RX_BUF_LENGTH] = {0}; /**< A buffer with data to transfer. */
static uint8_t m_rx_data[TX_RX_BUF_LENGTH] = {0}; /**< A buffer for incoming data. */

#define BMI160_CHIP_ID 			0xD1


static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */

/**
 * @brief SPI user event handler.
 * @param event
 */
static void spi_master_event_handler(nrf_drv_spi_evt_t const * p_event)
{
    spi_xfer_done = true;
    //NRF_LOG_RAW_INFO("SPI xfer completed.\r\n");
}


/**@brief Function for application main entry. Does not return. */
void spi_init(void)
{
	nrf_delay_ms(50);
		
	nrf_drv_spi_config_t const config =
	{
		.sck_pin  = BMI160_SPI_SCK_PIN,
		.mosi_pin = BMI160_SPI_MOSI_PIN,
		.miso_pin = BMI160_SPI_MISO_PIN,
		.ss_pin   = BMI160_SPI_SS_PIN,
		//.ss_pin   = NRF_DRV_SPI_PIN_NOT_USED,   
		.irq_priority = APP_IRQ_PRIORITY_LOW,
		.orc          = 0xCC,
		.frequency    = NRF_DRV_SPI_FREQ_8M,
		.mode         = NRF_DRV_SPI_MODE_0,
		//.mode         = NRF_DRV_SPI_MODE_1,
		.bit_order    = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST,
		//.bit_order    = NRF_DRV_SPI_BIT_ORDER_LSB_FIRST,	
	};
	ret_code_t err_code = nrf_drv_spi_init(&m_spi_master, &config, spi_master_event_handler);
	APP_ERROR_CHECK(err_code);
	NRF_LOG_INFO("spi init OK\r\n");
}


uint8_t reg_read (uint8_t reg)
{
    m_tx_data[0] = reg | (1 << BMI160_SPI_READ_BIT);
		
		// rising edge of CS pin switches BMI160 to SPI mode
		//nrf_gpio_pin_clear(BMI160_SPI_ENABLE_PIN);

	  //nrf_delay_us(10);
    // Start transfer.
    uint32_t err_code = nrf_drv_spi_transfer(&m_spi_master,
        m_tx_data, 1, m_rx_data, 1);

	  //nrf_delay_us(10);
		// set CS pin for next SPI communication
		//nrf_gpio_pin_set(BMI160_SPI_ENABLE_PIN);
		//APP_ERROR_CHECK(err_code);
		
		if(NRF_SUCCESS != err_code) {
			NRF_LOG_RAW_INFO("\t\treg_read ERROR 0x%X. Read Value 0x%X \r\n", 
					err_code, m_rx_data[0]);
			NRF_LOG_FLUSH();
		} /*else {
			NRF_LOG_RAW_INFO("\t\treg_read OK. Read Value 0x%X \r\n", m_rx_data[0]);
			NRF_LOG_FLUSH();
		}*/
    return m_rx_data[0];
}

/** Provides a serial buffer transfer implementation for the BMI160 base class
 *  to use for accessing device registers.  This implementation uses the SPI
 *  bus on the Intel Curie module to communicate with the BMI160.
 */
uint32_t serial_buffer_transfer(unsigned tx_cnt, unsigned rx_cnt)
{
    uint32_t status;

    if (rx_cnt) /* For read transfers, assume 1st byte contains register address */
        m_tx_data[0] |= (1 << BMI160_SPI_READ_BIT);

    /* Lock interrupts here to
     * - avoid concurrent access to the SPI bus
     * - avoid delays in SPI transfer due to unrelated interrupts
     */
    //status = ss_spi_xfer(buf, tx_cnt, rx_cnt);
    // Start transfer.
    uint32_t err_code = nrf_drv_spi_transfer(&m_spi_master,
        m_tx_data, 1, m_rx_data, rx_cnt);

		if(NRF_SUCCESS != err_code) {
			NRF_LOG_RAW_INFO("\t\treg_read ERROR 0x%X. Read Value 0x%X \r\n", 
					err_code, m_rx_data[0]);
			NRF_LOG_FLUSH();
		} /*else {
			NRF_LOG_RAW_INFO("\t\treg_read OK. Read Value 0x%X \r\n", m_rx_data[0]);
			NRF_LOG_FLUSH();
		}*/
    return status;
}


void reg_write(uint8_t reg, uint8_t data)
{
		// rising edge of CS pin switches BMI160 to SPI mode
		//nrf_gpio_pin_clear(BMI160_SPI_ENABLE_PIN);
	
    m_tx_data[0] = reg;
    m_tx_data[1] = data;
    // Start transfer.
    uint32_t err_code = nrf_drv_spi_transfer(&m_spi_master,
        m_tx_data, 2, m_rx_data, 0);

		// set CS pin for next SPI communication
		//nrf_gpio_pin_set(BMI160_SPI_ENABLE_PIN);
		//APP_ERROR_CHECK(err_code);

		if(NRF_SUCCESS == err_code) {
			NRF_LOG_RAW_INFO("\t\treg_write OK\r\n");
			NRF_LOG_FLUSH();
		} else {
			NRF_LOG_RAW_INFO("\t\treg_write ERROR %d\r\n", err_code);
			NRF_LOG_FLUSH();
		}
}

void reg_write_bits(uint8_t reg, uint8_t data, unsigned pos, unsigned len)
{
    uint8_t b = reg_read(reg);
    uint8_t mask = ((1 << len) - 1) << pos;
    data <<= pos; // shift data into correct position
    data &= mask; // zero all non-important bits in data
    b &= ~(mask); // zero all important bits in existing byte
    b |= data; // combine data with existing byte
    reg_write(reg, b);
}

uint8_t reg_read_bits(uint8_t reg, unsigned pos, unsigned len)
{
    uint8_t b = reg_read(reg);
    uint8_t mask = (1 << len) - 1;
    b >>= pos;
    b &= mask;
    return b;
}

/******************************************************************************/

/** Power on and prepare for general usage.
 * This will activate the device and take it out of sleep mode (which must be done
 * after start-up). This function also sets both the accelerometer and the gyroscope
 * to default range settings, namely +/- 2g and +/- 250 degrees/sec.
 */
void bmi160_init()
{
		/**
		 *	Configure CSB pin as output.
		 */
		//nrf_gpio_cfg_output(BMI160_SPI_ENABLE_PIN);
		//nrf_delay_ms(20);
		
	  while(true) {
			if(testConnection()) {
				NRF_LOG_RAW_INFO("\t\t SPI sensor connection OK!\r\n");
				break;
			} else {
				NRF_LOG_RAW_INFO("\t\t SPI sensor connection ERROR!  %d \r\n", SPI_SS_PIN);
			}
			NRF_LOG_FLUSH();
						
			nrf_delay_ms(1000);
		}
		//while(1) {};
		
    /* Issue a soft-reset to bring the device into a clean state */
    reg_write(BMI160_RA_CMD, BMI160_CMD_SOFT_RESET);
	  nrf_delay_ms(100);

    /* Issue a dummy-read to force the device into SPI comms mode */
    reg_read(0x7F);
	  nrf_delay_ms(100);

    /* Power up the accelerometer */
    reg_write(BMI160_RA_CMD, BMI160_CMD_ACC_MODE_NORMAL);
	  nrf_delay_ms(100);
	
    /* Wait for power-up to complete 		*/
    while (0x1 != reg_read_bits(BMI160_RA_PMU_STATUS,
                                BMI160_ACC_PMU_STATUS_BIT,
                                BMI160_ACC_PMU_STATUS_LEN))
				nrf_delay_ms(200);

    /* Power up the gyroscope */
    reg_write(BMI160_RA_CMD, BMI160_CMD_GYR_MODE_NORMAL);
	  nrf_delay_ms(100);
		
    /* Wait for power-up to complete 		*/
    while (0x1 != reg_read_bits(BMI160_RA_PMU_STATUS,
                                BMI160_GYR_PMU_STATUS_BIT,
                                BMI160_GYR_PMU_STATUS_LEN))
				nrf_delay_ms(200);

    setFullScaleGyroRange(BMI160_GYRO_RANGE_250);
    setFullScaleAccelRange(BMI160_ACCEL_RANGE_2G);

    /* Only PIN1 interrupts currently supported - map all interrupts to PIN1 */
    //reg_write(BMI160_RA_INT_MAP_0, 0xFF);
    //reg_write(BMI160_RA_INT_MAP_1, 0xF0);
    //reg_write(BMI160_RA_INT_MAP_2, 0x00);

		nrf_delay_ms(100);
		
	  while(true) {
			if(testConnection()) {
				NRF_LOG_RAW_INFO("\t\t SPI sensor connection OK!\r\n");
				break;
			} else {
				NRF_LOG_RAW_INFO("\t\t SPI sensor connection ERROR!  %d \r\n", SPI_SS_PIN);
			}
			NRF_LOG_FLUSH();						
			nrf_delay_ms(1000);
		}
}

/** Get Device ID.
 * This register is used to verify the identity of the device (0b11010001, 0xD1).
 * @return Device ID (should be 0xD1)
 * @see BMI160_RA_CHIP_ID
 */
uint8_t getDeviceID() {
    return reg_read(BMI160_RA_CHIP_ID);
}

/** Verify the SPI connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
bool testConnection()
{
    return (BMI160_CHIP_ID == getDeviceID());
}

/** Get full-scale gyroscope range.
 * The gyr_range parameter allows setting the full-scale range of the gyro sensors,
 * as described in the table below.
 *
 * <pre>
 * 4 = +/-  125 degrees/sec
 * 3 = +/-  250 degrees/sec
 * 2 = +/-  500 degrees/sec
 * 1 = +/- 1000 degrees/sec
 * 0 = +/- 2000 degrees/sec
 * </pre>
 *
 * @return Current full-scale gyroscope range setting
 * @see BMI160_RA_GYRO_RANGE
 * @see BMI160GyroRange
 */
uint8_t getFullScaleGyroRange() {
    return reg_read_bits(BMI160_RA_GYRO_RANGE,
                         BMI160_GYRO_RANGE_SEL_BIT,
                         BMI160_GYRO_RANGE_SEL_LEN);
}

/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see getFullScaleGyroRange()
 */
void setFullScaleGyroRange(uint8_t range) {
    reg_write_bits(BMI160_RA_GYRO_RANGE, range,
                   BMI160_GYRO_RANGE_SEL_BIT,
                   BMI160_GYRO_RANGE_SEL_LEN);
}

/** Get full-scale accelerometer range.
 * The FS_SEL parameter allows setting the full-scale range of the accelerometer
 * sensors, as described in the table below.
 *
 * <pre>
 *  3 = +/- 2g
 *  5 = +/- 4g
 *  8 = +/- 8g
 * 12 = +/- 16g
 * </pre>
 *
 * @return Current full-scale accelerometer range setting
 * @see BMI160_RA_ACCEL_RANGE
 * @see BMI160AccelRange
 */
uint8_t getFullScaleAccelRange() {
    return reg_read_bits(BMI160_RA_ACCEL_RANGE,
                         BMI160_ACCEL_RANGE_SEL_BIT,
                         BMI160_ACCEL_RANGE_SEL_LEN);
}

/** Set full-scale accelerometer range.
 * @param range New full-scale accelerometer range setting
 * @see getFullScaleAccelRange()
 * @see BMI160AccelRange
 */
void setFullScaleAccelRange(uint8_t range) {
    reg_write_bits(BMI160_RA_ACCEL_RANGE, range,
                   BMI160_ACCEL_RANGE_SEL_BIT,
                   BMI160_ACCEL_RANGE_SEL_LEN);
}

/** Get raw 6-axis motion sensor readings (accel/gyro).
 * Retrieves all currently available motion sensor values.
 * @param ax 16-bit signed integer container for accelerometer X-axis value
 * @param ay 16-bit signed integer container for accelerometer Y-axis value
 * @param az 16-bit signed integer container for accelerometer Z-axis value
 * @param gx 16-bit signed integer container for gyroscope X-axis value
 * @param gy 16-bit signed integer container for gyroscope Y-axis value
 * @param gz 16-bit signed integer container for gyroscope Z-axis value
 * @see getAcceleration()
 * @see getRotation()
 * @see BMI160_RA_GYRO_X_L
 */
void getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
    m_tx_data[0] = BMI160_RA_GYRO_X_L;
	
    serial_buffer_transfer(1, 13);
	  
		/* NRF_LOG_RAW_INFO("\t");	*/
		for(uint8_t i = 0; i < 13; i++) {
			NRF_LOG_RAW_INFO("%X ", m_rx_data[i])
		}
		NRF_LOG_RAW_INFO("\r\n"); NRF_LOG_FLUSH();

    *gx = (((int16_t)m_rx_data[2])  << 8) | m_rx_data[1];
    *gy = (((int16_t)m_rx_data[4])  << 8) | m_rx_data[3];
    *gz = (((int16_t)m_rx_data[6])  << 8) | m_rx_data[5];
    *ax = (((int16_t)m_rx_data[8])  << 8) | m_rx_data[7];
    *ay = (((int16_t)m_rx_data[10])  << 8) | m_rx_data[9];
    *az = (((int16_t)m_rx_data[12]) << 8) | m_rx_data[11];
    //*az = ((m_rx_data[12] & 0xFF ) << 8) | (m_rx_data[11] & 0xFF);
}

void getAcceleration(int16_t* x, int16_t* y, int16_t* z) {
    m_tx_data[0] = BMI160_RA_ACCEL_X_L;
    serial_buffer_transfer(1, 7);
	
		for(uint8_t i = 0; i < 7; i++) {
			NRF_LOG_RAW_INFO("%X ", m_rx_data[i])
		}
		NRF_LOG_RAW_INFO("\r\n"); NRF_LOG_FLUSH();
    *x = (((int16_t)m_rx_data[2]) << 8) | m_rx_data[1];
    *y = (((int16_t)m_rx_data[4]) << 8) | m_rx_data[3];
    *z = (((int16_t)m_rx_data[6]) << 8) | m_rx_data[5];
}
