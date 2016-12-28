/* Copyright (c) 2015 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"
#include "app_error.h"
#include <string.h>
#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "bmi160.h"


#define SPI_INSTANCE  0 /**< SPI instance index. */


//static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
//static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */

//#define TEST_STRING "Nordic"
//static uint8_t       m_tx_buf[] = TEST_STRING;           /**< TX buffer. */
//static uint8_t       m_rx_buf[sizeof(TEST_STRING) + 1];    /**< RX buffer. */
//static const uint8_t m_length = sizeof(m_tx_buf);        /**< Transfer length. */

/**
 * @brief SPI user event handler.
 * @param event
 */
/**
void spi_event_handler(nrf_drv_spi_evt_t const * p_event)
{
    spi_xfer_done = true;
    NRF_LOG_INFO("Transfer completed.\r\n");
    if (m_rx_buf[0] != 0)
    {
        NRF_LOG_INFO(" Received: \r\n");
        NRF_LOG_HEXDUMP_INFO(m_rx_buf, strlen((const char *)m_rx_buf));
    }
}
 */
 
int main(void)
{
    //LEDS_CONFIGURE(BSP_LED_0_MASK);
    //LEDS_OFF(BSP_LED_0_MASK);

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));

		float f = 1.12;
    NRF_LOG_INFO("SPI example %e\r\n", f);
    NRF_LOG_FLUSH();

	/*
    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.ss_pin   = SPI_SS_PIN;
    spi_config.miso_pin = SPI_MISO_PIN;
    spi_config.mosi_pin = SPI_MOSI_PIN;
    spi_config.sck_pin  = SPI_SCK_PIN;
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler));
	*/
	
		uint8_t cnt = 0;
	
		spi_init();
		bmi160_init();

    while (true)
    {
        //__WFE();
				int16_t ax, ay, az, gx, gy, gz;
			  getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
			  cnt++;
				
				if(cnt == 1) {
					float fgx, fgy, fgz;
					fgx = (gx * 250.0) / 32768.0;
					fgy = (gy * 250.0) / 32768.0;
					fgz = (gz * 250.0) / 32768.0;
					NRF_LOG_RAW_INFO("\t(%d %d %d) (%f %f %f)\r\n", ax, ay, az, fgx, fgy, fgz);
					NRF_LOG_FLUSH();
				}
        nrf_delay_ms(500);
        getAcceleration(&ax, &ay, &az);
				if(cnt == 1) {
					NRF_LOG_RAW_INFO("\t(%d %d %d)\r\n", ax, ay, az);
					NRF_LOG_FLUSH();
					cnt = 0;
				}
        nrf_delay_ms(500);
    }
}
