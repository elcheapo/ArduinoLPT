/* Copyright (c) 2007 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT. 
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRENTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 * $LastChangedRevision: 2310 $
 */ 

/** @file
 * @ingroup Main
 * Radio functions.
 *
 * This file handles all radio communication for the example application, i.e. 
 * radio_init, radio_send_packet and radio_interrupt function.
 *
 * @author Per Kristian Schanke
 */

#include "Arduino.h"
#include <SPI.h>
#include "../../../src/config.h"
#include "radio.h"


void radio_send_packet(uint8_t *packet, uint8_t length) {
	hal_nrf_write_tx_payload(packet, length);      // load message into radio
	CE_PULSE();                                 // send packet
}
/*
void radio_send_packet_no_ack(uint8_t *packet, uint8_t length) {
	hal_nrf_write_multibyte_reg(W_TX_PAYLOAD_NO_ACK, packet, length);      // load message into radio
	CE_PULSE();                                 // send packet
}
 */

#define ACK_PL
#define AUTO_ACK

/* For Elcheapo Nano Station */

/** The address of the radio. Parameter to the radio init LSB first */
const uint8_t NRF_address1[HAL_NRF_AW_5BYTES] = {0xaa,'s','r','a','M'};
const uint8_t NRF_address2[HAL_NRF_AW_5BYTES] = {0x5a,'s','r','a','M'}; // Existing remote 1
const uint8_t NRF_address3 = 0x55; // Existing remote 2
const uint8_t NRF_address4 = 0xa5; // Existing remote 3



uint8_t radio_pl_init_prx (void) {

	//	hal_spi_init(8000000);						// Init SPI at 8 MHz
	CE_LOW();        // Set Chip Enable (CE) pin low during chip init

	hal_nrf_write_reg(EN_RXADDR, 0);	 // First close all radio pipes
	hal_nrf_write_reg(EN_AA, 0);

	// Pipe 0/1/2/3 open with autoack
	hal_nrf_write_reg(EN_RXADDR, 0x0f);
	hal_nrf_write_reg(EN_AA, 0x0f);

	hal_nrf_write_reg(SETUP_AW, HAL_NRF_AW_5BYTES - 2); // 5 bytes address width
	hal_nrf_write_reg(SETUP_RETR, (((RF_RETRANS_DELAY/250)-1)<<4) | RF_RETRANSMITS);
	hal_nrf_write_reg(RF_CH, RF_CHANNEL);
	// Frequency = 2400 + RF_CHANNEL
//	hal_nrf_write_reg(RF_SETUP, 0x0e) ;
	//2 Mbits - not test PLL - 0dBm - default settings
	hal_nrf_write_reg(RF_SETUP, 0x26) ;
	//250 kbits - not test PLL - 0dBm - default settings

	// Write addresses LSB first
	hal_nrf_write_multibyte_reg(HAL_NRF_PIPE0, NRF_address1, HAL_NRF_AW_5BYTES);
	hal_nrf_write_multibyte_reg(HAL_NRF_TX, NRF_address1, HAL_NRF_AW_5BYTES); // Not used in PRX
	hal_nrf_write_multibyte_reg(HAL_NRF_PIPE1, NRF_address2, HAL_NRF_AW_5BYTES);
	hal_nrf_write_reg(HAL_NRF_PIPE2, NRF_address3);
	hal_nrf_write_reg(HAL_NRF_PIPE3, NRF_address4);

	hal_nrf_write_reg(RX_PW_P0, RF_PAYLOAD_LENGTH);
	hal_nrf_write_reg(RX_PW_P1, RF_PAYLOAD_LENGTH);
	hal_nrf_write_reg(RX_PW_P2, RF_PAYLOAD_LENGTH);
	hal_nrf_write_reg(RX_PW_P3, RF_PAYLOAD_LENGTH);

	hal_nrf_write_reg(DYNPD, 0x0f);			// Sets up dynamic payload on data pipe 0,1,2,3.
	hal_nrf_write_reg(FEATURE, 0x07);  // Enable dynamic payload, enable ack payload
	hal_nrf_write_reg(CONFIG, 0b00001111);
	// IRQ on, EN_CRC, 2 bytes CRC, PWR UP, PRX
	delay(2);
	hal_nrf_write_reg(STATUS, 0x70);			// Clear pending IRQ
	hal_nrf_flush_tx(); 						// flush tx fifo, to start clean
	CE_HIGH();        // Set Chip Enable (CE) to high to allow reception

	return hal_nrf_get_status();


}

/*
 * Return a packet if the radio has received a valid packet - returns immediately if no packet is received or if an invalid packet is received (>32)
 */
uint8_t radio_get_packet(uint8_t * packet, uint8_t & count, uint8_t & radio_id) {
	uint8_t status, fifo_status;

	fifo_status = hal_nrf_read_reg(FIFO_STATUS);
	status = hal_nrf_get_status();
#if 0
	if ((status != 0x0e) && (fifo_status != 0x11)) {
		Serial.print(F("FIFO/STATUS :"));
		Serial.print(fifo_status,16);
		Serial.write('/');
		Serial.println(status,16);
	}
#endif
	if ((fifo_status & 0x01) == 0) { // a packet is available
		// get it
		count = hal_nrf_read_reg(R_RX_PL_WID);
		if (( count > 32) || (count == 0)) {
			hal_nrf_flush_rx();
			hal_nrf_get_clear_irq_flags();
			return false;
		} else {
			radio_id = (status & 0x0f) >> 1;
			hal_nrf_read_multibyte_reg(R_RX_PAYLOAD, packet, count);
			// clear IRQ source
			hal_nrf_get_clear_irq_flags();
			return true;
		}
	} else if ((status & (1<<HAL_NRF_MAX_RT)) != 0 ) { // Max Retry, flush TX
		hal_nrf_flush_tx(); 		// flush tx fifo, avoid fifo jam
		hal_nrf_get_clear_irq_flags();
		return false;
		// TO BE CHECKED .... but does not seem to happen ...
	};
	return false;
}

