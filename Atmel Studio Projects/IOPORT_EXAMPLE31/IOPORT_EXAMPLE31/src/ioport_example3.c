/**
 * \file
 *
 * \brief Common IOPORT service example 3.
 *
 * Copyright (c) 2012-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */

/**
 * \mainpage
 * \section intro Introduction
 * This example demonstrates how to use the common IOPORT service for pin input
 * and output.
 *
 * \section files Main Files
 *  - ioport.h common gpio definitions
 *  - ioport_example3.c example application
 *  - conf_example.h example definitions
 *
 * \section ioport_service_section services/ioport/ioport.h
 * The common IOPORT service is described in
 * \ref ioport_group section.
 *
 * \section device_info Device Info
 * All Atmel SAM, MEGA, XMEGA and UC3 devices can be used. This example has been
 * tested with the following setup:
 *  - Xplain evaluation kit
 *  - EVK1100 evaluation kit
 *  - SAM3N evaluation kit
 *  - SAM4S evaluation kit
 *  - SAM4E evaluation kit
 *  - SAM4L evaluation kit
 *  - SAM4L Xplained Pro kit
 *  - SAM4L8 Xplained Pro kit
 *  - SAM3X evaluation kit
 *  - SAM4N Xplained Pro kit
 *  - STK600 evaluation Kit (Tested with STK600-ATMEGA128RFA1)
 *  - ATmega256RFR2 Xplained Pro kit
 *  - SAM4C evaluation kit
 *  - SAMG53 Xplained Pro kit
 *  - SAMG55 Xplained Pro kit
 *  - SAMV71 Xplained Ultra kit
 *  - SAME70 Xplained Pro kit
 *
 * \section example_description Description of the example
 * This example reads input from a button and outputs it on a LED.
 *
 * \section compinfo Compilation Info
 * This software was written for the GNU GCC and IAR for SAM and AVR.
 * Other compilers may or may not work.
 *
 * \section contactinfo Contact Information
 * For further information, visit <a href="http://www.atmel.com/">Atmel</a>.\n
 * Support and FAQ: https://www.microchip.com/support/
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */
#include <asf.h>
#include <string.h>
#include "conf_example.h"

#define STRING_EOL    "\r\n"

#define BIT0_PIN PIO_PD30_IDX
#define BIT1_PIN PIO_PD28_IDX
#define BIT2_PIN PIO_PD27_IDX
#define BIT3_PIN PIO_PA27_IDX
#define BIT4_PIN PIO_PD12_IDX
#define BIT5_PIN PIO_PD11_IDX
#define BIT6_PIN PIO_PA5_IDX
#define BIT7_PIN PIO_PA9_IDX

/**
 *  \brief Configure UART console.
 */
static void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
#if (defined CONF_UART_CHAR_LENGTH)
		.charlength = CONF_UART_CHAR_LENGTH,
#endif
		.paritytype = CONF_UART_PARITY,
#if (defined CONF_UART_STOP_BITS)
		.stopbits = CONF_UART_STOP_BITS,
#endif
	};

	/* Configure UART console. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
#if defined(__GNUC__)
	setbuf(stdout, NULL);
#endif
}

int main(void)
{
	sysclk_init();
	board_init();
	ioport_init();
	
	/* Configure debug UART */
	configure_console();
	
	puts("Starting\r\n\r\n");
	
	bool level = true;
	bool prevlevel = true;
	char status [4] = "OFF";
	
	bool bit0 = false;
	bool bit1 = false;
	bool bit2 = false;
	bool bit3 = false;
	bool bit4 = false;
	bool bit5 = false;
	bool bit6 = false;
	bool bit7 = false;

	/* Set output direction on the given LED IOPORTs */
	ioport_set_pin_dir(EXAMPLE_LED, IOPORT_DIR_OUTPUT);

	/* Set direction and pullup on the given button IOPORT */
	ioport_set_pin_dir(EXAMPLE_BUTTON, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(EXAMPLE_BUTTON, IOPORT_MODE_PULLUP);
	
	// Set counter Y read pins
	ioport_set_pin_dir(BIT0_PIN, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(BIT1_PIN, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(BIT2_PIN, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(BIT3_PIN, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(BIT4_PIN, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(BIT5_PIN, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(BIT6_PIN, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(BIT7_PIN, IOPORT_DIR_INPUT);
	
	
	

	while (true) {
		/* Get value from button and output it on led */
		level = ioport_get_pin_level(EXAMPLE_BUTTON);
		if(level != prevlevel)
		{
			if(level)
			{
				//strncpy(status, "ON", 3);
				bit0 = ioport_get_pin_level(BIT0_PIN);
				bit1 = ioport_get_pin_level(BIT1_PIN);
				bit2 = ioport_get_pin_level(BIT2_PIN);
				bit3 = ioport_get_pin_level(BIT3_PIN);
				bit4 = ioport_get_pin_level(BIT4_PIN);
				bit5 = ioport_get_pin_level(BIT5_PIN);
				bit6 = ioport_get_pin_level(BIT6_PIN);
				bit7 = ioport_get_pin_level(BIT7_PIN);
				
				printf("Read Data\r\n");
				printf("%d", bit7);
				printf("%d", bit6);
				printf("%d", bit5);
				printf("%d", bit4);
				printf("%d", bit3);
				printf("%d", bit2);
				printf("%d", bit1);
				printf("%d\r\n", bit0);
				
				
				
				
				
				
				
			}
			else
			{
				//strncpy(status, "OFF", 3);
			}
				
			//printf("Pin is %s\r\n", status);
		}
		
		ioport_set_pin_level(EXAMPLE_LED,
				!level);
		prevlevel = level;
	}
}
