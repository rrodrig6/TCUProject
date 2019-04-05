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
//////Declaring the 8-bits to read /////////////////////
#define BIT0_PIN PIO_PD30_IDX      //Pin 28
#define BIT1_PIN PIO_PD28_IDX      //Pin 27
#define BIT2_PIN PIO_PD27_IDX      //Pin 26
#define BIT3_PIN PIO_PA27_IDX      //Pin 25 
#define BIT4_PIN PIO_PD12_IDX      //Pin 24
#define BIT5_PIN PIO_PD11_IDX      //Pin 23
#define BIT6_PIN PIO_PA5_IDX       //Pin 22
#define BIT7_PIN PIO_PA9_IDX       //Pin 21
/////Declaring the 4 GPIOs to cycle through bytes///////
#define BYTE1_SHIFT PIO_PB3_IDX    //Pin 20
#define BYTE2_SHIFT PIO_PD21_IDX    //Pin 19
#define BYTE3_SHIFT PIO_PD22_IDX    //Pin 18
#define BYTE4_SHIFT PIO_PA24_IDX    //Pin 16
////Declaring the shift reg, the clear
#define CLK_SHIFT PIO_PD24_IDX    //Pin 14
#define CLEAR PIO_PD25_IDX        //Pin 12


/** IRQ priority for PIO (The lower the value, the greater the priority) */
// [main_def_pio_irq_prior]
#define IRQ_PRIOR_PIO    0
// [main_def_pio_irq_prior]

/** ms */
#define SAMPLE_PERIOD     1000


typedef enum {None, Byte1, Byte2, Byte3, Byte4}byteSelect;

/** Global g_ul_ms_ticks in milliseconds since start of application */
// [main_var_ticks]
volatile uint32_t g_ul_ms_ticks = 0;
// [main_var_ticks]

/**
 *  \brief Handler for System Tick interrupt.
 *
 *  Process System Tick Event
 *  Increments the g_ul_ms_ticks counter.
 */
// [main_systick_handler]
void SysTick_Handler(void)
{
	g_ul_ms_ticks++;
}
// [main_systick_handler]


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

static void mdelay(uint32_t ul_dly_ticks)
{
	uint32_t ul_cur_ticks;

	ul_cur_ticks = g_ul_ms_ticks;
	while ((g_ul_ms_ticks - ul_cur_ticks) < ul_dly_ticks);
}

uint8_t readTimerByte(byteSelect byte, char ** p_binaryString)
{
	uint8_t volatile readByte = 0;
	uint8_t volatile bit = 0;
	char bitString[9];
	char tempString[2];
	
	
	switch(byte)
	{
		case Byte1:
			ioport_set_pin_level(BYTE1_SHIFT,LOW);
			ioport_set_pin_level(BYTE2_SHIFT,HIGH);
			ioport_set_pin_level(BYTE3_SHIFT,HIGH);
			ioport_set_pin_level(BYTE4_SHIFT,HIGH);
			break;
			
		case Byte2:
			ioport_set_pin_level(BYTE1_SHIFT,HIGH);
			ioport_set_pin_level(BYTE2_SHIFT,LOW);
			ioport_set_pin_level(BYTE3_SHIFT,HIGH);
			ioport_set_pin_level(BYTE4_SHIFT,HIGH);
			break;
			
		case Byte3:
			ioport_set_pin_level(BYTE1_SHIFT,HIGH);
			ioport_set_pin_level(BYTE2_SHIFT,HIGH);
			ioport_set_pin_level(BYTE3_SHIFT,LOW);
			ioport_set_pin_level(BYTE4_SHIFT,HIGH);
			break;
			
		case Byte4:
			ioport_set_pin_level(BYTE1_SHIFT,HIGH);
			ioport_set_pin_level(BYTE2_SHIFT,HIGH);
			ioport_set_pin_level(BYTE3_SHIFT,HIGH);
			ioport_set_pin_level(BYTE4_SHIFT,LOW);
			break;	
			
		case None:
		default:
			ioport_set_pin_level(BYTE1_SHIFT,HIGH);
			ioport_set_pin_level(BYTE2_SHIFT,HIGH);
			ioport_set_pin_level(BYTE3_SHIFT,HIGH);
			ioport_set_pin_level(BYTE4_SHIFT,HIGH);
			break;
	}
	
	mdelay(1);
	bit = ioport_get_pin_level(BIT0_PIN);
	readByte = readByte | bit;
	sprintf(tempString, "%u", bit);
	bitString[7]= tempString[0];
	bit = ioport_get_pin_level(BIT1_PIN);
	readByte = readByte | (bit<<1);
	sprintf(tempString, "%u", bit);
	bitString[6]= tempString[0];
	bit = ioport_get_pin_level(BIT2_PIN);
	readByte = readByte | (bit<<2);
	sprintf(tempString, "%u", bit);
	bitString[5]= tempString[0];
	bit = ioport_get_pin_level(BIT3_PIN);
	readByte = readByte | (bit<<3);
	sprintf(tempString, "%u", bit);
	bitString[4]= tempString[0];
	bit = ioport_get_pin_level(BIT4_PIN);
	readByte = readByte | (bit<<4);
	sprintf(tempString, "%u", bit);
	bitString[3]= tempString[0];
	bit = ioport_get_pin_level(BIT5_PIN);
	readByte = readByte | (bit<<5);
	sprintf(tempString, "%u", bit);
	bitString[2]= tempString[0];
	bit = ioport_get_pin_level(BIT6_PIN);
	readByte = readByte | (bit<<6);
	sprintf(tempString, "%u", bit);
	bitString[1]= tempString[0];
	bit = ioport_get_pin_level(BIT7_PIN);
	readByte = readByte | (bit<<7);
	sprintf(tempString, "%u", bit);
	bitString[0]= tempString[0];
	
	bitString[8]='\0';
	
	ioport_set_pin_level(BYTE1_SHIFT,HIGH);
	ioport_set_pin_level(BYTE2_SHIFT,HIGH);
	ioport_set_pin_level(BYTE3_SHIFT,HIGH);
	ioport_set_pin_level(BYTE4_SHIFT,HIGH);
	
	strncpy(p_binaryString, bitString, 9);
	
	
	return readByte;
}


int main(void)
{
	sysclk_init();
	board_init();
	ioport_init();
	
	/* Configure debug UART */
	configure_console();
	
	printf("---Starting---\r\n\r\n");
	
	
	uint32_t volatile count = 0;
	uint8_t volatile readByte = 0;
	uint32_t readCount = 0;
	char * p_bitString;
	char bitString[9] = "ABCDEFGH";
	p_bitString = &bitString;
	
	// Set counter Y read pins
	ioport_set_pin_dir(BIT0_PIN, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(BIT0_PIN, IOPORT_MODE_PULLDOWN);
	ioport_set_pin_dir(BIT1_PIN, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(BIT1_PIN, IOPORT_MODE_PULLDOWN);
	ioport_set_pin_dir(BIT2_PIN, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(BIT2_PIN, IOPORT_MODE_PULLDOWN);
	ioport_set_pin_dir(BIT3_PIN, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(BIT3_PIN, IOPORT_MODE_PULLDOWN);
	ioport_set_pin_dir(BIT4_PIN, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(BIT4_PIN, IOPORT_MODE_PULLDOWN);
	ioport_set_pin_dir(BIT5_PIN, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(BIT5_PIN, IOPORT_MODE_PULLDOWN);
	ioport_set_pin_dir(BIT6_PIN, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(BIT6_PIN, IOPORT_MODE_PULLDOWN);
	ioport_set_pin_dir(BIT7_PIN, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(BIT7_PIN, IOPORT_MODE_PULLDOWN);
	
	/////This is for byte shift
	ioport_set_pin_dir(BYTE1_SHIFT, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(BYTE2_SHIFT, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(BYTE3_SHIFT, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(BYTE4_SHIFT, IOPORT_DIR_OUTPUT);
	
	ioport_set_pin_dir(CLK_SHIFT, IOPORT_DIR_OUTPUT);
	if (SysTick_Config(sysclk_get_cpu_hz() / 1000)) {
		printf("-F- Systick configuration error\r\n\r\n");
		while (1);
	}
	
	printf("I/O Configured\r\n\r\n");
	
	while (true) {
		volatile uint32_t ul_dummy;
		volatile readCount = 0;
		
		// Register counter outputs
		ioport_set_pin_level(CLK_SHIFT, HIGH);
		mdelay(1);
		ioport_set_pin_level(CLK_SHIFT, LOW);
		mdelay(1);
		
		
		readByte = readTimerByte(Byte1, p_bitString);
		readCount += (uint32_t) readByte;
		printf("[1]%s : %u\r\n", bitString, readByte);
		mdelay(1);
		
		readByte = readTimerByte(Byte2, p_bitString);
		readCount += ((uint32_t) readByte) << 8;
		printf("[2]%s : %u\r\n", bitString, readByte);
		mdelay(1);
		
		readByte = readTimerByte(Byte3, p_bitString);
		readCount += ((uint32_t) readByte) << 16;
		printf("[3]%s : %u\r\n", bitString, readByte);
		mdelay(1);
		
		readByte = readTimerByte(Byte4, p_bitString);
		readCount += ((uint32_t) readByte) << 24;
		printf("[4]%s : %u\r\n", bitString, readByte);
		
		printf("Count: %u\r\n\r\n", readCount);
		
		mdelay(1000);
	}
}
