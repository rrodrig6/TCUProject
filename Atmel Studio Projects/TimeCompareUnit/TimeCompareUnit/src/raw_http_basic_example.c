/**
 * \file
 *
 * \brief lwIP Raw HTTP basic example.
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
 *  \mainpage lwIP Raw HTTP basic example
 *
 *  \section Purpose
 *  This documents data structures, functions, variables, defines, enums, and
 *  typedefs in the software for the lwIP Raw HTTP basic example.
 *
 *  The given example is a lwIP example using the current lwIP stack and MAC driver.
 *
 *  \section Requirements
 *
 *  This package can be used with SAM3X-EK,SAM4E-EK,SAMV71 and SAME70.
 *
 *  \section Description
 *
 *  This example features a simple lwIP web server.
 *  - Plug the Ethernet cable directly into the evaluation kit to connect to the PC.
 *  - Configuring the PC network port to local mode to setup a 'point to point' network.
 *  - Start the example.
 *  - Launch your favorite web browser.
 *  - Type the WEB server example IP address in your browser's address bar.
 *
 *  \section Usage
 *
 *  -# Build the program and download it into the evaluation board. Please
 *     refer to the
 *     <a href="http://www.atmel.com/dyn/resources/prod_documents/6421B.pdf">
 *     SAM-BA User Guide</a>, the
 *     <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6310.pdf">
 *     GNU-Based Software Development</a>
 *     application note or the
 *     <a href="http://www.iar.com/website1/1.0.1.0/78/1/">
 *     IAR EWARM User and reference guides</a>,
 *     depending on the solutions that users choose.
 *  -# On the computer, open and configure a terminal application
 *     (e.g., HyperTerminal on Microsoft Windows) with these settings:
 *    - 115200 bauds
 *    - 8 bits of data
 *    - No parity
 *    - 1 stop bit
 *    - No flow control
 *  -# In the terminal window, the
 *     following text should appear (if DHCP mode is not enabled):
 *     \code
 *      Network up IP==xxx.xxx.xxx.xxx
 *      Static IP Address Assigned
 *     \endcode
 *
 */

#include <asf.h>
#include <string.h>
#include "conf_example.h"
#include "sysclk.h"
#include "ioport.h"
#include "stdio_serial.h"
#include "ethernet.h"
#include "httpd.h"

#define STRING_EOL    "\r\n"
#define STRING_HEADER "-- Raw HTTP Basic Example --"STRING_EOL \
		"-- "BOARD_NAME" --"STRING_EOL \
		"-- Compiled: "__DATE__" "__TIME__" --"STRING_EOL

//////Declaring the 8-bits to read /////////////////////
#define TIMERA0_PIN PIO_PD30_IDX	//Pin 28
#define TIMERA1_PIN PIO_PD28_IDX	//Pin 27
#define TIMERA2_PIN PIO_PD27_IDX	//Pin 26
#define TIMERA3_PIN PIO_PA_27_IDX	//Pin 25
#define TIMERA4_PIN PIO_PD12_IDX	//Pin 24
#define TIMERA5_PIN PIO_PD11_IDX	//Pin 23
#define TIMERA6_PIN PIO_PA5_IDX		//Pin 22
#define TIMERA7_PIN PIO_PA9_IDX		//Pin 21

/////Declaring the 4 GPIOs to cycle through bytes///////
#define TIMERA_SEL0_PIN PIO_PB3_IDX	//Pin20
#define TIMERA_SEL1_PIN PIO_PD21_IDX//Pin 19
#define TIMERA_SEL2_PIN PIO_PD22_IDX//Pin 18
#define TIMERA_SEL3_PIN PIO_PA24_IDX//Pin 16

////Declaring the shift reg, the clear
#define TIMERA_REG_CLK_PIN PIO_PD24_IDX	//Pin 14
#define TIMERA_CLR_PIN PIO_PA13_IDX		//Pin12
#define TIMERA_READY_PIN PIO_PC19_IDX
#define TIMERA_READY_MASK PIO_PC19


/** IRQ priority for PIO (The lower the value, the greater the priority) */
// [main_def_pio_irq_prior]
#define IRQ_PRIOR_PIO    0
// [main_def_pio_irq_prior]

/** ms */
#define SAMPLE_PERIOD     1000


typedef enum {Byte0, Byte1, Byte2, Byte3}byteSelect;

/** Global g_ul_ms_ticks in milliseconds since start of application */
// [main_var_ticks]
volatile uint32_t g_ul_ms_ticks = 0;
// [main_var_ticks]

static void mdelay(uint32_t ul_dly_ticks)
{
	uint32_t ul_cur_ticks;

	ul_cur_ticks = g_ul_ms_ticks;
	while ((g_ul_ms_ticks - ul_cur_ticks) < ul_dly_ticks);
}

static void waitCount(uint32_t ticks)
{
	for(uint32_t volatile i = ticks; i>0; i--)
	{
	}
}

uint8_t readTimerByteA(byteSelect byte, char ** p_binaryString)
{
	uint8_t volatile readByte = 0;
	uint8_t volatile bit = 0;
	char bitString[9];
	char tempString[2];
	
	
	switch(byte)
	{
		case Byte0:
		ioport_set_pin_level(TIMERA_SEL0_PIN,LOW);
		ioport_set_pin_level(TIMERA_SEL1_PIN,HIGH);
		ioport_set_pin_level(TIMERA_SEL2_PIN,HIGH);
		ioport_set_pin_level(TIMERA_SEL3_PIN,HIGH);
		break;
		
		case Byte1:
		ioport_set_pin_level(TIMERA_SEL0_PIN,HIGH);
		ioport_set_pin_level(TIMERA_SEL1_PIN,LOW);
		ioport_set_pin_level(TIMERA_SEL2_PIN,HIGH);
		ioport_set_pin_level(TIMERA_SEL3_PIN,HIGH);
		break;
		
		case Byte2:
		ioport_set_pin_level(TIMERA_SEL0_PIN,HIGH);
		ioport_set_pin_level(TIMERA_SEL1_PIN,HIGH);
		ioport_set_pin_level(TIMERA_SEL2_PIN,LOW);
		ioport_set_pin_level(TIMERA_SEL3_PIN,HIGH);
		break;
		
		case Byte3:
		ioport_set_pin_level(TIMERA_SEL0_PIN,HIGH);
		ioport_set_pin_level(TIMERA_SEL1_PIN,HIGH);
		ioport_set_pin_level(TIMERA_SEL2_PIN,HIGH);
		ioport_set_pin_level(TIMERA_SEL3_PIN,LOW);
		break;

		default:
		ioport_set_pin_level(TIMERA_SEL0_PIN,HIGH);
		ioport_set_pin_level(TIMERA_SEL1_PIN,HIGH);
		ioport_set_pin_level(TIMERA_SEL2_PIN,HIGH);
		ioport_set_pin_level(TIMERA_SEL3_PIN,HIGH);
		break;
	}
	
	waitCount(10000);
	bit = ioport_get_pin_level(TIMERA0_PIN);
	readByte = readByte | bit;
	sprintf(tempString, "%u", bit);
	bitString[7]= tempString[0];
	bit = ioport_get_pin_level(TIMERA1_PIN);
	readByte = readByte | (bit<<1);
	sprintf(tempString, "%u", bit);
	bitString[6]= tempString[0];
	bit = ioport_get_pin_level(TIMERA2_PIN);
	readByte = readByte | (bit<<2);
	sprintf(tempString, "%u", bit);
	bitString[5]= tempString[0];
	bit = ioport_get_pin_level(TIMERA3_PIN);
	readByte = readByte | (bit<<3);
	sprintf(tempString, "%u", bit);
	bitString[4]= tempString[0];
	bit = ioport_get_pin_level(TIMERA4_PIN);
	readByte = readByte | (bit<<4);
	sprintf(tempString, "%u", bit);
	bitString[3]= tempString[0];
	bit = ioport_get_pin_level(TIMERA5_PIN);
	readByte = readByte | (bit<<5);
	sprintf(tempString, "%u", bit);
	bitString[2]= tempString[0];
	bit = ioport_get_pin_level(TIMERA6_PIN);
	readByte = readByte | (bit<<6);
	sprintf(tempString, "%u", bit);
	bitString[1]= tempString[0];
	bit = ioport_get_pin_level(TIMERA7_PIN);
	readByte = readByte | (bit<<7);
	sprintf(tempString, "%u", bit);
	bitString[0]= tempString[0];
	
	bitString[8]='\0';
	
	ioport_set_pin_level(TIMERA_SEL0_PIN,HIGH);
	ioport_set_pin_level(TIMERA_SEL1_PIN,HIGH);
	ioport_set_pin_level(TIMERA_SEL2_PIN,HIGH);
	ioport_set_pin_level(TIMERA_SEL3_PIN,HIGH);
	
	strncpy(p_binaryString, bitString, 9);
	
	
	return readByte;
}

static void CountReady_Handler(uint32_t id, uint32_t mask)
{
	if (ID_PIOC == id && TIMERA_READY_MASK == mask) {
		uint8_t readByte = 0;
		uint32_t readCount = 0;
		char * p_bitString;
		char bitString[9] = "";
		p_bitString = &bitString;
		
		// Register counter outputs
		ioport_set_pin_level(TIMERA_REG_CLK_PIN, HIGH);
		waitCount(10000);
		ioport_set_pin_level(TIMERA_REG_CLK_PIN, LOW);
		waitCount(10000);
		
		readByte = readTimerByteA(Byte0, p_bitString);
		readCount += (uint32_t) readByte;
		printf("[1]%s : %u\r\n", bitString, readByte);
		waitCount(10000);
		
		readByte = readTimerByteA(Byte1, p_bitString);
		readCount += ((uint32_t) readByte) << 8;
		printf("[2]%s : %u\r\n", bitString, readByte);
		waitCount(10000);
		
		readByte = readTimerByteA(Byte2, p_bitString);
		readCount += ((uint32_t) readByte) << 16;
		printf("[3]%s : %u\r\n", bitString, readByte);
		waitCount(10000);
		
		readByte = readTimerByteA(Byte3, p_bitString);
		readCount += ((uint32_t) readByte) << 24;
		printf("[4]%s : %u\r\n", bitString, readByte);
		
		// Reset HW counter
		ioport_set_pin_level(TIMERA_CLR_PIN, LOW);
		waitCount(10000);
		ioport_set_pin_level(TIMERA_CLR_PIN, HIGH);
		
		printf("Count: %u\r\n", readCount);
		printf("\r\n");
	}
}

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

/**
 * \brief Main program function. Configure the hardware, initialize lwIP
 * TCP/IP stack, and start HTTP service.
 */
int main(void)
{
	/* Initialize the SAM system. */
	sysclk_init();
	board_init();

	/* Configure debug UART */
	configure_console();

	/* Print example information. */
	//puts(STRING_HEADER);

	/* Bring up the ethernet interface & initialize timer0, channel0. */
	//init_ethernet();

	/* Bring up the web server. */
	//httpd_init();
	
	printf("---Starting---\r\n\r\n");
	
	
	
	
	bool countResetPinValue = true;
	
	// Set counter Y read pins
	ioport_set_pin_dir(TIMERA0_PIN, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(TIMERA0_PIN, IOPORT_MODE_PULLDOWN);
	ioport_set_pin_dir(TIMERA1_PIN, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(TIMERA1_PIN, IOPORT_MODE_PULLDOWN);
	ioport_set_pin_dir(TIMERA2_PIN, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(TIMERA2_PIN, IOPORT_MODE_PULLDOWN);
	ioport_set_pin_dir(TIMERA3_PIN, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(TIMERA3_PIN, IOPORT_MODE_PULLDOWN);
	ioport_set_pin_dir(TIMERA4_PIN, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(TIMERA4_PIN, IOPORT_MODE_PULLDOWN);
	ioport_set_pin_dir(TIMERA5_PIN, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(TIMERA5_PIN, IOPORT_MODE_PULLDOWN);
	ioport_set_pin_dir(TIMERA6_PIN, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(TIMERA6_PIN, IOPORT_MODE_PULLDOWN);
	ioport_set_pin_dir(TIMERA7_PIN, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(TIMERA7_PIN, IOPORT_MODE_PULLDOWN);
	
	/////This is for byte shift
	ioport_set_pin_dir(TIMERA_SEL0_PIN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(TIMERA_SEL1_PIN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(TIMERA_SEL2_PIN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(TIMERA_SEL3_PIN, IOPORT_DIR_OUTPUT);
	
	ioport_set_pin_dir(TIMERA_REG_CLK_PIN, IOPORT_DIR_OUTPUT);
	
	ioport_set_pin_dir(TIMERA_CLR_PIN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_mode(TIMERA_CLR_PIN, IOPORT_MODE_PULLDOWN);
	
	ioport_set_pin_level(TIMERA_CLR_PIN, countResetPinValue);
	
	//Configure CountRead Pin and Interrupt
	ioport_set_pin_dir(TIMERA_READY_PIN, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(TIMERA_READY_PIN, (IOPORT_MODE_PULLUP | IOPORT_MODE_DEBOUNCE) );
	ioport_set_pin_sense_mode(TIMERA_READY_PIN, (IOPORT_SENSE_RISING));
	pio_handler_set(PIOC, ID_PIOC,
	COUNT_READY_MASK, (PIO_PULLUP | PIO_DEBOUNCE | PIO_IT_RISE_EDGE), CountReady_Handler);
	NVIC_EnableIRQ((IRQn_Type) ID_PIOC);
	pio_handler_set_priority(PIOC,
	(IRQn_Type) ID_PIOC, IRQ_PRIOR_PIO);
	pio_enable_interrupt(PIOC, TIMERA_READY_MASK);
	
	if (SysTick_Config(sysclk_get_cpu_hz() / 1000)) {
		printf("-F- Systick configuration error\r\n\r\n");
		while (1);
	}
	
	printf("I/O Configured\r\n\r\n");
	
	while (true) {
		
	}

	/* Program main loop.
	while (1) {
		// Check for input packet and process it.
		ethernet_task();
	}
	*/
}
