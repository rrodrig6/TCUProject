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

// DATA
// --COUNTERA
#define COUNTERA0_PIN PIO_PD30_IDX
#define COUNTERA1_PIN PIO_PD31_IDX
#define COUNTERA2_PIN PIO_PD27_IDX
#define COUNTERA3_PIN PIO_PA27_IDX
#define COUNTERA4_PIN PIO_PD12_IDX
#define COUNTERA5_PIN PIO_PD11_IDX
#define COUNTERA6_PIN PIO_PA5_IDX
#define COUNTERA7_PIN PIO_PA9_IDX
// --COUNTERB
#define COUNTERB0_PIN PIO_PC17_IDX
#define COUNTERB1_PIN PIO_PD28_IDX
#define COUNTERB2_PIN PIO_PC30_IDX
#define COUNTERB3_PIN PIO_PA0_IDX
#define COUNTERB4_PIN PIO_PB2_IDX
#define COUNTERB5_PIN PIO_PB3_IDX
#define COUNTERB6_PIN PIO_PA19_IDX
#define COUNTERB7_PIN PIO_PC31_IDX
// --COUNTERC
#define COUNTERC0_PIN PIO_PD22_IDX
#define COUNTERC1_PIN PIO_PD20_IDX
#define COUNTERC2_PIN PIO_PD21_IDX
#define COUNTERC3_PIN PIO_PD25_IDX
#define COUNTERC4_PIN PIO_PB1_IDX
#define COUNTERC5_PIN PIO_PB0_IDX
#define COUNTERC6_PIN PIO_PA4_IDX
#define COUNTERC7_PIN PIO_PA3_IDX

// BYTE SELECT
// --COUNTERA (Same for all counters)
#define COUNTERA_SEL0_PIN PIO_PD24_IDX
#define COUNTERA_SEL1_PIN PIO_PA6_IDX
#define COUNTERA_SEL2_PIN PIO_PC19_IDX
#define COUNTERA_SEL3_PIN PIO_PA13_IDX
// --COUNTERB
#define COUNTERB_SEL0_PIN PIO_PD24_IDX
#define COUNTERB_SEL1_PIN PIO_PA6_IDX
#define COUNTERB_SEL2_PIN PIO_PC19_IDX
#define COUNTERB_SEL3_PIN PIO_PA13_IDX
// --COUNTERC
#define COUNTERC_SEL0_PIN PIO_PD24_IDX
#define COUNTERC_SEL1_PIN PIO_PA6_IDX
#define COUNTERC_SEL2_PIN PIO_PC19_IDX
#define COUNTERC_SEL3_PIN PIO_PA13_IDX

// REGISTER CLK
#define COUNTERA_REG_CLK_PIN PIO_PA24_IDX
#define COUNTERB_REG_CLK_PIN PIO_PA24_IDX
#define COUNTERC_REG_CLK_PIN PIO_PA24_IDX

// COUNTER CLEAR
#define COUNTERA_CLR_PIN PIO_PC13_IDX
#define COUNTERB_CLR_PIN PIO_PD26_IDX
#define COUNTERC_CLR_PIN PIO_PA2_IDX

// COUNTER READY
#define COUNTERA_READY_PIN PIO_PA21_IDX
#define COUNTERA_READY_MASK PIO_PA21
#define COUNTERB_READY_PIN PIO_PB4_IDX
#define COUNTERB_READY_MASK PIO_PB4
#define COUNTERC_READY_PIN PIO_PB13_IDX
#define COUNTERC_READY_MASK PIO_PB13


/** IRQ priority for PIO (The lower the value, the greater the priority) */
// [main_def_pio_irq_prior]
#define IRQ_PRIOR_PIO    0
// [main_def_pio_irq_prior]

/** ms */
#define SAMPLE_PERIOD     1000


typedef enum {Byte0, Byte1, Byte2, Byte3}byteSelect;
	
// A
uint8_t counterAoutPins[8] = {COUNTERA0_PIN,
		COUNTERA1_PIN,
		COUNTERA2_PIN,
		COUNTERA3_PIN,
		COUNTERA4_PIN,
		COUNTERA5_PIN,
		COUNTERA6_PIN,
		COUNTERA7_PIN};

uint8_t counterAselPins[4] = {COUNTERA_SEL0_PIN,
		COUNTERA_SEL1_PIN,
		COUNTERA_SEL2_PIN,
		COUNTERA_SEL3_PIN};
		
//	B
uint8_t counterBoutPins[8] = {COUNTERB0_PIN,
		COUNTERB1_PIN,
		COUNTERB2_PIN,
		COUNTERB3_PIN,
		COUNTERB4_PIN,
		COUNTERB5_PIN,
		COUNTERB6_PIN,
		COUNTERB7_PIN};

uint8_t counterBselPins[4] = {COUNTERB_SEL0_PIN,
		COUNTERB_SEL1_PIN,
		COUNTERB_SEL2_PIN,
		COUNTERB_SEL3_PIN};

//	C
uint8_t counterCoutPins[8] = {COUNTERC0_PIN,
	COUNTERC1_PIN,
	COUNTERC2_PIN,
	COUNTERC3_PIN,
	COUNTERC4_PIN,
	COUNTERC5_PIN,
	COUNTERC6_PIN,
COUNTERC7_PIN};

uint8_t counterCselPins[4] = {COUNTERC_SEL0_PIN,
	COUNTERC_SEL1_PIN,
	COUNTERC_SEL2_PIN,
COUNTERC_SEL3_PIN};

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

uint8_t readCounterByte(uint8_t outputPins[], uint8_t selectPins[], byteSelect byte, char ** p_binaryString)
{
	uint8_t volatile readByte = 0;
	uint8_t volatile bit = 0;
	char bitString[9];
	char tempString[2];
	
	
	switch(byte)
	{
		case Byte0:
		ioport_set_pin_level(selectPins[0],LOW);
		ioport_set_pin_level(selectPins[1],HIGH);
		ioport_set_pin_level(selectPins[2],HIGH);
		ioport_set_pin_level(selectPins[3],HIGH);
		break;
		
		case Byte1:
		ioport_set_pin_level(selectPins[0],HIGH);
		ioport_set_pin_level(selectPins[1],LOW);
		ioport_set_pin_level(selectPins[2],HIGH);
		ioport_set_pin_level(selectPins[3],HIGH);
		break;
		
		case Byte2:
		ioport_set_pin_level(selectPins[0],HIGH);
		ioport_set_pin_level(selectPins[1],HIGH);
		ioport_set_pin_level(selectPins[2],LOW);
		ioport_set_pin_level(selectPins[3],HIGH);
		break;
		
		case Byte3:
		ioport_set_pin_level(selectPins[0],HIGH);
		ioport_set_pin_level(selectPins[1],HIGH);
		ioport_set_pin_level(selectPins[2],HIGH);
		ioport_set_pin_level(selectPins[3],LOW);
		break;

		default:
		ioport_set_pin_level(selectPins[0],HIGH);
		ioport_set_pin_level(selectPins[1],HIGH);
		ioport_set_pin_level(selectPins[2],HIGH);
		ioport_set_pin_level(selectPins[3],HIGH);
		break;
	}
	
	waitCount(10000);
	
	// Get all 8 bits
	for (uint8_t i=0; i<8; i++)
	{
		bit = ioport_get_pin_level(outputPins[i]);
		readByte = readByte | (bit<<i);
		sprintf(tempString, "%u", bit);
		bitString[7-i]= tempString[0];
	}
	
	bitString[8]='\0';
	
	ioport_set_pin_level(selectPins[0],HIGH);
	ioport_set_pin_level(selectPins[1],HIGH);
	ioport_set_pin_level(selectPins[2],HIGH);
	ioport_set_pin_level(selectPins[3],HIGH);
	
	strncpy(p_binaryString, bitString, 9);
	
	
	return readByte;
}

static void CountReady_Handler(uint32_t id, uint32_t mask)
{
	// COUNTERA READY
	if(ID_PIOB == id){
		if (COUNTERC_READY_MASK == mask) {
			uint8_t readByte = 0;
			uint32_t readCount = 0;
			char * p_bitString;
			char bitString[9] = "";
			p_bitString = &bitString;
		
			// Register counter outputs
			ioport_set_pin_level(COUNTERC_REG_CLK_PIN, HIGH);
			waitCount(10000);
			ioport_set_pin_level(COUNTERC_REG_CLK_PIN, LOW);
			waitCount(10000);
		
			readByte = readCounterByte(counterCoutPins, counterCselPins, Byte0, p_bitString);
			readCount += (uint32_t) readByte;
			printf("[C0]%s : %u\r\n", bitString, readByte);
			waitCount(10000);
		
			readByte = readCounterByte(counterCoutPins, counterCselPins, Byte1, p_bitString);
			readCount += ((uint32_t) readByte) << 8;
			printf("[C1]%s : %u\r\n", bitString, readByte);
			waitCount(10000);
			
			readByte = readCounterByte(counterCoutPins, counterCselPins, Byte2, p_bitString);
			readCount += ((uint32_t) readByte) << 16;
			printf("[C2]%s : %u\r\n", bitString, readByte);
			waitCount(10000);
			
			readByte = readCounterByte(counterCoutPins, counterCselPins, Byte3, p_bitString);
			readCount += ((uint32_t) readByte) << 24;
			printf("[C3]%s : %u\r\n", bitString, readByte);
			
			// Reset HW counter
			ioport_set_pin_level(COUNTERC_CLR_PIN, LOW);
			waitCount(10000);
			ioport_set_pin_level(COUNTERC_CLR_PIN, HIGH);
			
			printf("Count: %u\r\n", readCount);
			printf("\r\n");
		}
		else if (COUNTERB_READY_MASK == mask) {
			uint8_t readByte = 0;
			uint32_t readCount = 0;
			char * p_bitString;
			char bitString[9] = "";
			p_bitString = &bitString;
			
			// Register counter outputs
			ioport_set_pin_level(COUNTERB_REG_CLK_PIN, HIGH);
			waitCount(10000);
			ioport_set_pin_level(COUNTERB_REG_CLK_PIN, LOW);
			waitCount(10000);
			
			readByte = readCounterByte(counterBoutPins, counterBselPins, Byte0, p_bitString);
			readCount += (uint32_t) readByte;
			printf("[B0]%s : %u\r\n", bitString, readByte);
			waitCount(10000);
			
			readByte = readCounterByte(counterBoutPins, counterBselPins, Byte1, p_bitString);
			readCount += ((uint32_t) readByte) << 8;
			printf("[B1]%s : %u\r\n", bitString, readByte);
			waitCount(10000);
			
			readByte = readCounterByte(counterBoutPins, counterBselPins, Byte2, p_bitString);
			readCount += ((uint32_t) readByte) << 16;
			printf("[B2]%s : %u\r\n", bitString, readByte);
			waitCount(10000);
			
			readByte = readCounterByte(counterBoutPins, counterBselPins, Byte3, p_bitString);
			readCount += ((uint32_t) readByte) << 24;
			printf("[B3]%s : %u\r\n", bitString, readByte);
			
			// Reset HW counter
			ioport_set_pin_level(COUNTERB_CLR_PIN, LOW);
			waitCount(10000);
			ioport_set_pin_level(COUNTERB_CLR_PIN, HIGH);
			
			printf("Count: %u\r\n", readCount);
			printf("\r\n");
		}
		if (ID_PIOA == id && COUNTERA_READY_MASK == mask) {
			uint8_t readByte = 0;
			uint32_t readCount = 0;
			char * p_bitString;
			char bitString[9] = "";
			p_bitString = &bitString;
			
			// Register counter outputs
			ioport_set_pin_level(COUNTERA_REG_CLK_PIN, HIGH);
			waitCount(10000);
			ioport_set_pin_level(COUNTERA_REG_CLK_PIN, LOW);
			waitCount(10000);
			
			readByte = readCounterByte(counterAoutPins, counterAselPins, Byte0, p_bitString);
			readCount += (uint32_t) readByte;
			printf("[A0]%s : %u\r\n", bitString, readByte);
			waitCount(10000);
			
			readByte = readCounterByte(counterAoutPins, counterAselPins, Byte1, p_bitString);
			readCount += ((uint32_t) readByte) << 8;
			printf("[A1]%s : %u\r\n", bitString, readByte);
			waitCount(10000);
			
			readByte = readCounterByte(counterAoutPins, counterAselPins, Byte2, p_bitString);
			readCount += ((uint32_t) readByte) << 16;
			printf("[A2]%s : %u\r\n", bitString, readByte);
			waitCount(10000);
			
			readByte = readCounterByte(counterAoutPins, counterAselPins, Byte3, p_bitString);
			readCount += ((uint32_t) readByte) << 24;
			printf("[A3]%s : %u\r\n", bitString, readByte);
			
			// Reset HW counter
			ioport_set_pin_level(COUNTERA_CLR_PIN, LOW);
			waitCount(10000);
			ioport_set_pin_level(COUNTERA_CLR_PIN, HIGH);
			
			printf("Count: %u\r\n", readCount);
			printf("\r\n");
		}	
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
	for(uint8_t i = 0; i < 8 ; i++){
		ioport_set_pin_dir(counterAoutPins[i], IOPORT_DIR_INPUT);
		ioport_set_pin_dir(counterBoutPins[i], IOPORT_DIR_INPUT);
		ioport_set_pin_dir(counterCoutPins[i], IOPORT_DIR_INPUT);
		ioport_set_pin_mode(counterAoutPins[i], IOPORT_MODE_PULLDOWN);
		ioport_set_pin_mode(counterBoutPins[i], IOPORT_MODE_PULLDOWN);
		ioport_set_pin_mode(counterCoutPins[i], IOPORT_MODE_PULLDOWN);
	}
	
	/////This is for byte shift
	for(uint8_t i = 0; i < 4 ; i++){
		ioport_set_pin_dir(counterAselPins[i], IOPORT_DIR_OUTPUT);
		ioport_set_pin_dir(counterBselPins[i], IOPORT_DIR_OUTPUT);
		ioport_set_pin_dir(counterCselPins[i], IOPORT_DIR_OUTPUT);
	}
	
	ioport_set_pin_dir(COUNTERA_REG_CLK_PIN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(COUNTERB_REG_CLK_PIN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(COUNTERC_REG_CLK_PIN, IOPORT_DIR_OUTPUT);
	
	ioport_set_pin_dir(COUNTERA_CLR_PIN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_mode(COUNTERA_CLR_PIN, IOPORT_MODE_PULLDOWN);
	ioport_set_pin_dir(COUNTERB_CLR_PIN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_mode(COUNTERB_CLR_PIN, IOPORT_MODE_PULLDOWN);
	ioport_set_pin_dir(COUNTERC_CLR_PIN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_mode(COUNTERC_CLR_PIN, IOPORT_MODE_PULLDOWN);
	
	ioport_set_pin_level(COUNTERA_CLR_PIN, countResetPinValue);
	ioport_set_pin_level(COUNTERB_CLR_PIN, countResetPinValue);
	ioport_set_pin_level(COUNTERC_CLR_PIN, countResetPinValue);
	
	//Configure CountRead Pin and Interrupt
	ioport_set_pin_dir(COUNTERA_READY_PIN, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(COUNTERB_READY_PIN, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(COUNTERC_READY_PIN, IOPORT_DIR_INPUT);
	
	ioport_set_pin_mode(COUNTERA_READY_PIN, (IOPORT_MODE_PULLUP | IOPORT_MODE_DEBOUNCE) );
	ioport_set_pin_mode(COUNTERB_READY_PIN, (IOPORT_MODE_PULLUP | IOPORT_MODE_DEBOUNCE) );
	ioport_set_pin_mode(COUNTERC_READY_PIN, (IOPORT_MODE_PULLUP | IOPORT_MODE_DEBOUNCE) );
	
	ioport_set_pin_sense_mode(COUNTERA_READY_PIN, (IOPORT_SENSE_RISING));
	ioport_set_pin_sense_mode(COUNTERB_READY_PIN, (IOPORT_SENSE_RISING));
	ioport_set_pin_sense_mode(COUNTERC_READY_PIN, (IOPORT_SENSE_RISING));
	
	pio_handler_set(PIOA, ID_PIOA, COUNTERA_READY_MASK, (PIO_PULLUP | PIO_DEBOUNCE | PIO_IT_RISE_EDGE), CountReady_Handler);
	pio_handler_set(PIOB, ID_PIOB, (COUNTERB_READY_MASK || COUNTERC_READY_MASK), (PIO_PULLUP | PIO_DEBOUNCE | PIO_IT_RISE_EDGE), CountReady_Handler);
	
	NVIC_EnableIRQ((IRQn_Type) ID_PIOA);
	NVIC_EnableIRQ((IRQn_Type) ID_PIOB);
	
	pio_handler_set_priority(PIOA, (IRQn_Type) ID_PIOA, IRQ_PRIOR_PIO);
	pio_handler_set_priority(PIOB, (IRQn_Type) ID_PIOB, IRQ_PRIOR_PIO);
	
	pio_enable_interrupt(PIOA, COUNTERA_READY_MASK);
	pio_enable_interrupt(PIOB, ( COUNTERB_READY_MASK || COUNTERC_READY_MASK ));
	
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
