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
//#define COUNTERA_READY_PIN PIO_PA21_IDX
//#define COUNTERA_READY_MASK PIO_PA21
//#define COUNTERB_READY_PIN PIO_PB4_IDX
//#define COUNTERB_READY_MASK PIO_PB4
//#define COUNTERC_READY_PIN PIO_PB13_IDX
//#define COUNTERC_READY_MASK PIO_PB13

// SIGNAL READY
// --Pair A
#define SIGNAL1A_READY_PIN PIO_PA22_IDX
#define SIGNAL1A_READY_MASK PIO_PA22
#define SIGNAL2A_READY_PIN PIO_PA17_IDX
#define SIGNAL2A_READY_MASK PIO_PA17
// --Pair B
#define SIGNAL1B_READY_PIN PIO_PC12_IDX
#define SIGNAL1B_READY_MASK PIO_PC12
#define SIGNAL2B_READY_PIN PIO_PC9_IDX
#define SIGNAL2B_READY_MASK PIO_PC9

/** IRQ priority for PIO (The lower the value, the greater the priority) */
// [main_def_pio_irq_prior]
#define IRQ_PRIOR_PIO    0
// [main_def_pio_irq_prior]

/** ms */
#define SAMPLE_PERIOD     1000
	
struct counter
{
	uint8_t selectPins[4];
	uint8_t outputPins[8];
	uint8_t registerClkPin;
	uint8_t clearPin;
	char label;
};

struct counter counterA = {
							{	COUNTERA_SEL0_PIN,
								COUNTERA_SEL1_PIN,
								COUNTERA_SEL2_PIN,
								COUNTERA_SEL3_PIN	},
								
							{	COUNTERA0_PIN,
								COUNTERA1_PIN,
								COUNTERA2_PIN,
								COUNTERA3_PIN,
								COUNTERA4_PIN,
								COUNTERA5_PIN,
								COUNTERA6_PIN,
								COUNTERA7_PIN		},
								
								COUNTERA_REG_CLK_PIN,
								
								COUNTERA_CLR_PIN,
								
								'A'
							};
							
struct counter counterB = {
							{	COUNTERB_SEL0_PIN,
								COUNTERB_SEL1_PIN,
								COUNTERB_SEL2_PIN,
								COUNTERB_SEL3_PIN	},
							{	COUNTERB0_PIN,
								COUNTERB1_PIN,
								COUNTERB2_PIN,
								COUNTERB3_PIN,
								COUNTERB4_PIN,
								COUNTERB5_PIN,
								COUNTERB6_PIN,
								COUNTERB7_PIN		},
								
								COUNTERB_REG_CLK_PIN,
								
								COUNTERB_CLR_PIN,
								
								'B'
							};
							
struct counter counterC = {
							{	COUNTERC_SEL0_PIN,
								COUNTERC_SEL1_PIN,
								COUNTERC_SEL2_PIN,
								COUNTERC_SEL3_PIN	},
							{	COUNTERC0_PIN,
								COUNTERC1_PIN,
								COUNTERC2_PIN,
								COUNTERC3_PIN,
								COUNTERC4_PIN,
								COUNTERC5_PIN,
								COUNTERC6_PIN,
								COUNTERC7_PIN		},
								
								COUNTERC_REG_CLK_PIN,
								
								COUNTERC_CLR_PIN,
								
								'C'
							};

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

static void counterio_pulse_pin(uint8_t pin, uint8_t direction, uint32_t wait)
{
	ioport_set_pin_level(pin, direction);
	waitCount(wait);
	ioport_set_pin_level(pin, !direction);
}

uint8_t readCounterByte(uint8_t outputPins[], uint8_t selectPins[], uint8_t byte, char ** p_binaryString)
{
	uint8_t volatile readByte = 0;
	uint8_t volatile bit = 0;
	char bitString[9];
	char tempString[2];
	
	// Set byte pin low and the rest high
	for (uint8_t i = 0; i<4; i++)
	{
		if(i==byte){
			ioport_set_pin_level(selectPins[i], LOW);
		}
		else{
			ioport_set_pin_level(selectPins[i], HIGH);
		}
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

static void printCountValue(struct counter cntr){
	uint8_t readByte = 0;
	uint32_t readCount = 0;
	char * p_bitString;
	char bitString[9] = "";
	p_bitString = &bitString;
	
	// Register counter outputs
	counterio_pulse_pin(cntr.registerClkPin, HIGH, 10000);
	waitCount(10000);
	
	for(uint8_t byteIndex = 0; byteIndex<4; byteIndex++)
	{
		readByte = readCounterByte(cntr.outputPins, cntr.selectPins, byteIndex, p_bitString);
		readCount += ((uint32_t) readByte) << (8*byteIndex);
		printf("[%c%d]%s : %u\r\n", cntr.label, byteIndex, bitString, readByte);
		waitCount(10000);
	}
	
	// Reset HW counter and signal ready FFs
	counterio_pulse_pin(cntr.clearPin, LOW, 10000);
	
	printf("Count: %u\r\n", readCount);
	printf("\r\n");
}

static void CountReady_Handler(uint32_t id, uint32_t mask)
{
	if(ID_PIOA == id){
		if(SIGNAL1A_READY_MASK == mask ){
			if(ioport_get_pin_level(SIGNAL2A_READY_PIN))
			{
				printf("1 & 2 A High\r\n");
				printCountValue(counterA);
			}
		}
		else if( SIGNAL2A_READY_MASK == mask){
			if(ioport_get_pin_level(SIGNAL1A_READY_PIN))
			{
				printf("2 & 1 A High\r\n");
				printCountValue(counterA);
			}
		}
	}

	if(ID_PIOC ==id){
		if(SIGNAL1B_READY_MASK == mask ){
			if(ioport_get_pin_level(SIGNAL2B_READY_PIN))
			{
				printf("1 & 2 B High\r\n");
				printCountValue(counterB);
			}
		}
		else if( SIGNAL2B_READY_MASK == mask){
			if(ioport_get_pin_level(SIGNAL1B_READY_PIN))
			{
				printf("2 & 1 B High\r\n");
				printCountValue(counterB);
			}
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

	printf("--- Console configured\r\n");

	/* Print example information. */
	//puts(STRING_HEADER);

	/* Bring up the ethernet interface & initialize timer0, channel0. */
	//init_ethernet();

	/* Bring up the web server. */
	//httpd_init();
	
	
	
	
	bool countResetPinValue = true;
	
	// Set counter Y read pins
	for(uint8_t i = 0; i < 8 ; i++){
		ioport_set_pin_dir(counterA.outputPins[i], IOPORT_DIR_INPUT);
		ioport_set_pin_dir(counterB.outputPins[i], IOPORT_DIR_INPUT);
		ioport_set_pin_dir(counterC.outputPins[i], IOPORT_DIR_INPUT);
		ioport_set_pin_mode(counterA.outputPins[i], IOPORT_MODE_PULLDOWN);
		ioport_set_pin_mode(counterB.outputPins[i], IOPORT_MODE_PULLDOWN);
		ioport_set_pin_mode(counterC.outputPins[i], IOPORT_MODE_PULLDOWN);
	}
	
	printf("--- Counter data pins configured\r\n");
	
	/////This is for byte shift
	for(uint8_t i = 0; i < 4 ; i++){
		ioport_set_pin_dir(counterA.selectPins[i], IOPORT_DIR_OUTPUT);
		ioport_set_pin_dir(counterB.selectPins[i], IOPORT_DIR_OUTPUT);
		ioport_set_pin_dir(counterC.selectPins[i], IOPORT_DIR_OUTPUT);
	}
	
	printf("--- Counter select pins configured\r\n");
	
	ioport_set_pin_dir(counterA.registerClkPin, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(counterB.registerClkPin, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(counterC.registerClkPin, IOPORT_DIR_OUTPUT);
	
	printf("--- Counter register clock pins configured\r\n");
	
	ioport_set_pin_dir(counterA.clearPin, IOPORT_DIR_OUTPUT);
	ioport_set_pin_mode(counterA.clearPin, IOPORT_MODE_PULLDOWN);
	ioport_set_pin_dir(counterB.clearPin, IOPORT_DIR_OUTPUT);
	ioport_set_pin_mode(counterB.clearPin, IOPORT_MODE_PULLDOWN);
	ioport_set_pin_dir(counterC.clearPin, IOPORT_DIR_OUTPUT);
	ioport_set_pin_mode(counterC.clearPin, IOPORT_MODE_PULLDOWN);
	
	ioport_set_pin_level(counterA.clearPin, countResetPinValue);
	ioport_set_pin_level(counterB.clearPin, countResetPinValue);
	ioport_set_pin_level(counterC.clearPin, countResetPinValue);
	
	printf("--- Counter clear pins configured\r\n");
	//Configure CountRead Pin and Interrupt
	ioport_set_pin_dir(SIGNAL1A_READY_PIN, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(SIGNAL2A_READY_PIN, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(SIGNAL1B_READY_PIN, IOPORT_DIR_INPUT);
	ioport_set_pin_dir(SIGNAL2B_READY_PIN, IOPORT_DIR_INPUT);
	
	printf("--- Set signal ready pin direction\r\n");
	
	ioport_set_pin_mode(SIGNAL1A_READY_PIN, (IOPORT_MODE_PULLUP | IOPORT_MODE_DEBOUNCE) );
	ioport_set_pin_mode(SIGNAL2A_READY_PIN, (IOPORT_MODE_PULLUP | IOPORT_MODE_DEBOUNCE) );
	ioport_set_pin_mode(SIGNAL1B_READY_PIN, (IOPORT_MODE_PULLUP | IOPORT_MODE_DEBOUNCE) );
	ioport_set_pin_mode(SIGNAL2B_READY_PIN, (IOPORT_MODE_PULLUP | IOPORT_MODE_DEBOUNCE) );
	
	printf("--- Set signal ready pin mode\r\n");
	
	ioport_set_pin_sense_mode(SIGNAL1A_READY_PIN, (IOPORT_SENSE_RISING));
	ioport_set_pin_sense_mode(SIGNAL2A_READY_PIN, (IOPORT_SENSE_RISING));
	ioport_set_pin_sense_mode(SIGNAL1B_READY_PIN, (IOPORT_SENSE_RISING));
	ioport_set_pin_sense_mode(SIGNAL2B_READY_PIN, (IOPORT_SENSE_RISING));
	
	printf("--- Set signal ready pin sense\r\n");
	
	pio_handler_set(PIOA, ID_PIOA, (SIGNAL1A_READY_MASK || SIGNAL2A_READY_MASK), (PIO_PULLUP | PIO_DEBOUNCE | PIO_IT_RISE_EDGE), CountReady_Handler);
	pio_handler_set(PIOC, ID_PIOC, (SIGNAL1B_READY_MASK || SIGNAL2B_READY_MASK), (PIO_PULLUP | PIO_DEBOUNCE | PIO_IT_RISE_EDGE), CountReady_Handler);
	
	printf("--- Set signal ready handler\r\n");
	
	NVIC_EnableIRQ((IRQn_Type) ID_PIOA);
	NVIC_EnableIRQ((IRQn_Type) ID_PIOC);
	
	printf("--- Enabled IRQ\r\n");
	
	pio_handler_set_priority(PIOA, (IRQn_Type) ID_PIOA, IRQ_PRIOR_PIO);
	pio_handler_set_priority(PIOC, (IRQn_Type) ID_PIOB, IRQ_PRIOR_PIO);
	
	printf("--- Set handler priority\r\n");
	
	pio_enable_interrupt(PIOA, (SIGNAL1A_READY_MASK || SIGNAL2A_READY_MASK));
	pio_enable_interrupt(PIOC, (SIGNAL1B_READY_MASK || SIGNAL2B_READY_MASK));
	
	printf("--- Enabled interrupt\r\n");
	
	if (SysTick_Config(sysclk_get_cpu_hz() / 1000)) {
		printf("-F- Systick configuration error\r\n\r\n");
		while (1);
	}

	printf("--- Starting Main Loop ---\r\n\r\n");
	
	while (true) {
		
		printCountValue(counterA);
		waitCount(10000);
		printCountValue(counterB);
		waitCount(10000);
		printCountValue(counterC);
		mdelay(1000);
		
		
	}

	/* Program main loop.
	while (1) {
		// Check for input packet and process it.
		ethernet_task();
	}
	*/
}
