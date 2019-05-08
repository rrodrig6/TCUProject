/**
 * \file
 *
 * \brief Httpd server.
 *
 * Copyright (c) 2013-2018 Microchip Technology Inc. and its subsidiaries.
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

#include <string.h>
#include "lwip/debug.h"
#include "lwip/stats.h"
#include "lwip/tcp.h"
#include "lwipopts.h"
#include "httpd.h"
#include "fs.h"

#define SEND_BUFFER_MAX 128
#define RECEIVE_BUFFER_MAX 64

uint8_t sendBufferIndex;
uint8_t sendBufferLength;
char sendBuffer[SEND_BUFFER_MAX];

uint8_t receiveBufferIndex;
uint8_t receiveBufferLength;
char receiveBuffer[RECEIVE_BUFFER_MAX];


struct http_state {
  char *file;
  u32_t left;
  u8_t retries;
};

/**
 * \brief Callback called on connection error.
 *
 * \param arg Pointer to structure representing the HTTP state.
 * \param err Error code.
 */
static void http_conn_err(void *arg, err_t err)
{
	struct http_state *hs;

	LWIP_UNUSED_ARG(err);

	hs = arg;
	mem_free(hs);
}

/**
 * \brief Close HTTP connection.
 *
 * \param pcb Pointer to a TCP connection structure.
 * \param hs Pointer to structure representing the HTTP state.
 */
static void http_close_conn(struct tcp_pcb *pcb, struct http_state *hs)
{
	tcp_arg(pcb, NULL);
	tcp_sent(pcb, NULL);
	tcp_recv(pcb, NULL);
	mem_free(hs);
	tcp_close(pcb);
}

/**
 * \brief Send HTTP data.
 *
 * \param pcb Pointer to a TCP connection structure.
 * \param hs Pointer to structure representing the HTTP state.
 */
static void http_send_data(struct tcp_pcb *pcb, struct http_state *hs)
{
	err_t err;
	u32_t len;

	/* We cannot send more data than space available in the send buffer. */
	if (tcp_sndbuf(pcb) < hs->left) {
		len = tcp_sndbuf(pcb);
	} else {
		len = hs->left;
	}

	do {
		/* Use copy flag to avoid using flash as a DMA source (forbidden). */
		err = tcp_write(pcb, hs->file, len, TCP_WRITE_FLAG_COPY);
		if (err == ERR_MEM) {
			len /= 2;
		}
	} while (err == ERR_MEM && len > 1);

	if (err == ERR_OK) {
		hs->file += len;
		hs->left -= len;
	}
}

/**
 * \brief Poll for HTTP data.
 *
 * \param arg Pointer to structure representing the HTTP state.
 * \param pcb Pointer to a TCP connection structure.
 *
 * \return ERR_OK on success, ERR_ABRT otherwise.
 */
static err_t http_poll(void *arg, struct tcp_pcb *pcb)
{
	struct http_state *hs;

	hs = arg;

	if (hs == NULL) {
		tcp_abort(pcb);
		return ERR_ABRT;
	} else {
		if (hs->file == 0) {
			tcp_abort(pcb);
			return ERR_ABRT;
		}

		++hs->retries;
		if (hs->retries == 4) {
			tcp_abort(pcb);
			return ERR_ABRT;
		}

		http_send_data(pcb, hs);
	}

	return ERR_OK;
}

/**
 * \brief Callback to handle data transfer.
 *
 * \param arg Pointer to structure representing the HTTP state.
 * \param pcb Pointer to a TCP connection structure.
 * \param len Unused.
 *
 * \return ERR_OK on success, ERR_ABRT otherwise.
 */
static err_t http_sent(void *arg, struct tcp_pcb *pcb, u16_t len)
{
	struct http_state *hs;

	LWIP_UNUSED_ARG(len);

	hs = arg;

	hs->retries = 0;

	if (hs->left > 0) {
		http_send_data(pcb, hs);
	} else {
		http_close_conn(pcb, hs);
	}

	return ERR_OK;
}

/**
 * \brief Core HTTP server receive function. Handle the request and process it.
 *
 * \param arg Pointer to structure representing the HTTP state.
 * \param pcb Pointer to a TCP connection structure.
 * \param p Incoming request.
 * \param err Connection status.
 *
 * \return ERR_OK.
 */
static err_t http_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
{
	int i;
	char *data;
	struct fs_file file;
	struct http_state *hs;

	hs = arg;

	if (err == ERR_OK && p != NULL) {
		/* Inform TCP that we have taken the data. */
		tcp_recved(pcb, p->tot_len);

		if (hs->file == NULL) {
			data = p->payload;

			if (strncmp(data, "GET ", 4) == 0) {
				for (i = 0; i < 40; i++) {
					if (((char *)data + 4)[i] == ' ' ||
							((char *)data + 4)[i] == '\r' ||
							((char *)data + 4)[i] == '\n') {
						((char *)data + 4)[i] = 0;
					}
				}

				if (*(char *)(data + 4) == '/' &&
						*(char *)(data + 5) == 0) {
					fs_open("/index.html", &file);
				}
				else if (!fs_open((char *)data + 4, &file)) {
					fs_open("/404.html", &file);
				}

				hs->file = file.data;
				hs->left = file.len;
				/* printf("data %p len %ld\n", hs->file, hs->left);*/

				pbuf_free(p);
				http_send_data(pcb, hs);

				/* Tell TCP that we wish be to informed of data that has been
				successfully sent by a call to the http_sent() function. */
				tcp_sent(pcb, http_sent);
			} else {
				pbuf_free(p);
				http_close_conn(pcb, hs);
			}
		} else {
			pbuf_free(p);
		}
	}

	if (err == ERR_OK && p == NULL) {
		http_close_conn(pcb, hs);
	}

	return ERR_OK;
}

/**
 * \brief Accept incoming HTTP connection requests.
 *
 * \param arg Pointer to structure representing the HTTP state.
 * \param pcb Pointer to a TCP connection structure.
 * \param err Connection status.
 *
 * \return ERR_OK on success.
 */
static err_t http_accept(void *arg, struct tcp_pcb *pcb, err_t err)
{
	struct http_state *hs;

	LWIP_UNUSED_ARG(arg);
	LWIP_UNUSED_ARG(err);

	tcp_setprio(pcb, TCP_PRIO_MIN);

	/* Allocate memory for the structure that holds the state of the
	connection. */
	hs = (struct http_state *)mem_malloc(sizeof(struct http_state));

	if (hs == NULL) {
		//printf("http_accept: Out of memory\n");
		return ERR_MEM;
	}

	/* Initialize the structure. */
	hs->file = NULL;
	hs->left = 0;
	hs->retries = 0;

	/* Tell TCP that this is the structure we wish to be passed for our
	callbacks. */
	tcp_arg(pcb, hs);

	/* Tell TCP that we wish to be informed of incoming data by a call
	to the http_recv() function. */
	tcp_recv(pcb, http_recv);

	tcp_err(pcb, http_conn_err);

	tcp_poll(pcb, http_poll, 4);
	return ERR_OK;
}

/**
 * \brief HTTP server init.
 */
void httpd_init(void)
{
	struct tcp_pcb *pcb;

	pcb = tcp_new();
	tcp_bind(pcb, IP_ADDR_ANY, 80);
	pcb = tcp_listen(pcb);
	tcp_accept(pcb, http_accept);
}


// ECHO SERVER IMPLEMENTATION
// from https://github.com/dreamcat4/lwip/blob/master/contrib/apps/tcpecho_raw/echo.c

static struct tcp_pcb *echo_pcb;

enum echo_states
{
	ES_NONE = 0,
	ES_ACCEPTED,
	ES_RECEIVED,
	ES_CLOSING
};

struct echo_state
{
	u8_t state;
	u8_t retries;
	struct tcp_pcb *pcb;
	/* pbuf (chain) to recycle */
	struct pbuf *p;
};

err_t echo_accept(void *arg, struct tcp_pcb *newpcb, err_t err);
err_t echo_recv( void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
void echo_error( void *arg, err_t err);
err_t echo_poll(void *arg, struct tcp_pcb *tpcb);
err_t echo_sent( void *arg, struct tcp_pcb *tpcb, u16_t len);
void echo_send( struct tcp_pcb *tpcb, struct echo_state *es);
void echo_close( struct tcp_pcb *tpcb, struct echo_state *es);

void echo_init(void)
{
	echo_pcb = tcp_new();
	if(echo_pcb != NULL)
	{
		err_t err;
		err = tcp_bind(echo_pcb, IP_ADDR_ANY, 7);
		if(err==ERR_OK)
		{
			echo_pcb = tcp_listen(echo_pcb);
			tcp_accept(echo_pcb, echo_accept);
		}
		else
		{
			printf("!!! TCP BIND ERROR !!!\r\n");
		}
	}
	else
	{
		printf("!!! TCP NEW ERROR !!!\r\n");	
	}
}

err_t echo_accept(void *arg, struct tcp_pcb *newpcb, err_t err)
{
	err_t ret_err;
	struct echo_state *es;
	
	LWIP_UNUSED_ARG(arg);
	LWIP_UNUSED_ARG(err);
	
	/* commonly observed practice to call tcp_setprio(), why? */
	tcp_setprio(newpcb, TCP_PRIO_MIN);
	
	es = (struct echo_state *)mem_malloc(sizeof(struct echo_state));
	if(es !=NULL)
	{
		es->state = ES_ACCEPTED;
		printf(">>ES_ACCEPTED\r\n");
		es->pcb = newpcb;
		es->retries = 0;
		es->p = NULL;
		/* pass newly allocated es to our callback */
		tcp_arg(newpcb, es);
		tcp_recv(newpcb, echo_recv);
		tcp_err(newpcb, echo_error);
		tcp_poll(newpcb, echo_poll, 0);
		ret_err = ERR_OK;
	}
	else
	{
		ret_err = ERR_MEM;
	}
	return ret_err;
}

err_t echo_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
	struct echo_state *es;
	err_t ret_err;
	
	LWIP_ASSERT("arg != NULL", arg != NULL);
	es = (struct echo_state *)arg;
	if (p == NULL)
	{
		/* remote host closed connection */
		es->state = ES_CLOSING;
		printf(">>ES_CLOSING\r\n");
		if(es->p==NULL)
		{
			/*we're done sending, close it*/
			echo_close(tpcb,es);
		}
		else
		{
			/* we're not done yet */
			tcp_sent(tpcb, echo_sent);
			echo_send(tpcb, es);
		}
		ret_err = ERR_OK;
	}
	else if(err != ERR_OK)
	{
		/* cleanup, for unknown reason */
		if(p!=NULL)
		{
			es->p = NULL;
			pbuf_free(p);
		}
		ret_err = err;
	}
	else if(es->state == ES_ACCEPTED)
	{
		/* first data chunk in p->payload */
		es->state = ES_RECEIVED;
		printf(">>ES_RECEIVED\r\n");
		/* store reference to incoming pbuf (chain) */
		//es->p = p;
		
		// Save the read payload
		pbuf_copy_partial(p, receiveBuffer + receiveBufferIndex, p->tot_len, 0);
		receiveBufferIndex += p->tot_len;
		pbuf_free(p);
		/* install send completion notifier */
		//tcp_sent(tpcb, echo_sent);
		//echo_send(tpcb, es);
		ret_err = ERR_OK;
	}
	else if(es->state == ES_RECEIVED)
	{
		/*read some more data */
		if(es->p == NULL)
		{
			//es->p = p;
			
			// Save the read payload
			pbuf_copy_partial(p, receiveBuffer + receiveBufferIndex, p->tot_len, 0);
			receiveBufferIndex += p->tot_len;
			pbuf_free(p);
			//tcp_sent(tpcb, echo_sent);
			//echo_send(tpcb, es);
		}
		else
		{
			// Save the read payload
			pbuf_copy_partial(p, receiveBuffer + receiveBufferIndex, p->tot_len, 0);
			receiveBufferIndex += p->tot_len;
			pbuf_free(p);
			//struct pbuf *ptr;
			
			/* chain pbufs to the end of what we received previously */
			//ptr = es->p;
			//pbuf_chain(ptr, p);
		}
		ret_err = ERR_OK;
	}
	else if(es->state == ES_CLOSING)
	{
		/* odd case, remote side closing twice, trash data */
		tcp_recved(tpcb, p->tot_len);
		es->p = NULL;
		pbuf_free(p);
		ret_err = ERR_OK;
	}
	else
	{
		/* unknown es->state, trash data */
		tcp_recved(tpcb, p->tot_len);
		es->p = NULL;
		pbuf_free(p);
		ret_err = ERR_OK;
	}
	return ret_err;
}

void echo_error(void *arg, err_t err)
{
	struct echo_state *es;
	
	LWIP_UNUSED_ARG(err);
	
	es = (struct echo_state *)arg;
	if(es!=NULL)
	{
		mem_free(es);
	}
}

err_t echo_poll(void *arg, struct tcp_pcb *tpcb)
{
	err_t ret_err;
	struct echo_state *es;
	
	if(sendBufferIndex>0)
	{
		echo_send(tpcb, es);
	}
	
	/*
	es = (struct echo_state *)arg;
	if(es!=NULL)
	{
		if(es->p!=NULL)
		{
			// there is a remaining pbuf (chain)
			tcp_sent(tpcb, echo_sent);
			echo_send(tpcb, es);
		}
		else
		{
			// no remaining pbuf (chain)
			if(es->state == ES_CLOSING)
			{
				echo_close(tpcb, es);
			}
		}
		ret_err = ERR_OK;
	}
	else
	{
		// nothing to be done
		tcp_abort(tpcb);
		ret_err = ERR_ABRT;
	}
	*/
	return ret_err;
}

err_t echo_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
	struct echo_state *es;
	
	LWIP_UNUSED_ARG(len);
	
	es = (struct echo_state *)arg;
	es->retries = 0;
	
	if(es->p !=NULL)
	{
		/* still go pbufs to send */
		tcp_sent(tpcb, echo_sent);
		echo_send(tpcb, es);
	}
	else
	{
		/* no more pbufs to send */
		if(es->state == ES_CLOSING)
		{
			echo_close(tpcb, es);
		}
	}
	return ERR_OK;
}

void echo_send(struct tcp_pcb *tpcb, struct echo_state *es)
{
	printf(">>>ECHO SEND %u BYTES\r\n", sendBufferIndex);
	struct pbuf *ptr;
	err_t wr_err = ERR_OK;
	
	while ((wr_err==ERR_OK)&&
			(sendBufferIndex!=0) &&
			(sendBufferIndex <= tcp_sndbuf(tpcb)))
	{
		//ptr = es->p;
		
		// enqueue data for transmission 
		wr_err = tcp_write(tpcb, sendBuffer, sendBufferIndex, 1);
		sendBufferIndex = 0;
		if(wr_err ==ERR_OK)
		{
			/*
			u16_t plen;
			u8_t freed;
			plen = ptr->len;
			// continue with next pbuf in chain (if any) 
			es->p = ptr->next;
			if(es->p !=NULL)
			{
				// new reference! 
				pbuf_ref(es->p);
			}
			// chop first pbuf from chain
			do{
				//try hard to free pbuf
				freed = pbuf_free(ptr);
			}
			while(freed == 0);
			// we can read more data now
			tcp_recved(tpcb, plen);
			*/
		}
		else if(wr_err==ERR_MEM)
		{
			// we are low on memory, try later/harder, defer to poll */
			es->p = ptr;
		}
		else
		{
			// other problem ? //
			printf("!!! ECHO SEND ERROR !!!\r\n");
		}
	}
}

void echo_close(struct tcp_pcb *tpcb, struct echo_state *es)
{
	tcp_arg(tpcb, NULL);
	tcp_sent(tpcb, NULL);
	tcp_recv(tpcb, NULL);
	tcp_err(tpcb, NULL);
	tcp_poll(tpcb, NULL, 0);
	
	if(es != NULL)
	{
		mem_free(es);
	}
	tcp_close(tpcb);
}