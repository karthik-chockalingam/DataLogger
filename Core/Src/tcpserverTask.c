/*
 * tcpserverTask.c
 *
 *  Created on: Sep 2, 2023
 *      Author: karthik
 */

#include "tcpserverTask.h"

static struct netconn *srvr_conn;
static struct netbuf *buf;
typedef struct
{
	uint8_t setAnalog;
	uint8_t setDigital;
	uint8_t setSamplerate;
	uint8_t setOperation;
}logConfig;

extern struct netconn *client_conn;
extern QueueHandle_t configQueueHandle;

static void tcpTask(void *arg)
{
	err_t err, accept_err;
	logConfig clientConfig;
	BaseType_t status;

	char user_config[4];
	/* Create a new connection identifier. */
	srvr_conn = netconn_new(NETCONN_TCP);
	printf("Entering into tcpTask...\n");

	if (srvr_conn!=NULL)
	{
		/* Bind connection to the port number 10022. */
		err = netconn_bind(srvr_conn, IP_ADDR_ANY, 10022);

		if (err == ERR_OK)
		{
			/* Tell connection to go into listening mode. */
			netconn_listen(srvr_conn);

			while (1)
			{
				/* Grab new connection. */
				accept_err = netconn_accept(srvr_conn, &client_conn);

				/* Process the new connection. */
				if (accept_err == ERR_OK)
				{

					/* receive the data from the client */
					while (netconn_recv(client_conn, &buf) == ERR_OK)
					{
						memset (user_config, '\0', 4);  // clear the buffer
						strncpy (user_config, buf->p->payload, 4);   // get the message from the client

						clientConfig.setAnalog = (uint8_t)user_config[0];
						clientConfig.setDigital = (uint8_t)user_config[1];
						clientConfig.setSamplerate = (uint8_t)user_config[2];
						clientConfig.setOperation = (uint8_t)user_config[3];


						status = xQueueSend(configQueueHandle, (void*) &clientConfig, 0);
						if(status == pdPASS)
						{
							printf("Queuing config ...\n");
						}
						else
						{
							printf("Queuing config error...\n");
						}
						//netconn_write(newconn, smsg, len, NETCONN_COPY);

						netbuf_delete(buf);
					}

					/* Close connection and discard connection identifier. */
					netconn_close(client_conn);
					netconn_delete(client_conn);
				}
			}
		}
		else
		{
			netconn_delete(srvr_conn);
		}
	}
}


void tcpserver_init(void)
{
  sys_thread_new("tcp-task", tcpTask, NULL, 1024, osPriorityNormal);
}
