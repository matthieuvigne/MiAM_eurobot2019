#include "uCListener.h"

#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <stdlib.h>

// Length of the message to receive from the uC.
// The message consists of two 0xFF 0xFF bytes, then n bytes.
#define MESSAGE_LENGTH 2

// Encoder resolution: ticks to rad.
const double ENCODER_RESOLUTION = 2 * G_PI / (2048 * 4);

// Global variables: a uCData struct protected by a mutex.
GMutex uCMutex;
uCData listenerData;

// State in the message:
gboolean lastWasFF = 0; //If the last byte was a 0xFF byte.
int positionInMessage = -1;	// Where we are currently writting in the buffer.
unsigned char buffer[MESSAGE_LENGTH];

// For encoder: last value.
int16_t lastEncoderValue = 0;
// First read: take current value as 0.
gboolean isFirstRead = TRUE;

void *listenerThread(void *portName)
{
	// Init data structure.
	listenerData.encoderValue = 0.0;

	// Open communication
	int port = uart_open((gchar*)portName, B115200);
	if(port < 0)
	{
		printf("Failed to initialize listener port. Listener thread exiting.\n");
		return 0;
	}
	while(TRUE)
	{
		// Read a single byte from the serial port.
		unsigned char lastData = 0;
		int result = read(port, &lastData, 1);
		if(result < 0)
			printf("Error reading from microcontroller: %d\n", errno);
		else
		{
			// If it's a 0xFF, and if the previous byte was also a 0xFF, a new message starts.
			if(lastData == 0xFF && lastWasFF)
				positionInMessage = 0;
			else
			{
				lastWasFF = lastData == 0xFF;

				// If we are currently reading a message, add it to the buffer.
				if(positionInMessage > -1)
				{
					buffer[positionInMessage] = lastData;
					positionInMessage ++;
				}
				// If the end of a message was reached, decode it.
				if(positionInMessage == MESSAGE_LENGTH)
				{
					// Reset status.
					lastWasFF = FALSE;
					positionInMessage = -1;

					// Decode message.

					// Get current encoder value.
					int16_t encoderValue = (1 << 15) - ((buffer[0] << 8) + buffer[1]);

					if(isFirstRead)
					{
						isFirstRead = FALSE;
						lastEncoderValue = encoderValue;
					}
					// Determine the direction of the encoder from the shortest travel possible (i.e. we assume
					// that between two successive read, the encoder has turned by less than 16000 cournts, i.e. 2 turns).
					int32_t deltaEncoder = encoderValue - lastEncoderValue;
					while(deltaEncoder > (1 << 14) - 1)
						deltaEncoder -= 2 * (1 << 14);
					while(deltaEncoder < -(1 << 14))
						deltaEncoder += 2 * (1 << 14);
					lastEncoderValue = encoderValue;

					double encoderIncrement = deltaEncoder * ENCODER_RESOLUTION;

					g_mutex_lock(&uCMutex);
					listenerData.encoderValue += encoderIncrement;
					g_mutex_unlock(&uCMutex);
				}
			}
		}
	}
	return 0;
}


void uCListener_startListener(gchar *portName)
{
	g_thread_new("uCListener", listenerThread, portName);
}

uCData uCListener_getData()
{
	g_mutex_lock(&uCMutex);
	uCData currentData = listenerData;
	g_mutex_unlock(&uCMutex);
	return currentData;
}

