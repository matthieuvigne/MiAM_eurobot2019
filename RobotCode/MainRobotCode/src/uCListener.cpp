#include "uCListener.h"

#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <stdlib.h>
#include <mutex>
#include <thread>

#include <cmath>

// Length of the message to receive from the uC.
// The message consists of two 0xFF 0xFF bytes, then n bytes.
#define MESSAGE_LENGTH 5

// Encoder resolution: ticks to rad.
const double ENCODER_RESOLUTION = 2 * M_PI / (1024 * 4);

// Global variables: a uCData struct protected by a mutex.
std::mutex uCMutex;
uCData listenerData;

// State in the message:
bool lastWasFF = false; //If the last byte was a 0xFF byte.
int positionInMessage = -1;    // Where we are currently writting in the buffer.
unsigned char buffer[MESSAGE_LENGTH];

// For encoder: last value.
int16_t lastEncoderValue[2] = {0, 0};
// First read: take current value as 0.
bool isFirstRead = true;

void uCListener_listenerThread(std::string const& portName)
{
    // Init data structure.
    listenerData.encoderValues[0] = 0.0;
    listenerData.encoderValues[1] = 0.0;

    // Open communication
    int port = uart_open(portName, B1000000);
    if(port < 0)
    {
        printf("Failed to initialize listener port. Terminating.\n");
        exit(0);
    }
    while(true)
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
                    lastWasFF = false;
                    positionInMessage = -1;

                    // Verify checksum.
                    // Sum of previous bytes must be equal to the checksum.
                    uint8_t checksum = 0;
                    for(int i = 0; i < MESSAGE_LENGTH - 1; i++)
                        checksum += buffer[i];
                    if(checksum != buffer[MESSAGE_LENGTH - 1])
                    {
                        #ifdef DEBUG
                            printf("[uCListener] Invalid checksum, refusing packet\n");
                        #endif
                    }
                    else
                    {
                        // Decode message.
                        // Get current encoder value.
                        for(int i = 0; i < 2; i++)
                        {
                            int16_t encoderValue = (1 << 15) - ((buffer[0 + 2 * i] << 8) + buffer[1 + 2 * i]);

                            if(isFirstRead)
                            {
                                isFirstRead = false || (i == 0);
                                lastEncoderValue[i] = encoderValue;
                            }
                            // Determine the direction of the encoder from the shortest travel possible (i.e. we assume
                            // that between two successive read, the encoder has turned by less than 16000 cournts, i.e. 2 turns).
                            int32_t deltaEncoder = encoderValue - lastEncoderValue[i];
                            while(deltaEncoder > (1 << 14) - 1)
                                deltaEncoder -= 2 * (1 << 14);
                            while(deltaEncoder < -(1 << 14))
                                deltaEncoder += 2 * (1 << 14);
                            lastEncoderValue[i] = encoderValue;

                            double encoderIncrement = deltaEncoder * ENCODER_RESOLUTION;
                            uCMutex.lock();
                            listenerData.encoderValues[i] += encoderIncrement;
                            uCMutex.unlock();
                        }
                    }
                }
            }
        }
    }
}


uCData uCListener_getData()
{
    uCMutex.lock();
    uCData currentData = listenerData;
    uCMutex.unlock();
    return currentData;
}
