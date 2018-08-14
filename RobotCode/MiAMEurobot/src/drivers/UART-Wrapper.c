#include "MiAMEurobot/drivers/UART-Wrapper.h"

#include <fcntl.h>
#include <unistd.h>
// Open a uart port.
int uart_open(gchar *portName, int speed)
{
	// Open in read-write mode.
	int port = open(portName, O_RDWR);
	if(port == -1)
		return port;

	// Setup UART port.
	struct termios options;
	tcgetattr(port, &options);

	//Set communication speed and options
	//8 bit, 1 stop, no parity
	cfsetispeed(&options, speed);
	cfsetospeed(&options, speed);
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	options.c_cflag &= ~CSIZE;
	options.c_cflag |= CS8;
	options.c_lflag &= ~ECHO;

	// Set input and output to raw mode.
	cfmakeraw(&options);

	tcsetattr(port, TCSAFLUSH, &options);
	g_usleep(100000);
	tcflush(port, TCIOFLUSH);

	return port;
}
