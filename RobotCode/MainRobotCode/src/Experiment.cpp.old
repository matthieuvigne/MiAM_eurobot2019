#include <stdio.h>
#include <unistd.h>
#include <sys/socket.h>
#include <fcntl.h>

#include <MiAMEurobot/MiAMEurobot.h>
#include "Experiment.h"

// Include local headers and definition of str2ba to prevent having to link to bluez library.
#include "bluetooth/bluetooth.h"
#include "bluetooth/rfcomm.h"

int bachk(const char *str)
{
	if (!str)
		return -1;

	if (strlen(str) != 17)
		return -1;

	while (*str) {
		if (!isxdigit(*str++))
			return -1;

		if (!isxdigit(*str++))
			return -1;

		if (*str == 0)
			break;

		if (*str++ != ':')
			return -1;
	}

	return 0;
}

int str2ba(const char *str, bdaddr_t *ba)
{
	int i;

	if (bachk(str) < 0) {
		memset(ba, 0, sizeof(*ba));
		return -1;
	}

	for (i = 5; i >= 0; i--, str += 3)
		ba->b[i] = strtol(str, NULL, 16);

	return 0;
}

Experiment::Experiment():
	port_(-1)
{
}

bool Experiment::startConnection()
{
	// Hard-coded destination.
	char dest[18] = "98:D3:71:F9:87:3E";
	// Create socket struct.
	struct sockaddr_rc addr = { 0 };
	addr.rc_family = AF_BLUETOOTH;
	addr.rc_channel = (uint8_t) 1;
	str2ba( dest, &addr.rc_bdaddr );

	// Create socket and try to connect with timeout.
	port_ = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

	int flags = fcntl(port_, F_GETFL, 0);
	fcntl(port_, F_SETFL, flags | O_NONBLOCK);

	// Try to connect with 2s timeout
	int status = connect(port_, (struct sockaddr *)&addr, sizeof(addr));
	if (status < 0)
	{
		if (errno == EINPROGRESS)
			{
			struct timeval timeout;
			timeout.tv_sec = 5;
			timeout.tv_usec = 0;
			fd_set set;
			FD_ZERO(&set);
			FD_SET(port_, &set);

			if(select(port_ + 1, NULL, &set, NULL, &timeout) > 0)
			{
				// Check that there was no connection error.
				socklen_t lon = sizeof(int);
				int valopt = sizeof(int);
				getsockopt(port_, SOL_SOCKET, SO_ERROR, (void*)(&valopt), &lon);
				if (valopt)
					port_ = -1;
			}
			else
				port_ = -1;
		}
		else
		{
			port_ = -1;
		}
	}

	// Set blocking mode again.
	flags = fcntl(port_, F_GETFL, 0);
	fcntl(port_, F_SETFL, flags & ~O_NONBLOCK);

	// Give some time for the connection to setup.
	usleep(500000);
	// Flush serial buffer.
	isConnected();

	// Test connection with arduino.
	return isConnected();
}


bool Experiment::isConnected()
{
	if(port_ < 0)
		return false;
	// Send 'a', look if 'M' is the reply.
	if(write(port_, "a", 1) < 0)
		return false;

	// Check that return value is 'M', with 10ms timeout.
	// Read up to 10 bytes to flush buffer if needed.
	unsigned char reply[10];
	if(read_timeout(port_, reply, 10, 50) < 1)
		return false;
	return reply[0] == 'M';
}

bool Experiment::start()
{

	if(port_ < 0)
		return false;
	// Send 's', look if 'S' is the reply.
	if(write(port_, "s", 1) < 0)
		return false;

	// Check that return value is 'S', with 10ms timeout.
	// Read up to 10 bytes to flush buffer if needed.
	unsigned char reply[10];
	if(read_timeout(port_, reply, 10, 50) < 1)
		return false;
	return reply[0] == 'S';
}

