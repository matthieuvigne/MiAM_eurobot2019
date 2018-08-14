#include "MiAMEurobot/drivers/MaestroServoDriver.h"
#include "MiAMEurobot/drivers/UART-Wrapper.h"

#include <math.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

// Send a command to the device.
// driver: device to send command to.
// commandID: the command ID to use.
// parameters: command parameters.
// length: length of parameters.
// return: result of write.
int sendMaestroCommand(MaestroDriver driver, int commandID, char *parameters, int length)
{
	char message[3 + length];
	message[0] = 0xAA;
	message[1] = driver.deviceID;
	message[2] = commandID;
	for(int i = 0; i < length; i++)
		message[3+i] = parameters[i];

	return write(driver.port, message, 3 + length);
}


gboolean maestro_init(MaestroDriver *driver, gchar *portName, int deviceID)
{
	// Open port
	driver->port = uart_open(portName, B115200);
	driver->deviceID = deviceID;

	if(driver->port == -1)
		return FALSE;

	return TRUE;
}


void maestro_clearError(MaestroDriver driver)
{
	char param = 0;
	sendMaestroCommand(driver, 0x21, &param, 0);
}

void maestro_setPosition(MaestroDriver driver, int servo, double position)
{
	maestro_clearError(driver);
	char parameters[3];
	parameters[0] = servo;
	// Command unit: 0.25us.
	int servoCommand = (int) floor(position * 4);
	if(servoCommand < 0)
		servoCommand = 0;
	if(servoCommand > 2500 * 4)
		servoCommand = 2500 * 4;
	parameters[1] = servoCommand & 0x7F;
	parameters[2] = (servoCommand >> 7) & 0x7F;
	sendMaestroCommand(driver, 0x04, parameters, 3);
}


void maestro_setSpeed(MaestroDriver driver, int servo, int speed)
{
	maestro_clearError(driver);
	char parameters[3];
	parameters[0] = servo;
	// Command unit: 0.25 us/s
	int servoCommand = speed / 25;
	parameters[1] = servoCommand & 0xFF;
	parameters[2] = (servoCommand >> 8) & 0xFF;
	sendMaestroCommand(driver, 0x07, parameters, 3);
}
