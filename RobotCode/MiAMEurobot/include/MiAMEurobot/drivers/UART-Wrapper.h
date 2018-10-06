/// \file drivers/UART-Wrapper.h
/// \brief Helper for using UART serial port.
///
/// \details This helper enables easy opening of a UART port.
///	\note	 All functions in this header should be prefixed with uart_.
#ifndef UART_WRAPPER
	#define UART_WRAPPER

	#include <termios.h>
	#include <glib.h>

	/// \brief Open a serial communication for the given file name, at the given speed.
	///
    /// \param portName Serial port file name ("/dev/ttyOx")
    /// \param speed Communication speed (i.e. B115200, or one of the constants defined in termios.h
    /// \return The open port file descriptor (positive int), or -1 on failure.
	int uart_open(gchar *portName, int speed);
#endif
