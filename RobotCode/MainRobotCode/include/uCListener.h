/// \file uCListener.h
/// \brief Communication with a slave micro-controller to access some sensors.
///
/// \details This uC constantly broadcast the status of the sensor. This file runs a thread that listen to the serial
///          port, update the value as needed and makes it available through a specific data structure.
///
///    \note     All functions in this header should be prefixed with uCListener_.
#ifndef ARDUINOSLAVE_H
     #define ARDUINOSLAVE_H

    ///< Global includes
    #include <MiAMEurobot/MiAMEurobot.h>

    typedef struct {
        double encoderValues[2]; ///<< Current position of the two encoders, in rad.
    }uCData;

    /// \brief The listener thread ; this function never returns and needs to be launched as a separate thread.
    ///
    /// \param[in] portName Name of the serial port to connect to.
    /// \note By default, the port name to which the Arduino connects can change (/dev/ttyACMx). To bind to a fix
    /// path, create a rule in /etc/udev/rules.d/10-local.rules. For instance, for an Arduino Uno board add the following line:
    /// SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0043", SYMLINK+="arduinoUno"
    /// This makes any arduino board match the symlink /dev/arduinoUno
    void uCListener_listenerThread(std::string const& portName);

    /// \brief Get the last value read from the sensors.
    uCData uCListener_getData();

 #endif
