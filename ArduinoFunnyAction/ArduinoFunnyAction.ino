// Code for Arduino board on the experiment.
// This board is linked to a bluetooth module to communicate with the main robot.
// The module was configured as follow:
//  - name: MiAMxp
//  - pin: 1987
//  - hdw address: 98:D3:71:F9:87:3E
//  - baud rate: 115200
//
// To connect: bluetoothctl // scan on // pair 98:D3:71:F9:87:3E // enter pin.
// For more info on linux/bluetooth: https://blog.dbrgn.ch/2017/8/22/hc06-bluetooth-terminal-on-linux/

#include <SoftwareSerial.h>
SoftwareSerial BTserial(2, 3); 

void setup() 
{
    // Start communication with bluetooth adapter.
    BTserial.begin(115200);  
}
 
void loop()
{
 
    // Keep reading from HC-06 and send to Arduino Serial Monitor
    if (BTserial.available())
    {  
      char c = BTserial.read();

      if (c == 'a')
      {
        // Send acknoledge.
        BTserial.write("M");
      }
      if (c == 's')
      {
        // Start experiment, send acknoledge.
        BTserial.write("S");
      }
    }
}
