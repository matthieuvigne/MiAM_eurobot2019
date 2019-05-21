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
SoftwareSerial BTserial(A2, A3); 

#include <AFMotor.h>
AF_DCMotor motor(1); // create motor #2, 64KHz pwm

void setup() 
{
    // Start communication with bluetooth adapter.
    BTserial.begin(115200); 
    pinMode(A0, OUTPUT);
    pinMode(A1, OUTPUT);
    pinMode(A4, OUTPUT);
    pinMode(A5, OUTPUT);
    digitalWrite(A0, LOW);
    digitalWrite(A1, LOW);
    digitalWrite(A4, LOW);
    digitalWrite(A5, LOW);
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
        motor.setSpeed(255);
        motor.run(FORWARD);
        digitalWrite(A0, HIGH);
        digitalWrite(A1, HIGH);
        digitalWrite(A4, HIGH);
        digitalWrite(A5, HIGH);
      }
    }
}
