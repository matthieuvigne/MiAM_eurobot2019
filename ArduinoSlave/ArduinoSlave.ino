
// Code for the slave Arduino uC, used in the robot to access low-level sensor.
// This code reads the following sensors, sending the result over uart:
//  - wheel rotary encoder.

// Left encoder : pin number in port D.
#define pinA 2
#define pinB 3

bool oldB = 0;

// Encoder value - stored over 15 bits in a 16-bits number.
int encoderCount = 0;

void encoderInterrupt()
{
  // Get current status.
  bool currentA = (PIND & (1 << pinA)) >> pinA;

  // The direction of the encoder is given by currentA xor oldB
  encoderCount += (oldB ^ currentA ? 1 : -1);

  // Keep the number within 15 bits.
  if(encoderCount > (1 << 14) - 1)
    encoderCount = - 1 << 14;
  else if(encoderCount < - (1 << 14))
    encoderCount = (1 << 14) - 1;
  
  //encoderCount ++;
  oldB = (PIND & (1 << pinB)) >> pinB;
}


void setup() 
{
  // Enable serial port.
  Serial.begin(115200);
  // Interrupt for first encoder
  pinMode(pinA, INPUT);   
  pinMode(pinB, INPUT);     
  attachInterrupt(digitalPinToInterrupt(pinA), encoderInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinB), encoderInterrupt, CHANGE);
  
}

void loop() {
  Serial.write(0xFF);
  Serial.write(0xFF);
  // Take encoderCount, use its 2s complement with 2^15
  int encoderComp = (1 << 15) - encoderCount;
  Serial.write((encoderComp >> 8) & 0xFF);
  Serial.write(encoderComp & 0xFF);
  
  delayMicroseconds(100);
}
