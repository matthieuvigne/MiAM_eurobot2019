
// Code for the slave Arduino uC, used in the robot to access low-level sensor.
// This code reads the following sensors, sending the result over uart:
//  - wheel rotary encoder.

// Read a single bit off a register
#define READ_BIT(REG, BIT) (REG & (1 << BIT)) >> BIT;
#define ENCODER_TO_REGISTER(ENCODER) (ENCODER ? PINB : PIND)

// Encoder pin number: first encoder is in port D, second encoder is in port B.
const char encoderPinA[2] = {7,0};
const char encoderPinB[2] = {6,1};

// Encoder old B value.
bool oldB[2] = {0, 0};

// Current encoder position.
int encoderCount[2] = {0, 0};


// Interrupt function, called when an encoder interrupt is triggered.
void handleEncoder(char encoderNumber)
{
  // Get current status.
  bool currentA = READ_BIT(ENCODER_TO_REGISTER(encoderNumber), encoderPinA[encoderNumber]);
  
  // The direction of the encoder is given by currentA xor oldB
  encoderCount[encoderNumber] += (oldB[encoderNumber] ^ currentA ? 1 : -1);
  oldB[encoderNumber] =  READ_BIT(ENCODER_TO_REGISTER(encoderNumber), encoderPinB[encoderNumber]);
  
  // Keep the number within 15 bits.
  if(encoderCount[encoderNumber] > (1 << 14) - 1)
    encoderCount[encoderNumber] = - 1 << 14;
  else if(encoderCount[encoderNumber] < - (1 << 14))
    encoderCount[encoderNumber] = (1 << 14) - 1;
  
}

// Interrupt for first encoder.
ISR(PCINT2_vect)
{
  handleEncoder(0);
}

// Interrupt for second encoder.
ISR(PCINT0_vect)
{
  handleEncoder(1);
}


void setup() 
{
  // Enable serial port.
  Serial.begin(115200);
  // Set encoder pins as input.
  DDRD &= ~(1 << encoderPinA[0]);
  DDRD &= ~(1 << encoderPinB[0]);
  DDRB &= ~(1 << encoderPinA[1]);
  DDRB &= ~(1 << encoderPinB[1]);
  // Setup interrupt for port B and D.
  PCMSK2 = 0x00;
  PCMSK2 |= 1 << encoderPinA[0];
  PCMSK2 |= 1 << encoderPinB[0];
  PCMSK0 = 0x00;
  PCMSK0 |= 1 << encoderPinA[1];
  PCMSK0 |= 1 << encoderPinB[1];
  // Enalbe interrupt for port B and D.
  PCICR = 0b101;
  
}

void loop() {
 Serial.write(0xFF);
 Serial.write(0xFF);
 for(int i = 0; i < 2; i++)
 {
  // Take encoderCount, use its 2s complement with 2^15
  int encoderComp = (1 << 15) - encoderCount[i];
  Serial.write((encoderComp >> 8) & 0xFF);
  Serial.write(encoderComp & 0xFF);
 }
 delayMicroseconds(100);
}
