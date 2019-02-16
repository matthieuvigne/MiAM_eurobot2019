// Test the USB LCD screen.
#include <MiAMEurobot/drivers/USBLCDDriver.h>
#include <iostream>
#include <unistd.h>

int main(int argc, char **argv)
{
	std::cout << "USB LCD screen test." << std::endl;
	std::cout << "Requierments: LCD screen plugged into any usb port of Raspberry Pi." << std::endl;

	USBLCD lcd;
	if(!lcd.init("/dev/ttyACM0"))
	{
		std::cout << "Could not connect to USB LCD screen" << std::endl;
		return -1;
	}

	std::cout << "Connection sucessful. Testing display and backlight" << std::endl;
	lcd.setText("First line", 0);
	lcd.setText("Second line", 1);
	lcd.setLCDBacklight(255, 127, 127);
	usleep(1000000);

	std::cout << "Testing button state and LEDs. Ctrl+C to terminate." << std::endl;
	lcd.setText("Button state", 0);
	while(true)
	{
		uint8_t state = lcd.getButtonState();
		std::string text = "                ";
		if(state & lcd::LEFT_BUTTON)
			text[4] = '1';
		else
			text[4] = '0';
		if(state & lcd::MIDDLE_BUTTON)
			text[8] = '1';
		else
			text[8] = '0';
		if(state & lcd::RIGHT_BUTTON)
			text[12] = '1';
		else
			text[12] = '0';
		lcd.setText(text, 1);
		lcd.setLED(state);
		usleep(50000);
	}
}
