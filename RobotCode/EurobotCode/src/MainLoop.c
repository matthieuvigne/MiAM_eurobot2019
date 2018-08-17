/// \file MainLoop.c
/// \brief This file implements the main function, as well as several other features.
///
/// \details We use the glib for event handling. The main loop thus consists in initializing the robot,
///			 starting the auxiliary threads, and then running a GMainLoop with timeout functions.
///			 One timeout function is responsible for checking the infrared sensors - camera wifi communication will
///			 also be done by a callback function.
///
///			 This file includes:
///				- The main() function.
///				- Functions to check on the infrareds sensors.
///				- The heartbeat thread.
///				- The timer thread.

#include "Robot.h"
#include "MotionController.h"
#include "Localisation.h"
#include "Strategy.h"

#include <sys/resource.h>


gboolean enableIR()
{
	robot_disableIROnStartup = FALSE;
	return FALSE;
}


GTimer *timerMain;

// Function that stops the robot.
gboolean stop_robot()
{
	printf("End of match, time: %f\n", g_timer_elapsed(timerMain, NULL));
	// Kill startegy thread to stop the motors.
	pthread_kill(strategyThread, SIGINT);
	g_usleep(500000);
	exit(0);

	return TRUE;
}

// Heartbeat thread.
// This thread simply blinks a cape led, to show that the code is still running
void *heartbeatThread()
{
	while(TRUE)
	{
		g_usleep(200000);
		gpio_digitalWrite(CAPE_LED[0], 1);
		g_usleep(200000);
		gpio_digitalWrite(CAPE_LED[0], 0);
	}
}

gboolean initRobot()
{
	gboolean isInitSuccessful = TRUE;
	gboolean initLCD = lcd_initDefault(&robotLCD, &I2C_2);
	if(!initLCD)
	{
		printf("Could not detect LCD display\n");
		// Can't do much more that turning on the led if we don't have the LCD display.
		gpio_digitalWrite(CAPE_LED[1], 1);
		g_usleep(1000000);
	}
	isInitSuccessful &= initLCD;

	gboolean initIMU = imu_initDefault(&robotIMU, &I2C_1, TRUE);
	if(!initIMU)
	{
		printf("Could not detect IMU\n");
		lcd_setText(robotLCD, "IMU init failed", 0);
		lcd_setBacklight(robotLCD, TRUE, FALSE, FALSE);
		g_usleep(1000000);
	}
	isInitSuccessful &= initIMU;

	gboolean initServo = maestro_initDefault(&robotServo, "/dev/ttyO4");
	if(!initServo)
	{
		printf("Could not talk to servo driver\n");
		lcd_setText(robotLCD, "Servo init failed", 0);
		lcd_setBacklight(robotLCD, TRUE, FALSE, FALSE);
		g_usleep(1000000);
	}
	else
		servo_initPosition();
	isInitSuccessful &= initServo;

	gboolean initMouse = ANDS9800_init(&robotMouseSensor, SPI_0);
	if(!initMouse)
	{
		printf("Could not detect mouse sensor\n");
		lcd_setText(robotLCD, "Mouse init failed", 0);
		lcd_setBacklight(robotLCD, TRUE, FALSE, FALSE);
		g_usleep(1000000);
	}
	isInitSuccessful &= initMouse;

	// Color sensor is unsused in this version of the code.
	//~ gboolean initColor = colorSensor_init(&robotColorSensor, &I2C_1);
	//~ if(!initColor)
	//~ {
		//~ printf("Could not detect color sensor\n");
		//~ lcd_setText(robotLCD, "Color init failed", 0);
		//~ lcd_setBacklight(robotLCD, TRUE, FALSE, FALSE);
		//~ g_usleep(1000000);
	//~ }
	//~ else
	//~ {
		//~ // Setup sensor
		//~ colorSensor_setGain(robotColorSensor, TCS34725_GAIN_16X);
		//~ colorSensor_setIntegrationTime(robotColorSensor, 5);
	//~ }
	//~ isInitSuccessful &= initColor;

	// Try motor init - we are more likely to get a failure here, so print in on the first line.
	gboolean initMotor = motion_initMotors();
	if(!initMotor)
	{
		printf("Could not init motors: are they turned on ?\n");
		lcd_setText(robotLCD, "Motor init failed", 0);
		lcd_setBacklight(robotLCD, TRUE, TRUE, FALSE);
	}
	isInitSuccessful &= initMotor;

	if(isInitSuccessful)
	{
		printf("Init successful\n");
		lcd_setText(robotLCD, "Robot init done", 0);
		lcd_setBacklight(robotLCD, FALSE, TRUE, FALSE);
	}
	g_usleep(1000000);
	return isInitSuccessful;
}

// Wait for match start, returns robot side (TRUE = RIGHT, FALSE = LEFT)
gboolean waitForStart(gboolean isInitDone)
{
	// Wait for staring jack to be plugged in.
	if(gpio_digitalRead(CAPE_DIGITAL[0]) == 1)
	{
		printf("Waiting for jack to be plugged in\n");
		lcd_setText(robotLCD, "Waiting for jack", 1);
		while(gpio_digitalRead(CAPE_DIGITAL[0]) == 1)
			g_usleep(50000);
		printf("Jack plugged in, waiting for match start\n");
		// Prevent unwanted start from switch bounce.
		g_usleep(1000000);
	}
	if(isInitDone)
		lcd_setText(robotLCD, "Ready to start", 0);

	gboolean isRightSide = FALSE;
	gboolean oldSide = FALSE;
	lcd_setText(robotLCD, "     GREEN     >", 1);
	lcd_setBacklight(robotLCD, FALSE, TRUE, FALSE);

	gboolean motorReleased = TRUE;
	motion_releaseMotors();
	while(gpio_digitalRead(CAPE_DIGITAL[0]) == 0)
	{
		if(!isInitDone)
		{
			// Retry motor init.
			isInitDone = motion_initMotors();
			if(isInitDone)
			{
				servo_initPosition();
				printf("Motor init successful\n");
				lcd_setText(robotLCD, "Motor init done", 0);
				g_usleep(1000000);
				lcd_setText(robotLCD, "Ready to start", 0);
			}
		}

		if(lcd_isButtonPressed(robotLCD, LCD_BUTTON_RIGHT))
		{
			oldSide = isRightSide;
			isRightSide = TRUE;
		}

		if(lcd_isButtonPressed(robotLCD, LCD_BUTTON_LEFT))
		{
			oldSide = isRightSide;
			isRightSide = FALSE;
		}

		if(lcd_isButtonPressed(robotLCD, LCD_BUTTON_SELECT))
		{
			motorReleased = !motorReleased;
			if(motorReleased)
				motion_releaseMotors();
			else
				motion_stopMotors();
			while(lcd_isButtonPressed(robotLCD, LCD_BUTTON_SELECT))
				g_usleep(20000);
		}

		if(oldSide != isRightSide)
		{
			if(isRightSide)
			{
				lcd_setText(robotLCD, "<    ORANGE     ", 1);
				lcd_setBacklight(robotLCD, TRUE, TRUE, FALSE);
			}
			else
			{
				lcd_setText(robotLCD, "     GREEN     >", 1);
				lcd_setBacklight(robotLCD, FALSE, TRUE, FALSE);
			}
		}
		g_usleep(50000);
	}
	return isRightSide;
}

int main(int argc, char **argv)
{
	// Init beaglebone.
	BBB_enableCape();
	// Set process priority to 10 lower than standard process.
	setpriority(PRIO_PROCESS, 0, -10);

	// Create a GMainLoop
	GMainLoop* loop = g_main_loop_new(0, 0);
	// Start heartbeat thead.
	g_thread_new("Heartbeat", heartbeatThread, NULL);
	// Init robot hardware.
	gboolean initDone = initRobot();

	// Start the sensor thread (to start IR and logging).
	g_thread_new("SensorUpdate", localisation_startSensorUpdate, NULL);

	// Wait for match to start, redoing motor init if previously failed.
	//~ robot_isOnRightSide = waitForStart(initDone);
	robot_isOnRightSide = FALSE;
	//~ servo_closeClaws();

	// Start match, set timer to 100s.
	timerMain = g_timer_new();
	g_timer_start(timerMain);
	g_timeout_add(100000, stop_robot, NULL);

	// Reset robot position to initial position: right of the zone, back against the wall.
	startingPosition.x = 60 + CHASSIS_BACK;
	startingPosition.y = 60 + CHASSIS_SIDE;
	startingPosition.theta = 0;
	localisation_reset(startingPosition, TRUE, TRUE, TRUE);

	// Turn screen backlight to white, display on second line IR sensor status.
	lcd_setBacklight(robotLCD, TRUE, TRUE, TRUE);
	lcd_clear(robotLCD);
	lcd_setText(robotLCD, "Score:          ", 0);
	//~ lcd_setText(robotLCD, "IR F:  B: ", 1);

	// Launch the strategy thread
	g_thread_new("Strategy", strategy_runMatch, NULL);


	// Timeout functions to check on the infrared sensors.
	robot_disableIROnStartup = TRUE;
	//~ g_timeout_add(1500, enableIR, NULL);
	// Run glib main loop.
    g_main_loop_run(loop);
	return 0;
}
