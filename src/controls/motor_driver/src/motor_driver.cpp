#include <string>
#include <sstream>
#include "ros/ros.h"
#include "serial/serial.h"
#include "motor_driver/motor_driver.h"

using namespace motor_abs;

/**
 * format
 * Formats string into format to send across serial
 *
 * @param motor Which motor we're setting the speed of
 * @param speed What to set the speed to
 * @return Formatted string with two speeds xor'd for crcish thing
 */
std::string motor_driver::format(int16_t motor_1_speed, int16_t motor_2_speed)
{
	std::ostringstream stm;
	stm << motor_1_speed << "," << motor_2_speed << "," << (motor_1_speed^motor_2_speed) <<"*";
	return stm.str();
}

/**
 * motor_driver
 * Constructor for motor_driver. Initializes serial connection
 * 
 * @param port Serial port arduino is connected by
 * @param baud Baud rate for serial connection. Should be 115200 for arduino
 */
motor_driver::motor_driver(std::string port, uint32_t baud)
{
	connection = new serial::Serial(port, baud, serial::Timeout::simpleTimeout(1000));
}

/**
 * check_connection
 * Checks if serial connection is open
 * 
 * @return Returns if connection is open or not
 */
bool motor_driver::check_connection()
{
	return connection->isOpen(); 
}

/**
 * set_speed
 * Sets the speed of a motor
 *
 * @param motor Which motor we're setting the speed of
 * @param speed What to set the speed to
 */
void motor_driver::set_speed(int16_t motor_1_speed, int16_t motor_2_speed)
{
	if (motor_1_speed >= MAX_BACKWARD && motor_1_speed <= MAX_FORWARD && motor_2_speed >= MAX_BACKWARD && motor_2_speed <= MAX_FORWARD)
	{
		std::string message = format(motor_1_speed, motor_2_speed);
		std::string result = "";
		while (result != "true")
		{
			connection->write(message);
			result = connection->readline();
			result = result.substr(0, result.size()-2);
		}
	}
}

/**
 * ~motor_driver
 * Destructor for motor_driver. Closes serial connection.
 */
motor_driver::~motor_driver()
{
	connection->close();
	delete &connection;
}