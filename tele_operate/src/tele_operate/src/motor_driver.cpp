#include <string>
#include <sstream>
#include <stdint.h>
#include "ros/ros.h"
#include "serial/serial.h"
#include "motor_driver.h"

using namespace motor_abs;

/**
 * Class to send motor commands to the arduino
 */
class motor_driver
{
private:
	/**
	 * Serial connection to arduino
	 */
	serial::Serial connection;
	
	/**
	 * format
	 * Formats string into format to send across serial
	 *
	 * @param motor Which motor we're setting the speed of
	 * @param speed What to set the speed to
	 * @return Formatted string
	 */
	std::string format(int16_t motor, int16_t speed)
	{
		std::ostringstream stm;
		stm << motor << "," << speed << "*";
		return stm.str();
	}

public:
	/**
	 * motor_driver
	 * Constructor for motor_driver. Initializes serial connection
	 * 
	 * @param port Serial port arduino is connected by
	 * @param baud Baud rate for serial connection. Should be 115200 for the arduino
	 */
	motor_driver(std::string port, int baud)
	{
		connection = Serial(port, baud, serial::Timeout::simpleTimeout(1000));
	}

	/**
	 * check_connection
	 * Checks if serial connection is open
	 * 
	 * @return Returns if connection is open or not
	 */
	bool check_connection()
	{
		return connection.isOpen(); 
	}

	/**
	 * set_speed
	 * Sets the speed of a motor
	 *
	 * @param motor Which motor we're setting the speed of
	 * @param speed What to set the speed to
	 */
	void set_speed(int16_t motor, int16_t speed)
	{
		if (speed >= MAX_BACKWARD && speed <= MAX_FORWARD)
		{
			std::string message = format(motor, speed);
			connection.write(message);
		}
	}

	/**
	 * ~motor_driver
	 * Destructor for motor_driver. Closes serial connection.
	 */
	~motor_driver()
	{
		connection.close();
	}
}