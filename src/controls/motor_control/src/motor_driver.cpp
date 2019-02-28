#include <string>
#include <sstream>
#include "serial/serial.h"
#include "../include/motor_driver.h"

using namespace motor_abs;

class motor_driver
{
private:
	serial::Serial connection;
	std::string format(motor_control command)
	{
		std::ostringstream stm;
		stm << command.motor << "," << command.speed << "*";
		return stm.str();
	}

public:
	motor_driver(std::string port, std::int baud = 115200)
	{
		connection = Serial(port, baud, serial::Timeout::simpleTimeout(1000));
	}

	bool check_connection()
	{
		return connection.isOpen(); 
	}

	void set_speed(motor_control command)
	{
		if (command.speed >= MAX_BACKWARD && command.speed <= MAX_FORWARD)
		{
			std::string message = format(command);
			connection.write(message);
		}
	}

	~motor_driver()
	{
		connection.close();
	}
}

class motor_control
{
public:
	int16 motor
	int16 speed
}