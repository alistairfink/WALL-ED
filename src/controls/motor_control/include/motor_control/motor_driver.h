#ifndef MOTOR_ABS_H_
#define MOTOR_ABS_H_

#include "ros/ros.h"
#include "serial/serial.h"

namespace motor_abs {
	/**
	 * Motor Constants
	 * Useful for changing speeds.
	 */
	const int16_t MOTOR_1 = 1;
	const int16_t MOTOR_2 = 2;
	const int16_t MAX_FORWARD = 400;
	const int16_t MAX_BACKWARD = -400;

	class motor_driver
	{
	private:
		serial::Serial connection;
		string format(int16_t motor, int16_t speed);

	public:
		motor_driver(string port, int baud);
		bool check_connection();	
		void set_speed(int16_t motor, int16_t speed);
		~motor_driver();
	}
}

#endif