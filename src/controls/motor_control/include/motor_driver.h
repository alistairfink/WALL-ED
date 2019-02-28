#ifndef MOTOR_ABS_H_
#define MOTOR_ABS_H_

namespace motor_abs {
	/**
	 * Motor Constants
	 * Useful for changing speeds.
	 */
	const int16 MOTOR_1 = 1;
	const int16 MOTOR_2 = 2;
	const int16 MAX_FORWARD = 400;
	const int16 MAX_BACKWARD = -400;

	class motor_driver
	{
	private:
		serial::Serial connection;
		std::string format(int16 motor, int16 speed);

	public:
		motor_driver(std::string port, std::int baud = 115200);
		bool check_connection();	
		void set_speed(int16 motor, int16 speed);
		~motor_driver();
	}
}

#endif