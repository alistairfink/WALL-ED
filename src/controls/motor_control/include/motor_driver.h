#ifndef MOTOR_ABS_H_
#define MOTOR_ABS_H_

namespace motor_abs {
	const int16 MOTOR_1 = 1;
	const int16 MOTOR_2 = 2;
	const int16 MAX_FORWARD = 400;
	const int16 MAX_BACKWARD = -400;

	class motor_driver
	{
	private:
		serial::Serial connection;
		std::string format(motor_control command);

	public:
		motor_driver(std::string port, std::int baud = 115200);
		bool check_connection();
		void set_speed(motor_control command);
		~motor_driver();
	}

	class motor_control
	{
	public:
		int16 motor
		int16 speed
	}
}

#endif