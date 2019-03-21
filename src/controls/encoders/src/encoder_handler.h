#ifndef ENCODERS_H
#define ENCODERS_H

#include "ros/ros.h"

class encoder_handler {
public:
	encoder_handler();
	~encoder_handler();

private:
	int16_t motor_l(void);
	int16_t motor_r(void);
	void rotary_clear(void);
};

#endif
