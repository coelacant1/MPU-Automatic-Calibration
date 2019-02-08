#pragma once

#include "MPUState.h"

//parse the serial data coming from the teensy

//stores list of gimbal states

class MPUParser {
private:
	long startTime;
	long endTime;

public:
	MPUParser(int MPUNum);
	
};
