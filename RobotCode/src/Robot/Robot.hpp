/**
 * File: Robot.hpp
 * Author: Kevin Lou
 * Created On: Jan 16 2023
 * Last Updated: Jan 16 2023
 *
 * Copyright (c) 2023 3946E
 */

#ifndef ROBOTCODE_ROBOT_HPP
#define ROBOTCODE_ROBOT_HPP

#include "../../include/api.h"
#include "../Devices/Encoder.hpp"
#include "../Devices/Controller.hpp"

/**Controllers**/
extern Controller master;
extern Controller partner;

/**Drive**/
extern pros::Motor fl, fr, bl, br;
extern Encoder lEnc, rEnc, bEnc;

/**Shooter**/
extern pros::Motor flywheel1, flywheel2, indexer;

/**Intake**/
extern pros::Motor intake;

#endif //ROBOTCODE_ROBOT_HPP
