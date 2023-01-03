/**
 * File: Units.hpp
 * Author: Kevin Lou
 * Created On: Jan 3 2023
 * Last Updated: Jan 3 2023
 *
 * Copyright (c) 2023 3946E
 *
 * Summary of File:
 *  - Proper configuration to use units on VEX brain
 *  - Enables necessary units
 *  - Enables unit literals
 *  - Enables unit math
 */

#ifndef ROBOTCODE_UNITS_HPP
#define ROBOTCODE_UNITS_HPP

#define UNIT_LIB_DISABLE_IOSTREAM

#define DISABLE_PREDEFINED_UNITS
#define ENABLE_PREDEFINED_LENGTH_UNITS
#define ENABLE_PREDEFINED_TIME_UNITS
#define ENABLE_PREDEFINED_ANGLE_UNITS
#define ENABLE_PREDEFINED_VELOCITY_UNITS
#define ENABLE_PREDEFINED_ACCELERATION_UNITS
#define ENABLE_PREDEFINED_ANGULAR_VELOCITY_UNITS

#undef _U
#undef _L
#undef _N
#undef _S
#undef _P
#undef _C
#undef _X
#undef _B

#include <units.h>

using namespace units::literals;
using namespace units::math;

using namespace units;
using namespace units::length;
using namespace units::time;
using namespace units::angle;
using namespace units::velocity;
using namespace units::acceleration;
using namespace units::angular_velocity;

#endif //ROBOTCODE_UNITS_HPP
