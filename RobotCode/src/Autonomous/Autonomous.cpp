/**
 * File: Autonomous.cpp
 * Author: Kevin Lou
 * Created On: Jan 3 2023
 * Last Updated: Jan 3 2023
 *
 * Copyright (c) 2023 3946E
 *
 * Summary of File:
 *  - Contains everything that controls the autonomous period
 */

#include "Autonomous.hpp"

Autonomous::AutonSide Autonomous::autonSide = NONE;
int Autonomous::autonNumber = 1;

std::string Autonomous::noAutonDescription = "No auton selected";

std::string Autonomous::progDescription = "Prog Skills";

std::string Autonomous::left1Description = "left1";
std::string Autonomous::left2Description = "left2";
std::string Autonomous::left3Description = "left3";
std::string Autonomous::left4Description = "left4";

std::string Autonomous::right1Description = "right1";
std::string Autonomous::right2Description = "right2";
std::string Autonomous::right3Description = "right3";
std::string Autonomous::right4Description = "right4";
