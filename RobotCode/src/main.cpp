/**
 * File: main.cpp
 * Author: Kevin Lou
 * Created On: Jan 3 2023
 * Last Updated: Jan 3 2023
 *
 * Copyright (c) 2023 3946E
 *
 * Summary of File:
 *  - Contains functions for VEX competition template
 */

#include "main.h"
#include "Units/Units.hpp"
#include "UI/UI.hpp"
#include "Robot/Drive.hpp"
#include "Robot/Intake.hpp"
#include "Robot/Shooter.hpp"

Drive driveSubsystem(0_in,0_in,0_rad);
Intake intakeSubsystem;
Shooter shooterSubsystem;
pros::ADIDigitalOut piston ('A');

void initialize() {
    UI::initialize();
}

void disabled() {
    //not used
}

void competition_initialize() {
    //not used
}

void autonomous() {
    driveSubsystem.driveFwd();
    intakeSubsystem.moveFor(400);
    driveSubsystem.driveRev();
    shooterSubsystem.on();
    driveSubsystem.turn();
    shooterSubsystem.shoot2();
    pros::delay(500);
    shooterSubsystem.off();
}

void opcontrol() {

    while(true){
        driveSubsystem.op_control();
        intakeSubsystem.op_control();
        shooterSubsystem.op_control();

        if(master.yPressed()){
            autonomous();
        }

        if(master.leftPressed() && partner.leftPressed()){
            piston.set_value(1);
        }
    }
}
