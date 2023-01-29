#ifndef INC_3946X_2023_CONTROLLER_HPP
#define INC_3946X_2023_CONTROLLER_HPP

#include "../../include/api.h"

class Controller : public pros::Controller{
public:
    using pros::Controller::Controller;

    double getLeftX(){
        return (double) (get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)) / 127;
    }

    double getLeftY(){
        return (double) (get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) / 127;
    }

    double getRightX(){
        return (double) (get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) / 127;
    }

    double getRightY(){
        return (double) (get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)) / 127;
    }

    bool leftPressed() {
        return get_digital(pros::E_CONTROLLER_DIGITAL_LEFT);
    }

    bool rightPressed() {
        return get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT);
    }

    bool upPressed() {
        return get_digital(pros::E_CONTROLLER_DIGITAL_UP);
    }

    bool downPressed() {
        return get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
    }

    bool aPressed() {
        return get_digital(pros::E_CONTROLLER_DIGITAL_A);
    }

    bool bPressed() {
        return get_digital(pros::E_CONTROLLER_DIGITAL_B);
    }

    bool xPressed() {
        return get_digital(pros::E_CONTROLLER_DIGITAL_X);
    }

    bool yPressed() {
        return get_digital(pros::E_CONTROLLER_DIGITAL_Y);
    }

    bool r1Pressed() {
        return get_digital(pros::E_CONTROLLER_DIGITAL_R1);
    }

    bool r2Pressed() {
        return get_digital(pros::E_CONTROLLER_DIGITAL_R2);
    }

    bool l1Pressed() {
        return get_digital(pros::E_CONTROLLER_DIGITAL_L1);
    }

    bool l2Pressed() {
        return get_digital(pros::E_CONTROLLER_DIGITAL_L2);
    }
};


#endif //INC_3946X_2023_CONTROLLER_HPP
