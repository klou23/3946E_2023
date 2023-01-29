#ifndef ROBOTCODE_DRIVE_HPP
#define ROBOTCODE_DRIVE_HPP

#include "Robot.hpp"
#include "../Units/Units.hpp"

class Drive{

    inch_t x;
    inch_t y;
    radian_t theta;
    radian_t theta_0;
    radian_t lastLeft;
    radian_t lastRight;
    radian_t lastBack;

public:

    Drive(inch_t x, inch_t y, radian_t theta) : x(x), y(y), theta(theta) {
        theta_0 = theta;
    }

    Drive() {
        Drive(0_in,0_in,0_rad);
    }

    double curve(double input, double exp){
        double sign = (input < 0) ? (-1.0) : (1.0);
        return sign * std::abs(std::pow(input, exp));
    }

    void op_control(){
//        double lv = curve(master.getLeftY(), 2) + curve(master.getRightX(), 2);
//        double rv = curve(master.getLeftY(), 2) - curve(master.getRightX(), 2);
//        if(std::max(lv, rv) > 1.0){
//            double divide = std::max(lv, rv);
//            lv /= divide;
//            rv /= divide;
//        }
//        fl.move_velocity(lv*600);
//        fr.move_velocity(rv*600);
//        bl.move_velocity(lv*600);
//        br.move_velocity(rv*600);
        double lv = master.getLeftY();
        double rv = master.getRightY();
        fl.move_velocity(lv*600);
        fr.move_velocity(rv*600);
        bl.move_velocity(lv*600);
        br.move_velocity(rv*600);
    }

    void driveFwd(){
        fl.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        fr.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        bl.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        br.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        fl.move_velocity(100);
        fr.move_velocity(100);
        bl.move_velocity(100);
        br.move_velocity(100);
        pros::delay(500);
        fl.move_velocity(0);
        fr.move_velocity(0);
        bl.move_velocity(0);
        br.move_velocity(0);
        fl.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        fr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        bl.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        br.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    }

    void driveRev(){
        fl.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        fr.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        bl.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        br.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        fl.move_velocity(-100);
        fr.move_velocity(-100);
        bl.move_velocity(-100);
        br.move_velocity(-100);
        pros::delay(1000);
        fl.move_velocity(0);
        fr.move_velocity(0);
        bl.move_velocity(0);
        br.move_velocity(0);
        fl.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        fr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        bl.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        br.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    }

    void turn(){
        fl.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        fr.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        bl.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        br.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        fl.move_velocity(-100);
        fr.move_velocity(100);
        bl.move_velocity(-100);
        br.move_velocity(100);
        pros::delay(1500);
        fl.move_velocity(0);
        fr.move_velocity(0);
        bl.move_velocity(0);
        br.move_velocity(0);
        fl.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        fr.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        bl.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        br.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    }
};

#endif //ROBOTCODE_DRIVE_HPP
