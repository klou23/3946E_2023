#ifndef ROBOTCODE_INTAKE_HPP
#define ROBOTCODE_INTAKE_HPP

#include "Robot.hpp"
#include "../Units/Units.hpp"

class Intake{

public:

    void op_control(){
        if(master.l1Pressed()){
            intake.move_velocity(200);
        }else if(master.l2Pressed()){
            intake.move_velocity(-200);
        }else{
            intake.move_velocity(0);
        }
    }

    void moveFor(int time){
        intake.move_velocity(100);
        pros::delay(time);
        intake.move_velocity(0);
    }

};

#endif //ROBOTCODE_INTAKE_HPP
