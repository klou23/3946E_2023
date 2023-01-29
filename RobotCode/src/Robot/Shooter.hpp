#ifndef ROBOTCODE_SHOOTER_HPP
#define ROBOTCODE_SHOOTER_HPP

#include "Robot.hpp"
#include "../Units/Units.hpp"

class Shooter{

public:

    bool flywheelOn = false;
    bool indexerPrev = false;
    bool indexerOn = false;

    void op_control(){
        if(partner.aPressed()){
            flywheelOn = true;
        }else if(partner.bPressed()){
            flywheelOn = false;
        }

        if(flywheelOn){
            flywheel1.move_velocity(600);
            flywheel2.move_velocity(600);
        }else{
            flywheel1.move_velocity(0);
            flywheel2.move_velocity(0);
        }

        if(partner.r1Pressed() && !indexerPrev){
            indexerOn = true;
        }
        indexerPrev = partner.r1Pressed();

        if(indexer.get_position() > 240){
            indexerOn = false;
        }

        indexer.move_absolute(indexerOn ? 250:0, 100);

    }

    void on(){
        flywheel1.move_velocity(600);
        flywheel2.move_velocity(600);
    }

    void off(){
        flywheel1.move_velocity(0);
        flywheel2.move_velocity(0);
    }

    void shoot2(){
        indexer.move_absolute(250, 100);
        pros::delay(500);
        indexer.move_absolute(0, 100);
        pros::delay(500);
        indexer.move_absolute(250, 100);
        pros::delay(500);
        indexer.move_absolute(0, 100);
    }
};

#endif //ROBOTCODE_SHOOTER_HPP
