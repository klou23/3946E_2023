/**
 * File: main.cpp
 * Author: Kevin Lou
 * Created On: Jan 3 2023
 * Last Updated: Jan 3 2023
 *
 * Copyright (c) 2023 3946E
 *
 * Summary of File:
 *  - Contains all robot code
 */

#include "main.h"
#include "LouUI/LouUI.hpp"
#include "Devices/Controller.hpp"
#include "Devices/Encoder.hpp"
#include <fstream>

/**Controller Initialization**/
Controller master(pros::E_CONTROLLER_MASTER);
Controller partner(pros::E_CONTROLLER_PARTNER);

/**Drive Initialization**/
pros::Motor FLDrive(18, pros::E_MOTOR_GEARSET_06, true);
pros::Motor FRDrive(6, pros::E_MOTOR_GEARSET_06, false);
pros::Motor BLDrive(12, pros::E_MOTOR_GEARSET_06, true);
pros::Motor BRDrive(1, pros::E_MOTOR_GEARSET_06, false);
Encoder lDriveEnc(20);
Encoder rDriveEnc(21);

/**Shooter Initialization**/
pros::Motor flywheel1(8, pros::E_MOTOR_GEARSET_06, true);
pros::Motor flywheel2(9, pros::E_MOTOR_GEARSET_06, false);
pros::Motor indexer(13, pros::E_MOTOR_GEARSET_36, false);

/**Intake Initialization**/
pros::Motor intake(10, pros::E_MOTOR_GEARSET_18, false);

/**String Launcher Initialization**/
pros::ADIDigitalOut stringLauncher('A');

/**Auton timing manager**/
double autonTime = 0;

namespace Autonomous{
    enum AutonSide{
        LEFT,
        RIGHT,
        PROG,
        NONE
    };

    AutonSide autonSide = NONE;
    int autonNumber = 1;

    std::string noAutonDescription = "No auton selected";

    std::string progDescription = "Programming Skills";

    std::string left1Description = "left1";
    std::string left2Description = "left2";
    std::string left3Description = "left3";
    std::string left4Description = "left4";

    std::string right1Description = "right1";
    std::string right2Description = "right2";
    std::string right3Description = "right3";
    std::string right4Description = "right4";
}

namespace Drive{
    meter_t x = 0_m;
    meter_t y = 0_m;
    radian_t theta = 0_rad;

    double lastLeft;
    double lastRight;

    inch_t trackingWheelDiam;
    inch_t trackWidth;
    inch_t odomTrackWidth;

    double b;
    double zeta;

    int speed = 600;

    class Path{
    private:
        int N;
        std::vector<meter_t> x;
        std::vector<meter_t> y;
        std::vector<radian_t> theta;
        std::vector<meters_per_second_t> v;
        std::vector<radians_per_second_t> omega;
        int curPoint = 0;

    public:
        Path(std::string pathName){
            std::ifstream fin("/usd/Path" + pathName + ".txt");
            fin >> N;
            x.resize(N);
            y.resize(N);
            theta.resize(N);
            v.resize(N);
            omega.resize(N);
            for(int i = 0; i < N; i++){
                double tempX, tempY, tempTheta, tempV, tempOmega;
                fin >> tempX >> tempY >> tempTheta >> tempV >> tempOmega;
                x[i] = meter_t(tempX);
                y[i] = meter_t(tempY);
                theta[i] = radian_t(tempTheta);
                v[i] = meters_per_second_t(tempV);
                omega[i] = radians_per_second_t(tempOmega);
            }
        }

        meter_t getX(){
            return x[curPoint];
        }

        meter_t getY(){
            return y[curPoint];
        }

        radian_t getTheta(){
            return theta[curPoint];
        }

        meters_per_second_t getV(){
            return v[curPoint];
        }

        radians_per_second_t getOmega(){
            return omega[curPoint];
        }

        void increment(){
            curPoint++;
        }

        bool done(){
            return curPoint >= N;
        }
    };

    void initialize(inch_t _trackingWheelDiam, inch_t _trackWidth, inch_t _odomTrackWidth, double _b, double _zeta){
      	trackingWheelDiam = _trackingWheelDiam;
        trackWidth = _trackWidth;
        odomTrackWidth = _odomTrackWidth;
        b = _b;
        zeta = _zeta;

        lDriveEnc.reverse();
        lDriveEnc.reset_position();
        rDriveEnc.reset_position();

		lastLeft = lDriveEnc.getPosition().value();
		lastRight = rDriveEnc.getPosition().value();
    }

    meters_per_second_t ramseteV(meter_t x, meter_t y, radian_t theta, meter_t xD, meter_t yD, radian_t thetaD,
                                 meters_per_second_t vD, radians_per_second_t omegaD){
        auto deltaX = xD-x;
        auto deltaY = yD-y;
        auto deltaT = thetaD-theta;
        if(std::abs(deltaT.value()) < 0.00001) deltaT += 0.0001_rad;

        double ex = std::cos(theta.value())*deltaX.value() + std::sin(theta.value())*deltaY.value();
        double ey = -std::sin(theta.value())*deltaX.value() + std::cos(theta.value())*deltaY.value();
        double et = deltaT.value();

        double k = 2*zeta*std::sqrt(omegaD.value()*omegaD.value() + b*vD.value()*vD.value());
        return meters_per_second_t(vD.value()*std::cos(et)+k*ex);
    }

    radians_per_second_t ramseteOmega(meter_t x, meter_t y, radian_t theta, meter_t xD, meter_t yD, radian_t thetaD,
                                     meters_per_second_t vD, radians_per_second_t omegaD){
        auto deltaX = xD-x;
        auto deltaY = yD-y;
        auto deltaT = thetaD-theta;
        if(std::abs(deltaT.value()) < 0.00001) deltaT += 0.0001_rad;

        double ex = std::cos(theta.value())*deltaX.value() + std::sin(theta.value())*deltaY.value();
        double ey = -std::sin(theta.value())*deltaX.value() + std::cos(theta.value())*deltaY.value();
        double et = deltaT.value();
        double k = 2*zeta*std::sqrt(omegaD.value()*omegaD.value() + b*vD.value()*vD.value());
        return radians_per_second_t(omegaD.value() + k*et + b*vD.value()*sin(et)*ey/et);
    }

    double mps_rpm(double mps){
        double wheelDiam = 3.25;
        double ips = mps/0.0254;
        double rps = ips/(wheelDiam*M_PI);
        return rps*60;
    }

    revolutions_per_minute_t leftVel(meters_per_second_t v, radians_per_second_t omega){
        meter_t trackWidth_m = trackWidth;
        double groundVel = v.value() - 0.5*trackWidth_m.value()*omega.value();
        return revolutions_per_minute_t(mps_rpm(groundVel));
    }

    revolutions_per_minute_t rightVel(meters_per_second_t v, radians_per_second_t omega){
        meter_t trackWidth_m = trackWidth;
        double groundVel = v.value() + 0.5*trackWidth_m.value()*omega.value();
        return revolutions_per_minute_t(mps_rpm(groundVel));
    }

    void auton(void* param){
        Path p("DriveTest");
        std::ofstream fout("/usd/log.txt");
        FLDrive.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        FRDrive.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        BLDrive.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        BRDrive.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        while(!p.done()){
            auto px = p.getX();
            auto py = p.getY();
            auto pTheta = p.getTheta();
            auto pv = p.getV();
            auto pOmega = p.getOmega();

            fout << x.value() << " " << y.value() << " " << theta.value() << std::endl;

            auto targV = ramseteV(x, y, theta, px, py, pTheta, pv, pOmega);
            auto targOmega = ramseteOmega(x, y, theta, px, py, pTheta, pv, pOmega);

            auto lv = leftVel(targV, targOmega);
            auto rv = rightVel(targV, targOmega);

            if(std::abs(std::max(lv.value(), rv.value())) > 300){
                double maxVel = std::abs(std::max(lv.value(), rv.value()));
                lv *= 300.0/maxVel;
                rv *= 300.0/maxVel;
            }

            FLDrive.move_velocity(lv.value());
            BLDrive.move_velocity(lv.value());
            FRDrive.move_velocity(rv.value());
            BRDrive.move_velocity(rv.value());
            pros::delay(10);
            autonTime += 10;
            p.increment();
        }

        fout.close();

        FLDrive.move_velocity(0);
        BLDrive.move_velocity(0);
        FRDrive.move_velocity(0);
        BRDrive.move_velocity(0);
    }

    void drivePath(std::string pathName){

        Path p(pathName);
        while(!p.done()){
            auto px = p.getX();
            auto py = p.getY();
            auto pTheta = p.getTheta();
            auto pv = p.getV();
            auto pOmega = p.getOmega();

            auto targV = ramseteV(x, y, theta, px, py, pTheta, pv, pOmega);
            auto targOmega = ramseteOmega(x, y, theta, px, py, pTheta, pv, pOmega);

            auto lv = leftVel(targV, targOmega);
            auto rv = rightVel(targV, targOmega);

            if(std::abs(std::max(lv.value(), rv.value())) > 300){
                double maxVel = std::abs(std::max(lv.value(), rv.value()));
                lv *= 300.0/maxVel;
                rv *= 300.0/maxVel;
            }

            FLDrive.move_velocity(lv.value());
            BLDrive.move_velocity(lv.value());
            FRDrive.move_velocity(rv.value());
            BRDrive.move_velocity(rv.value());
            pros::delay(10);
            autonTime += 10;
            p.increment();
        }
        FLDrive.move_velocity(0);
        BLDrive.move_velocity(0);
        FRDrive.move_velocity(0);
        BRDrive.move_velocity(0);
    }

    double curve(double input, double exp){
        double sign = (input < 0) ? (-1.0) : (1.0);
        return sign * std::abs(std::pow(input, exp));
    }

    void odom(void* param){
//        while(true){
//            radian_t curLeft = lDriveEnc.getPosition();
//            radian_t curRight = rDriveEnc.getPosition();
//
//            inch_t deltaL = (curLeft-lastLeft).value() * trackingWheelDiam/2.0;
//            inch_t deltaR = (curRight-lastRight).value() * trackingWheelDiam/2.0;
//
//            lastLeft = curLeft;
//            lastRight = curRight;
//
//            inch_t deltaLr = curLeft.value() * trackingWheelDiam/2;
//            inch_t deltaRr = curRight.value() * trackingWheelDiam/2;
//
//            radian_t thetaNew = radian_t(((deltaRr-deltaLr) / odomTrackWidth).value());
//
//            radian_t deltaTheta = thetaNew - theta;
//
//            inch_t d = (deltaR + deltaL)/2.0;
//            radian_t thetaM = theta + deltaTheta/2.0;
//
//            double cosRot = std::cos(thetaM.value());
//            double sinRot = std::sin(thetaM.value());
//
//            x += d*cosRot;
//            y += d*sinRot;
//            theta = thetaNew;
//
//            pros::delay(1000);
//        }

        while(true){
            double trackingWheelDiam = 2.75;
            double trackingDiam = 3/1.125;

            double curLeft = lDriveEnc.getPosition().value();
            double curRight = rDriveEnc.getPosition().value();

            double deltaL = (curLeft-lastLeft) * trackingWheelDiam/2;
            double deltaR = (curRight-lastRight) * trackingWheelDiam/2;

            lastLeft = curLeft;
            lastRight = curRight;

            double deltaLr = curLeft * trackingWheelDiam/2;
            double deltaRr = curRight * trackingWheelDiam/2;

            double thetaNew = (deltaRr-deltaLr)/trackingDiam;

            double deltaTheta = thetaNew - theta.value();

            double d = (deltaR + deltaL)/2.0;
            double thetaM = theta.value() + deltaTheta/2.0;

            double cosRot = std::cos(thetaM);
            double sinRot = std::sin(thetaM);

            x += inch_t(d*cosRot);
            y += inch_t(d*sinRot);
            theta = radian_t(thetaNew);
            pros::delay(10);
        }

    }

    void op_control(void* param){
        while(true){

            if(master.yPressed()){
                ::autonomous();
            }

            //Set speed
            if(master.upPressed()){
                speed = 600;
            }else if(master.rightPressed()){
              	speed = 450;
            }else if(master.downPressed()){
                speed = 300;
            }

            //move drive
            double lv = curve(master.getLeftY(), 2);
            double rv = curve(master.getRightY(), 2);

            FLDrive.move_velocity(lv*speed);
            FRDrive.move_velocity(rv*speed);
            BLDrive.move_velocity(lv*speed);
            BRDrive.move_velocity(rv*speed);

            pros::delay(10);
        }
    }

//    void drive(inch_t targInch){
//
//        double l0 = lDriveEnc.getPosition().value();
//        double r0 = rDriveEnc.getPosition().value();
//        double lTarg = l0 + targInch.value()/(2.75/2.0);
//        double rTarg = r0 + targInch.value()/(2.75/2.0);
//
//        double le = lTarg - lDriveEnc.getPosition().value();
//        double re = rTarg - rDriveEnc.getPosition().value();
//
//        double p = 0.05;
//        FLDrive.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
//        BLDrive.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
//        FRDrive.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
//        BRDrive.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
//
//        std::cout << "targs: " << lTarg << " " << rTarg << std::endl;
//
//        while(le > 0.1 && re > 0.1){
//            le = lTarg - lDriveEnc.getPosition().value();
//            re = rTarg - rDriveEnc.getPosition().value();
//
//            std::cout << lDriveEnc.getPosition().value() << " " << le << " " << re << std::endl;
//
//            FLDrive.move_velocity(std::min(le*p*600, FLDrive.get_actual_velocity()+25));
//            BLDrive.move_velocity(std::min(le*p*600, BLDrive.get_actual_velocity()+25));
//            FRDrive.move_velocity(std::min(re*p*600, FRDrive.get_actual_velocity()+25));
//            BRDrive.move_velocity(std::min(re*p*600, BRDrive.get_actual_velocity()+25));
//
//
//            pros::delay(10);
//        }
//    }

    void driveTimer(int time, int maxVel){
        int curTime = 0;
        int vel = 0;

        FLDrive.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        BLDrive.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        FRDrive.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        BRDrive.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

        while(curTime < time){
            vel += 25;
            vel = std::min(vel, maxVel);
            curTime += 10;
            FLDrive.move_velocity(vel);
            BLDrive.move_velocity(vel);
            FRDrive.move_velocity(vel);
            BRDrive.move_velocity(vel);
            pros::delay(10);
        }

        for(int i = 0; i < 20; i++){
            vel -= 10;
            FLDrive.move_velocity(vel);
            BLDrive.move_velocity(vel);
            FRDrive.move_velocity(vel);
            BRDrive.move_velocity(vel);
            pros::delay(10);
        }

        FLDrive.move_velocity(0);
        BLDrive.move_velocity(0);
        FRDrive.move_velocity(0);
        BRDrive.move_velocity(0);
    }

    void driveRevTimer(int time, int maxVel){
        int curTime = 0;
        int vel = 0;

        FLDrive.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        BLDrive.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        FRDrive.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        BRDrive.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

        while(curTime < time){
            vel += 10;
            vel = std::min(vel, maxVel);
            curTime += 10;
            FLDrive.move_velocity(-vel);
            BLDrive.move_velocity(-vel);
            FRDrive.move_velocity(-vel);
            BRDrive.move_velocity(-vel);
            pros::delay(10);
        }

        for(int i = 0; i < 20; i++){
            vel -= 10;
            FLDrive.move_velocity(-vel);
            BLDrive.move_velocity(-vel);
            FRDrive.move_velocity(-vel);
            BRDrive.move_velocity(-vel);
            pros::delay(10);
        }

        FLDrive.move_velocity(0);
        BLDrive.move_velocity(0);
        FRDrive.move_velocity(0);
        BRDrive.move_velocity(0);
    }

    void turn(degree_t targetDeg){
        radian_t targetRad = targetDeg;
        double targ = targetRad.value();
        double error = targ - theta.value();
        double p = 0.15;
        FLDrive.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        BLDrive.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        FRDrive.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        BRDrive.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

        while(error > 0.02 || error < -0.02){
//        while(true){
            error = targ - theta.value();
            FLDrive.move_velocity(-error*p*600);
            BLDrive.move_velocity(-error*p*600);
            FRDrive.move_velocity(error*p*600);
            BRDrive.move_velocity(error*p*600);
            std::cout << theta.value() << " " << error << std::endl;
            pros::delay(10);
        }

        FLDrive.move_velocity(0);
        BLDrive.move_velocity(0);
        FRDrive.move_velocity(0);
        BRDrive.move_velocity(0);
    }

}

namespace Shooter{

    revolutions_per_minute_t targetVel = 0_rpm;
    double gain;

    void initialize(double _gain){
        gain = _gain;
    }

    double signum(double a){
        return (a>0) ? 1.0 : -1.0;
    }

    double output = 0;
    revolutions_per_minute_t prevError = 0_rpm;
    double tbh = 0;

    void setVel(int vel){
        if(vel == 3000){
            targetVel = 3000_rpm;
            tbh = 1;
            output = 1;
        }
        if(vel == 2800){
            targetVel = 2800_rpm;
            tbh = 0.93;
            output = 0.93;
        }
        if(vel == 2400){
            targetVel = 2400_rpm;
            tbh = 0.83;
            output = 0.83;
        }else if(vel == 0){
            targetVel = 0_rpm;
            tbh = 0;
            output = 0;
        }
    }

    void auton(void* param){
        while(true){
            //run flywheel using TBH controller
            revolutions_per_minute_t error = targetVel - revolutions_per_minute_t(flywheel1.get_actual_velocity()*5);
            output += gain * error.value();
            output = std::min(output, 1.0);
            output = std::max(output, 0.0);
            if(signum(error.value()) != signum(prevError.value())){
                output = 0.5 * (output + tbh);
                tbh = output;
                prevError = error;
            }
            if(targetVel.value() < 1) output = 0;
            flywheel1.move_voltage(output*12000);
            flywheel2.move_voltage(output*12000);

            pros::delay(10);
        }

    }

    void op_control(void* param){
        double output = 0;
        revolutions_per_minute_t prevError = 0_rpm;
        double tbh = 0;

        bool indexerPrev = false;
        bool indexerOn = false;

        while(true){
            //set target vel, tbh, and output
            if(partner.upPressed()){
                targetVel = 3000_rpm;
                tbh = 1;
                output = 1;
            }
            else if(partner.rightPressed()){
                targetVel = 2400_rpm;
                tbh = 0.83;
                output = 0.83;
            }
            else if(partner.downPressed()){
                targetVel = 2100_rpm;
                tbh = 0.77;
                output = 0.77;
            }
            else if(partner.bPressed()){
                targetVel = 0_rpm;
                tbh = 0;
                output = 0;
            }

            //run flywheel using TBH controller
            revolutions_per_minute_t error = targetVel - revolutions_per_minute_t(flywheel1.get_actual_velocity()*5);
            output += gain * error.value();
            output = std::min(output, 1.0);
            output = std::max(output, 0.0);
            if(signum(error.value()) != signum(prevError.value())){
                output = 0.5 * (output + tbh);
                tbh = output;
                prevError = error;
            }
            if(targetVel.value() < 1) output = 0;
            flywheel1.move_voltage(output*12000);
            flywheel2.move_voltage(output*12000);

            //control indexer
            if(partner.r1Pressed() && !indexerPrev && targetVel.value() > 0.001){
                indexerOn = true;
            }else if(partner.r2Pressed()){
                indexerOn = false;
            }
            indexerPrev = partner.r1Pressed();

            if(indexer.get_position() > 290){
                indexerOn = false;
            }

            indexer.move_absolute(indexerOn ? 300:0, 100);

            pros::delay(10);
        }

    }

    void shoot(){
        indexer.move_absolute(300, 100);
        while(indexer.get_position() < 290) pros::delay(10);
        indexer.move_absolute(0, 100);
    }
}

namespace Intake{

    void initialize(){

    }

    void setVel(int vel){
        intake.move_velocity(vel);
    }

    void op_control(void* param){
        while(true){
            if(master.l1Pressed()){
                intake.move_velocity(200);
            }else if(master.l2Pressed()){
                intake.move_velocity(-200);
            }else if(master.r1Pressed()){
                intake.move_velocity(100);
            }else if(master.r2Pressed()){
                intake.move_velocity(-100);
            }else{
                intake.move_velocity(0);
            }

            pros::delay(10);
        }
    }
}

namespace StringLauncher{
    void initialize(){}

    void op_control(void* param){
        while(true){
            if(master.leftPressed() && partner.leftPressed()){
                stringLauncher.set_value(1);
            }
            pros::delay(50);
        }
    }
}

namespace UI{

    LouUI::Display display;

    /***Auton Selector***/
    LouUI::ToggleButton* leftButton = nullptr;
    LouUI::Label* leftButtonLabel = nullptr;
    LouUI::ToggleButton* rightButton = nullptr;
    LouUI::Label* rightButtonLabel = nullptr;
    LouUI::ToggleButton* progButton = nullptr;
    LouUI::Label* progButtonLabel = nullptr;
    LouUI::ToggleButton* auton1Button = nullptr;
    LouUI::Label* auton1ButtonLabel = nullptr;
    LouUI::ToggleButton* auton2Button = nullptr;
    LouUI::Label* auton2ButtonLabel = nullptr;
    LouUI::ToggleButton* auton3Button = nullptr;
    LouUI::Label* auton3ButtonLabel = nullptr;
    LouUI::ToggleButton* auton4Button = nullptr;
    LouUI::Label* auton4ButtonLabel = nullptr;
    LouUI::Label* autonDescription = nullptr;

    /***Drive Data***/
    LouUI::Gauge* flTempGauge = nullptr;
    LouUI::Label* flTempGaugeLabel = nullptr;
    LouUI::Gauge* frTempGauge = nullptr;
    LouUI::Label* frTempGaugeLabel = nullptr;
    LouUI::Gauge* blTempGauge = nullptr;
    LouUI::Label* blTempGaugeLabel = nullptr;
    LouUI::Gauge* brTempGauge = nullptr;
    LouUI::Label* brTempGaugeLabel = nullptr;
    LouUI::Label* odomLabel = nullptr;
    LouUI::Label* odomXLabel = nullptr;
    LouUI::Label* odomYLabel = nullptr;
    LouUI::Label* odomThetaLabel = nullptr;

    /***Flywheel***/
    LouUI::Gauge* flywheelSpeedGauge = nullptr;
    LouUI::Label* flywheelSpeedLabel = nullptr;
    LouUI::Gauge* flywheel1TempGauge = nullptr;
    LouUI::Gauge* flywheel2TempGuage = nullptr;

    void setAuton(){
        if(progButton->isToggled()){
            auton1Button->setState(LouUI::ToggleButton::RELEASED);
            auton2Button->setState(LouUI::ToggleButton::RELEASED);
            auton3Button->setState(LouUI::ToggleButton::RELEASED);
            auton4Button->setState(LouUI::ToggleButton::RELEASED);
            Autonomous::autonSide = Autonomous::PROG;
            Autonomous::autonNumber = 0;
            autonDescription->setText(Autonomous::progDescription);
            return;
        }

        if(leftButton->isToggled()) Autonomous::autonSide = Autonomous::LEFT;
        else if(rightButton->isToggled()) Autonomous::autonSide = Autonomous::RIGHT;
        else Autonomous::autonSide = Autonomous::NONE;

        if(auton1Button->isToggled()) Autonomous::autonNumber = 1;
        else if(auton2Button->isToggled()) Autonomous::autonNumber = 2;
        else if(auton3Button->isToggled()) Autonomous::autonNumber = 3;
        else if(auton4Button->isToggled()) Autonomous::autonNumber = 4;
        else Autonomous::autonNumber = 0;

        if(Autonomous::autonSide == Autonomous::NONE){
            autonDescription->setText(Autonomous::noAutonDescription);
        }else if(Autonomous::autonSide == Autonomous::LEFT){
            switch (Autonomous::autonNumber) {
                case 1:
                    autonDescription->setText(Autonomous::left1Description);
                    break;
                case 2:
                    autonDescription->setText(Autonomous::left2Description);
                    break;
                case 3:
                    autonDescription->setText(Autonomous::left3Description);
                    break;
                case 4:
                    autonDescription->setText(Autonomous::left4Description);
                    break;
                default:
                    autonDescription->setText(Autonomous::noAutonDescription);
            }
        }else if(Autonomous::autonSide == Autonomous::RIGHT){
            switch (Autonomous::autonNumber) {
                case 1:
                    autonDescription->setText(Autonomous::right1Description);
                    break;
                case 2:
                    autonDescription->setText(Autonomous::right2Description);
                    break;
                case 3:
                    autonDescription->setText(Autonomous::right3Description);
                    break;
                case 4:
                    autonDescription->setText(Autonomous::right4Description);
                    break;
                default:
                    autonDescription->setText(Autonomous::noAutonDescription);
            }
        }
    }

    lv_res_t selectLeft(lv_obj_t *obj){
        rightButton->setState(LouUI::ToggleButton::RELEASED);
        progButton->setState(LouUI::ToggleButton::RELEASED);
        setAuton();
        return LV_RES_OK;
    }
    lv_res_t selectRight(lv_obj_t *obj){
        leftButton->setState(LouUI::ToggleButton::RELEASED);
        progButton->setState(LouUI::ToggleButton::RELEASED);
        setAuton();
        return LV_RES_OK;
    }
    lv_res_t selectProg(lv_obj_t *obj){
        leftButton->setState(LouUI::ToggleButton::RELEASED);
        rightButton->setState(LouUI::ToggleButton::RELEASED);
        setAuton();
        return LV_RES_OK;
    }
    lv_res_t selectAuton1(lv_obj_t *obj){
        auton2Button->setState(LouUI::ToggleButton::RELEASED);
        auton3Button->setState(LouUI::ToggleButton::RELEASED);
        auton4Button->setState(LouUI::ToggleButton::RELEASED);
        setAuton();
        return LV_RES_OK;
    }
    lv_res_t selectAuton2(lv_obj_t *obj){
        auton1Button->setState(LouUI::ToggleButton::RELEASED);
        auton3Button->setState(LouUI::ToggleButton::RELEASED);
        auton4Button->setState(LouUI::ToggleButton::RELEASED);
        setAuton();
        return LV_RES_OK;
    }
    lv_res_t selectAuton3(lv_obj_t *obj){
        auton1Button->setState(LouUI::ToggleButton::RELEASED);
        auton2Button->setState(LouUI::ToggleButton::RELEASED);
        auton4Button->setState(LouUI::ToggleButton::RELEASED);
        setAuton();
        return LV_RES_OK;
    }
    lv_res_t selectAuton4(lv_obj_t *obj){
        auton1Button->setState(LouUI::ToggleButton::RELEASED);
        auton2Button->setState(LouUI::ToggleButton::RELEASED);
        auton3Button->setState(LouUI::ToggleButton::RELEASED);
        setAuton();
        return LV_RES_OK;
    }

    void initialize(){
        display.addScreen("Auton");
        display.addScreen("Drive");
        display.addScreen("Flywheel");

        /***Auton***/
        leftButton = (new LouUI::ToggleButton(display.getScreen("Auton")))
                ->setPosition(27, 0)
                ->setSize(120, 40)
                ->setMainColor(LouUI::Color(140,140,140), LouUI::ToggleButton::ALL_PRESSED)
                ->setGradientColor(LouUI::Color(140,140,140), LouUI::ToggleButton::ALL_PRESSED)
                ->setMainColor(LouUI::Color(125, 125, 125), LouUI::ToggleButton::ALL_RELEASED)
                ->setGradientColor(LouUI::Color(125, 125, 125), LouUI::ToggleButton::ALL_RELEASED)
                ->setBorderColor(LouUI::Color("WHITE"), LouUI::ToggleButton::ALL)
                ->setBorderOpacity(255, LouUI::ToggleButton::ALL)
                ->setBorderWidth(5, LouUI::ToggleButton::ALL_TOGGLED)
                ->setBorderWidth(0, LouUI::ToggleButton::ALL_UNTOGGLED);
        rightButton = (new LouUI::ToggleButton(display.getScreen("Auton"), *leftButton))
                ->align(leftButton->getObj(), LouUI::OUT_RIGHT_MID, 20, 0);
        progButton = (new LouUI::ToggleButton(display.getScreen("Auton")))
                ->setSize(120, 40)
                ->setMainColor(LouUI::Color(0,140,0), LouUI::ToggleButton::ALL_PRESSED)
                ->setGradientColor(LouUI::Color(0,140,0), LouUI::ToggleButton::ALL_PRESSED)
                ->setMainColor(LouUI::Color(0, 125, 0), LouUI::ToggleButton::ALL_RELEASED)
                ->setGradientColor(LouUI::Color(0, 125, 0), LouUI::ToggleButton::ALL_RELEASED)
                ->setBorderColor(LouUI::Color("WHITE"), LouUI::ToggleButton::ALL)
                ->setBorderOpacity(255, LouUI::ToggleButton::ALL)
                ->setBorderWidth(5, LouUI::ToggleButton::ALL_TOGGLED)
                ->setBorderWidth(0, LouUI::ToggleButton::ALL_UNTOGGLED)
                ->align(rightButton->getObj(), LouUI::OUT_RIGHT_MID, 20, 0);

        auton1Button = (new LouUI::ToggleButton(display.getScreen("Auton"), *leftButton))
                ->setPosition(5, 60)
                ->setSize(100, 40);
        auton2Button = (new LouUI::ToggleButton(display.getScreen("Auton"), *auton1Button))
                ->align(auton1Button->getObj(), LouUI::OUT_RIGHT_MID, 10, 0);
        auton3Button = (new LouUI::ToggleButton(display.getScreen("Auton"), *auton1Button))
                ->align(auton2Button->getObj(), LouUI::OUT_RIGHT_MID, 10, 0);
        auton4Button = (new LouUI::ToggleButton(display.getScreen("Auton"), *auton1Button))
                ->align(auton3Button->getObj(), LouUI::OUT_RIGHT_MID, 10, 0);

        leftButtonLabel = (new LouUI::Label(leftButton->getObj()))
                ->setText("LEFT");
        rightButtonLabel = (new LouUI::Label(rightButton->getObj()))
                ->setText("RIGHT");
        progButtonLabel = (new LouUI::Label(progButton->getObj()))
                ->setText("PROG");
        auton1ButtonLabel = (new LouUI::Label(auton1Button->getObj()))
                ->setText("1");
        auton2ButtonLabel = (new LouUI::Label(auton2Button->getObj()))
                ->setText("2");
        auton3ButtonLabel = (new LouUI::Label(auton3Button->getObj()))
                ->setText("3");
        auton4ButtonLabel = (new LouUI::Label(auton4Button->getObj()))
                ->setText("4");

        leftButton->setAction(selectLeft);
        rightButton->setAction(selectRight);
        progButton->setAction(selectProg);

        auton1Button->setAction(selectAuton1);
        auton2Button->setAction(selectAuton2);
        auton3Button->setAction(selectAuton3);
        auton4Button->setAction(selectAuton4);

        autonDescription = (new LouUI::Label(display.getScreen("Auton")))
                ->setTextAlign(LouUI::Label::LEFT)
                ->setLongMode(LouUI::Label::BREAK)
                ->setWidth(440)
                ->align(display.getScreen("Auton"), LouUI::CENTER, 0, 55)
                ->setText(Autonomous::noAutonDescription);

        /***Drive***/

        flTempGauge = (new LouUI::Gauge(display.getScreen("Drive")))
                ->setPosition(20, 0)
                ->setSize(100, 100)
                ->setScale(200, 11, 0)
                ->setRange(20, 70)
                ->setCriticalValue(55)
                ->setValue(20);

        blTempGauge = (new LouUI::Gauge(display.getScreen("Drive")))
                ->setSize(100, 100)
                ->setScale(200, 11, 0)
                ->setRange(20, 70)
                ->setCriticalValue(55)
                ->setValue(20)
                ->align(flTempGauge->getObj(), LouUI::Align::OUT_BOTTOM_MID, 0, -20);

        frTempGauge = (new LouUI::Gauge(display.getScreen("Drive")))
                ->setSize(100, 100)
                ->setScale(200, 11, 0)
                ->setRange(20, 70)
                ->setCriticalValue(55)
                ->setValue(20)
                ->align(flTempGauge->getObj(), LouUI::Align::OUT_RIGHT_MID, 40, 0);

        brTempGauge = (new LouUI::Gauge(display.getScreen("Drive")))
                ->setSize(100, 100)
                ->setScale(200, 11, 0)
                ->setRange(20, 70)
                ->setCriticalValue(55)
                ->setValue(20)
                ->align(frTempGauge->getObj(), LouUI::Align::OUT_BOTTOM_MID, 0, -20);


        flTempGaugeLabel = (new LouUI::Label(display.getScreen("Drive")))
                ->setText("FL")
                ->align(flTempGauge->getObj(), LouUI::Align::IN_TOP_LEFT, -20, 0);

        blTempGaugeLabel = (new LouUI::Label(display.getScreen("Drive")))
                ->setText("BL")
                ->align(blTempGauge->getObj(), LouUI::Align::IN_TOP_LEFT, -20, 0);

        frTempGaugeLabel = (new LouUI::Label(display.getScreen("Drive")))
                ->setText("FR")
                ->align(frTempGauge->getObj(), LouUI::Align::IN_TOP_LEFT, -20, 0);

        brTempGaugeLabel = (new LouUI::Label(display.getScreen("Drive")))
                ->setText("BR")
                ->align(brTempGauge->getObj(), LouUI::Align::IN_TOP_LEFT, -20, 0);

        odomLabel = (new LouUI::Label(display.getScreen("Drive")))
                ->setText("Odom")
                ->align(frTempGaugeLabel->getObj(), LouUI::Align::OUT_RIGHT_MID, 120, 15);

        odomXLabel = (new LouUI::Label(display.getScreen("Drive")))
                ->setText("X - 0")
                ->align(odomLabel->getObj(), LouUI::Align::OUT_BOTTOM_MID, 20, 15);

        odomYLabel = (new LouUI::Label(display.getScreen("Drive")))
                ->setText("Y - 0")
                ->align(odomXLabel->getObj(), LouUI::Align::OUT_BOTTOM_MID, 0, 10);

        odomThetaLabel = (new LouUI::Label(display.getScreen("Drive")))
                ->setText("T - 0")
                ->align(odomYLabel->getObj(), LouUI::Align::OUT_BOTTOM_MID, 0, 10);

        /***Flywheel***/
        flywheelSpeedGauge = (new LouUI::Gauge(display.getScreen("Flywheel")))
                ->setPosition(0, 0)
                ->setSize(190, 190)
                ->setScale(250, 31, 7)
                ->setRange(0, 3000)
                ->setCriticalValue(2800)
                ->setValue(20);

        flywheelSpeedLabel = (new LouUI::Label(display.getScreen("Flywheel")))
                ->setText("RPM")
                ->align(flywheelSpeedGauge->getObj(), LouUI::Align::IN_BOTTOM_MID, 0, -30);

        flywheel1TempGauge = (new LouUI::Gauge(display.getScreen("Flywheel")))
                ->setSize(100, 100)
                ->setScale(200, 11, 0)
                ->setRange(20, 70)
                ->setCriticalValue(55)
                ->setValue(20)
                ->align(flywheelSpeedGauge->getObj(), LouUI::Align::OUT_RIGHT_TOP, 50, 0);

        flywheel2TempGuage = (new LouUI::Gauge(display.getScreen("Flywheel")))
                ->setSize(100, 100)
                ->setScale(200, 11, 0)
                ->setRange(20, 70)
                ->setCriticalValue(55)
                ->setValue(20)
                ->align(flywheel1TempGauge->getObj(), LouUI::Align::OUT_BOTTOM_MID, 0, -20);
    }

    void update(){
        while(true){

            //Drive
            flTempGauge->setValue(FLDrive.get_temperature());
            blTempGauge->setValue(BLDrive.get_temperature());
            frTempGauge->setValue(FRDrive.get_temperature());
            brTempGauge->setValue(BRDrive.get_temperature());

            //Flywheel
            flywheelSpeedGauge->setValue(flywheel1.get_actual_velocity()*5);
            flywheel1TempGauge->setValue(flywheel1.get_temperature());
            flywheel2TempGuage->setValue(flywheel2.get_temperature());

            //Drive
            inch_t driveX = ::Drive::x;
            inch_t driveY = ::Drive::y;
            degree_t driveTheta = ::Drive::theta;
            odomXLabel->setText("X - " + std::to_string(driveX.value()));
            odomYLabel->setText("Y - " + std::to_string(driveY.value()));
            odomThetaLabel->setText("Theta - " + std::to_string(driveTheta.value()));

            pros::delay(50);
        }
    }
}

void initialize() {
    UI::initialize();
    Drive::initialize(2.75_in, 17_in, 3_in, 2.0, 0.7);
    Shooter::initialize(0.00001);
    Intake::initialize();
    StringLauncher::initialize();
}

void disabled() {
    //not used
}

void competition_initialize() {
    //not used
}

void prog(){
    Drive::driveTimer(100, 100);
    Intake::setVel(100);
    pros::delay(500);
    Intake::setVel(0);
    Drive::driveRevTimer(800, 100);
    Shooter::setVel(2800);
    Drive::turn(178_deg);
    Shooter::shoot();
    pros::delay(900);
    Shooter::shoot();
    pros::delay(300);

    Shooter::setVel(0);
    Drive::driveTimer(1000, 200);
    Drive::driveRevTimer(200, 200);
    Drive::turn(260_deg);
    Drive::driveTimer(900, 150);
    Drive::driveTimer(950, 50);
    Intake::setVel(100);
    pros::delay(400);
    Intake::setVel(0);
    Drive::driveRevTimer(500, 200);

    Drive::turn(175_deg);
    Drive::driveTimer(1500, 200);
    Drive::turn(120.5_deg);
    Intake::setVel(200);
    Drive::driveTimer(4400, 300);
    Intake::setVel(-200);

    Drive::driveRevTimer(600, 200);
    Intake::setVel(0);
    Shooter::setVel(2800);
    Drive::turn(230_deg);
    Shooter::shoot();
    pros::delay(800);
    Shooter::shoot();
    pros::delay(800);
    Shooter::shoot();
    pros::delay(300);
    Shooter::setVel(0);

    Drive::turn(160_deg);
    Drive::driveTimer(1800, 150);
    Drive::driveTimer(500, 50);
    Intake::setVel(100);
    pros::delay(300);
    Intake::setVel(0);
    Drive::driveRevTimer(900, 200);
    Drive::turn(275_deg);
    Drive::driveRevTimer(400, 150);
    stringLauncher.set_value(1);
}

void auton(){
    Drive::driveTimer(100, 100);
    Intake::setVel(-100);
    pros::delay(400);
    Intake::setVel(0);
    Drive::driveRevTimer(200, 100);
}

void auton1(){
    Drive::driveTimer(2000, 100);
    Drive::turn(-45_deg);
    Drive::driveTimer(1000, 100);
    Intake::setVel(-100);
    pros::delay(400);
    Intake::setVel(0);
    Drive::driveRevTimer(400, 100);
}

void autonomous() {
    pros::Task fly(Shooter::auton);
    pros::Task odom(Drive::odom);
//    pros::Task driveTask(Drive::auton);
//    pros::Task intakeTask(Intake::auton);
//    Drive::drivePath("prog_1");
//    pros::delay(500);
//    Drive::drivePath("prog_2");
//    Drive::turn(360_deg);
//    Drive::drive(1_in);
//    Intake::setVel(100);
//    pros::delay(500);
//    Intake::setVel(0);
//    Drive::drive(-24_in);

//    prog();
    auton1();
}

void opcontrol() {
    pros::Task odom(Drive::odom);
    pros::Task driveTask(Drive::op_control);
    pros::Task shooterTask(Shooter::op_control);
    pros::Task intakeTask(Intake::op_control);
    pros::Task stringLauncher(StringLauncher::op_control);
    pros::Task updateUI(UI::update);
}
