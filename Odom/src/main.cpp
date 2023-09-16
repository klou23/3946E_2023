#include "main.h"
#include "squiggles.hpp"
#include <fstream>

#define SKILLS false

/*** Initialization ***/
Controller master(okapi::ControllerId::master);
Controller partner(okapi::ControllerId::partner);

Motor l1(3, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor l2(4, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor r1(10, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor r2(12, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);

Motor intake(9, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);

Motor flywheel1(2, true, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
Motor flywheel2(5, false, AbstractMotor::gearset::blue, AbstractMotor::encoderUnits::degrees);
//Motor indexer(21, true, AbstractMotor::gearset::red, AbstractMotor::encoderUnits::degrees);
Motor indexer(21, true, AbstractMotor::gearset::red, AbstractMotor::encoderUnits::degrees);

pros::IMU gyro(7);

RotationSensor odomL(13, true);
RotationSensor odomR(11);

pros::Optical opticalSensor(20);

pros::ADIDigitalOut stringLauncher('B');

const QLength lOdomWheelDiam = 2.7655734109_in;
const QLength rOdomWheelDiam = 2.732338269_in;
const QLength odomTrackWidth = 2.5507200007_in;

const double ROBOT_WIDTH = 0.3556;  //meters

const double MAX_VEL_FAST = 1;     // m/s
const double MAX_ACCEL_FAST = 1.5;   // m/s^2
const double MAX_JERK_FAST = 2.0;    // m/s^3

const double MAX_VEL_MED = 0.5;     // m/s
const double MAX_ACCEL_MED = 1.0;   // m/s^2
const double MAX_JERK_MED = 2.0;    // m/s^3

const double MAX_VEL_SLOW = 0.25;    // m/s
const double MAX_ACCEL_SLOW = 0.75;  // m/s^2
const double MAX_JERK_SLOW = 1.5;  // m/s^3

const double MAX_VEL_SUPER_SLOW = 0.1;      // m/s
const double MAX_ACCEL_SUPER_SLOW = 1.0;    // m/s^2
const double MAX_JERK_SUPER_SLOW = 1.5;     // m/s^3

squiggles::Constraints constraintsFast = squiggles::Constraints(MAX_VEL_FAST, MAX_ACCEL_FAST, MAX_JERK_FAST);
squiggles::Constraints constraintsMed = squiggles::Constraints(MAX_VEL_MED, MAX_ACCEL_MED, MAX_JERK_MED);
squiggles::Constraints constraintsSlow = squiggles::Constraints(MAX_VEL_SLOW, MAX_ACCEL_SLOW, MAX_JERK_SLOW);
squiggles::Constraints constraintsSuperSlow = squiggles::Constraints(MAX_VEL_SUPER_SLOW, MAX_ACCEL_SUPER_SLOW,
                                                                     MAX_JERK_SUPER_SLOW);

squiggles::SplineGenerator generatorFast = squiggles::SplineGenerator(
        constraintsFast,
        std::make_shared<squiggles::TankModel>(ROBOT_WIDTH, constraintsFast), 0.01);
squiggles::SplineGenerator generatorMed = squiggles::SplineGenerator(
        constraintsMed,
        std::make_shared<squiggles::TankModel>(ROBOT_WIDTH, constraintsMed), 0.01);
squiggles::SplineGenerator generatorSlow = squiggles::SplineGenerator(
        constraintsSlow,
        std::make_shared<squiggles::TankModel>(ROBOT_WIDTH, constraintsSlow), 0.01);
squiggles::SplineGenerator generatorSuperSlow = squiggles::SplineGenerator(
        constraintsSuperSlow,
        std::make_shared<squiggles::TankModel>(ROBOT_WIDTH, constraintsSuperSlow), 0.01);

std::shared_ptr<ChassisController> chassis =
        ChassisControllerBuilder()
                .withMotors({-3,-4},{10,12})
                        // Green gearset, 4 in wheel diam, 11.5 in wheel track
                .withDimensions({AbstractMotor::gearset::blue, (72.0 / 36.0)}, {{3.25_in, 14.11_in}, imev5BlueTPR})
                .withMaxVelocity(300)
                .build();

QLength robotX;
QLength robotY;
QAngle robotTheta;

double lastLeft;
double lastRight;

void odometry(void* param){

    while(true){
//        double trackingWheelDiam = odomWheelDiam.convert(inch);
        double trackingWheelDiam = 2.75;
        double trackingDiam = odomTrackWidth.convert(inch);

        double curLeft = odomL.get() * (M_PI/180.0);
        double curRight = odomR.get() * (M_PI/180.0);

        double deltaL = (curLeft-lastLeft) * trackingWheelDiam/2;
        double deltaR = (curRight-lastRight) * trackingWheelDiam/2;

        lastLeft = curLeft;
        lastRight = curRight;

        double d = (deltaL + deltaR)/2.0;

//        QAngle thetaNew = radian * ((curRight-curLeft)/trackingDiam);
//        QAngle thetaNew = robotTheta + radian * ((deltaR - deltaL)/trackingDiam);
        QAngle thetaNew = gyro.get_rotation()*-1.0202751342*degree;
        double thetaM = (robotTheta.convert(radian)+thetaNew.convert(radian))/2.0;

        double cosRot = std::cos(thetaM);
        double sinRot = std::sin(thetaM);

        robotX += (d * cosRot) * inch;
        robotY += (d * sinRot) * inch;
        robotTheta = thetaNew;

        pros::delay(10);
    }
}

squiggles::Pose makePose(QLength x, QLength y, QAngle theta){
    double x_m = x.convert(meter);
    double y_m = y.convert(meter);
    double theta_rad = theta.convert(radian);
    return squiggles::Pose(x_m, y_m, theta_rad);
}

double wheelVeltoRPM(double wheelVel){
    double inchesPerMinute = wheelVel * 2362.2;
    double wheelCircumferenceInch = 10.2101761242;
    double RPM = inchesPerMinute/wheelCircumferenceInch;
    return RPM*2.0;
}

void followPath(std::vector<squiggles::ProfilePoint> path){

    int index = 0;
    while(true){
        std::ifstream fin("/usd/log_target" + std::to_string(index) + ".txt");
        if(!fin.good()) break;
        else index++;
    }

    std::ofstream fout_actual("/usd/log_actual" + std::to_string(index) + ".txt");
    std::ofstream fout_target("/usd/log_target" + std::to_string(index) + ".txt");

    for(squiggles::ProfilePoint p : path){

        double targX = p.vector.pose.x;
        double targY = p.vector.pose.y;
        double targTheta = p.vector.pose.yaw;

        fout_actual << robotX.convert(meter) << " " << robotY.convert(meter) << " " << robotTheta.convert(radian) <<std::endl;
        fout_target << targX << " " << targY << " " << targTheta << std::endl;

        double leftWheelVel = p.wheel_velocities[0];
        double rightWheelVel = p.wheel_velocities[1];

        double leftRPM = wheelVeltoRPM(leftWheelVel);
        double rightRPM = wheelVeltoRPM(rightWheelVel);

        l1.moveVelocity(leftRPM);
        l2.moveVelocity(leftRPM);
        r1.moveVelocity(rightRPM);
        r2.moveVelocity(rightRPM);

        pros::delay(10);
    }
    l1.moveVelocity(0);
    l2.moveVelocity(0);
    r1.moveVelocity(0);
    r2.moveVelocity(0);

    fout_target.close();
    fout_actual.close();
}

std::vector<double> ramseteWheelVels(double targX, double targY, double targTheta, std::vector<double> wheelVels){
//    double b = 2.0;
//    double zeta = 0.7;
    double b = 10.0;
    double zeta = 0.7;

    double lv = wheelVels[0];
    double rv = wheelVels[1];

    double targV = (lv+rv)/2;
    double targOmega = (rv-lv)/ROBOT_WIDTH;

    double deltaX = targX - robotX.convert(meter);
    double deltaY = targY - robotY.convert(meter);
    double robotPos = robotTheta.convert(radian);
//    if(robotPos > 0){
//        while(robotPos > 0) robotPos -= 2*M_PI;
//        robotPos += 2*M_PI;
//    }else if(robotPos < 0){
//        while(robotPos < 0) robotPos += 2*M_PI;
//        robotPos -= 2*M_PI;
//    }
//    if(robotPos < 0) robotPos += 2*M_PI;
    if(targTheta > 0){
        while(targTheta > 0) targTheta -= 2*M_PI;
        targTheta += 2*M_PI;
    }else if(targTheta < 0){
        while(targTheta < 0) targTheta += 2*M_PI;
        targTheta -= 2*M_PI;
    }
    if(targTheta < 0) targTheta += 2*M_PI;

    double deltaT = 1<<30;
    for(int i = -10; i <= 10; i++){
        double robotPos = robotTheta.convert(radian) + i*2*M_PI;
        double turn = targTheta - robotPos;
        if(std::abs(turn) < std::abs(deltaT)){
            deltaT = turn;
        }
    }

//    double deltaT = targTheta - robotPos;

//    double deltaT = targTheta - robotTheta.convert(radian);

    if(std::abs(deltaT) < 0.000001) deltaT += 0.0001;

    double theta = robotTheta.convert(radian);

    double ex = std::cos(theta)*deltaX + std::sin(theta)*deltaY;
    double ey = -std::sin(theta)*deltaX + std::cos(theta)*deltaY;
    double et = deltaT;

    double k = 2*zeta*sqrt(targOmega*targOmega + b*targV*targV);

    double v = targV*std::cos(et)+k*ex;
    double omega = targOmega + k*et + b*targV*sin(et)*ey/et;

    std::vector<double> sol;
    sol.push_back(v - 0.5*omega*ROBOT_WIDTH);
    sol.push_back(v + 0.5*omega*ROBOT_WIDTH);
    return sol;
}

void followPathRamsete(std::vector<squiggles::ProfilePoint> path){

    int index = 0;
    while(true){
        std::ifstream fin("/usd/log_target" + std::to_string(index) + ".txt");
        if(!fin.good()) break;
        else index++;
    }

    std::ofstream fout_actual("/usd/log_actual" + std::to_string(index) + ".txt");
    std::ofstream fout_target("/usd/log_target" + std::to_string(index) + ".txt");

    for(squiggles::ProfilePoint p : path){
        double targX = p.vector.pose.x;
        double targY = p.vector.pose.y;
        double targTheta = p.vector.pose.yaw;

        fout_actual << robotX.convert(meter) << " " << robotY.convert(meter) << " " << robotTheta.convert(radian) <<std::endl;
        fout_target << targX << " " << targY << " " << targTheta << std::endl;

        auto wheelVels = ramseteWheelVels(targX, targY, targTheta, p.wheel_velocities);

        double leftWheelVel = wheelVels[0];
        double rightWheelVel = wheelVels[1];

        double leftRPM = wheelVeltoRPM(leftWheelVel);
        double rightRPM = wheelVeltoRPM(rightWheelVel);

        l1.moveVelocity(leftRPM);
        l2.moveVelocity(leftRPM);
        r1.moveVelocity(rightRPM);
        r2.moveVelocity(rightRPM);

        pros::delay(10);
    }
    l1.moveVelocity(0);
    l2.moveVelocity(0);
    r1.moveVelocity(0);
    r2.moveVelocity(0);

    fout_target.close();
    fout_actual.close();
}

/**
 * Drives robot to state
 * @param x target x position
 * @param y target y position
 * @param theta target heading
 * @param speed squiggles controller to use (1-slow, 2-med, 3-fast)
 * @param ramsete whether or not use use ramsete controller
 */
void driveToPose(QLength x, QLength y, QAngle theta, int speed, bool ramsete = false){
    squiggles::Pose curPose = makePose(robotX, robotY, robotTheta);
    squiggles::Pose targPose = makePose(x, y, theta);

    std::vector<squiggles::ProfilePoint> path;

    if(speed == 0){
        path = generatorSuperSlow.generate({curPose, targPose});
    }else if(speed == 1){
        path = generatorSlow.generate({curPose, targPose});
    }else if(speed == 2){
        path = generatorMed.generate({curPose, targPose});
    }else if(speed == 3){
        path = generatorFast.generate({curPose, targPose});
    }

    if(ramsete) followPathRamsete(path);
    else followPath(path);
}

void driveDist(QLength dist, int vel){
    chassis->setMaxVelocity(vel);
    chassis->moveDistance(dist);
}

void driveDistRamsete(QLength dist, int speed){
    squiggles::Pose curPose = makePose(robotX, robotY, robotTheta);

    QLength targetX = robotX + std::cos(robotTheta.convert(radian))*dist;
    QLength targetY = robotY + std::sin(robotTheta.convert(radian))*dist;

    squiggles::Pose targPose = makePose(targetX, targetY, robotTheta);

    std::vector<squiggles::ProfilePoint> path;

    if(speed == 0){
        path = generatorSuperSlow.generate({curPose, targPose});
    }else if(speed == 1){
        path = generatorSlow.generate({curPose, targPose});
    }else if(speed == 2){
        path = generatorMed.generate({curPose, targPose});
    }else if(speed == 3){
        path = generatorFast.generate({curPose, targPose});
    }

    followPathRamsete(path);
}

void turnAngle(QAngle angle, int vel){
    chassis->setMaxVelocity(vel);
    chassis->turnAngle(-angle);
}

void turnToAngle(QAngle angle, int vel){
    double angleRad = angle.convert(radian);
    if(angleRad > 0){
        while(angleRad > 0){
            angleRad -= 2*M_PI;
        }
        angleRad += 2*M_PI;
    }else if(angleRad < 0){
        while(angleRad < 0){
            angleRad += 2*M_PI;
        }
        angleRad -= 2*M_PI;
    }
    if(angleRad < 0) angleRad += 2*M_PI;

    double minTurn = 1<<30;
    for(int i = -10; i <= 10; i++){
        double robotPos = robotTheta.convert(radian) + i*2*M_PI;
        double turn = angleRad - robotPos;
        if(std::abs(turn) < std::abs(minTurn)){
            minTurn = turn;
        }
    }

    double robotPos = robotTheta.convert(radian);
    if(robotPos > 0){
        while(robotPos > 0) robotPos -= 2*M_PI;
        robotPos += 2*M_PI;
    }else if(robotPos < 0){
        while(robotPos < 0) robotPos += 2*M_PI;
        robotPos -= 2*M_PI;
    }
    if(robotPos < 0) robotPos += 2*M_PI;

    turnAngle((angleRad - robotPos)*radian, vel);
    turnAngle(minTurn*radian, vel);
}

namespace Shooter{

    double targetVel = 0;
    double gain = 0.00001;
    double output = 0;
    double prevError = 0;
    double tbh = 0;

    double signum(double a){
        return (a>0) ? 1.0 : -1.0;
    }

    void setSpeed(double vel){
        targetVel = vel;
        output = 1*vel/3000.0;
        tbh = 1*vel/3000.0;
    }

    void shoot(){
        indexer.moveAbsolute(58, 600);
        int wait = 0;
        while(indexer.getPosition() < 57 && wait < 250){
            pros::delay(10);
            wait += 10;
        }
        indexer.moveAbsolute(-3, 600);
        wait = 0;
        while(indexer.getPosition() > 0 && wait < 250){
            pros::delay(10);
            wait += 10;
        }
    }

    void speedControl(void* param){
        while(true){
            //run flywheel using TBH controller
            double error = targetVel - (flywheel1.getActualVelocity() * 5);
            output += gain * error;
            output = std::min(output, 1.0);
            output = std::max(output, 0.0);
            if (signum(error) != signum(prevError)) {
                output = 0.5 * (output + tbh);
                tbh = output;
                prevError = error;
            }
            if (targetVel < 1) output = 0;
            flywheel1.moveVoltage(output * 12000);
            flywheel2.moveVoltage(output * 12000);

            pros::delay(10);
        }

    }

    void op_control(void* param) {
        double output = 0;
        double prevError = 0;
        double tbh = 0;

        bool indexerPrev = false;
        bool indexerOn = false;

        while (true) {
            //set target vel, tbh, and output
            if (partner.getDigital(ControllerDigital::up)) {
                targetVel = 3000;
                tbh = 1;
                output = 1;
            } else if (partner.getDigital(ControllerDigital::right)) {
                targetVel = 2400;
                tbh = 0.83;
                output = 0.83;
            } else if (partner.getDigital(ControllerDigital::down)) {
                targetVel = 2200;
                tbh = 0.77;
                output = 0.77;
            } else if (partner.getDigital(ControllerDigital::B)) {
                targetVel = 0;
                tbh = 0;
                output = 0;
            }

            //run flywheel using TBH controller
            double error = targetVel - (flywheel1.getActualVelocity() * 5);
            output += gain * error;
            output = std::min(output, 1.0);
            output = std::max(output, 0.0);
            if (signum(error) != signum(prevError)) {
                output = 0.5 * (output + tbh);
                tbh = output;
                prevError = error;
            }
            if (targetVel < 1) output = 0;
            flywheel1.moveVoltage(output * 12000);
            flywheel2.moveVoltage(output * 12000);

            //control indexer
            if(partner.getDigital(ControllerDigital::R1) && !indexerPrev && targetVel > 0.001){
                indexerOn = true;
            }else if(partner.getDigital(ControllerDigital::R2)){
                indexerOn = false;
            }
            indexerPrev = partner.getDigital(ControllerDigital::R1);

            if(indexer.getPosition() > 54){
                indexerOn = false;
            }

            indexer.moveAbsolute(indexerOn ? 55:-3, 100);

            pros::delay(10);
        }
    }
}

void scoreRoller(int maxWait){
    intake.moveVelocity(150);
    int wait = 0;
    while(wait < maxWait && opticalSensor.get_hue() > 120){
        wait += 10;
        pros::delay(10);
    }
    intake.moveVelocity(0);
}

void initialize() {
	pros::lcd::initialize();

    odomL.reset();
    odomR.reset();

    pros::delay(200);

    lastLeft = odomL.get() * (M_PI/180.0);
    lastRight = odomR.get() * (M_PI/180.0);

    robotX = 0_m;
    robotY = 0_m;
    robotTheta = 0_rad;

    gyro.reset(true);
    pros::delay(2000);
    pros::lcd::set_text(2, "IMU calibrated");
}

void disabled() {}

void competition_initialize() {}

void prog() {

//    Shooter::setSpeed(2400);

    //get first roller
    driveDist(1.5_in, 100);
//    intake.moveVelocity(200);
//    pros::delay(350);
//    intake.moveVelocity(0);
    scoreRoller(1000);
    intake.moveVelocity(-100);
    driveDist(-8_in, 200);

    //get second roller
    turnToAngle(-100_deg, 300);
//    turnAngle(-100_deg, 300);
    intake.moveVelocity(600);
    driveToPose(-14_in, -27_in, -90_deg, 2, true);
    intake.moveVelocity(0);
    driveDist(1.5_in, 100);
//    intake.moveVelocity(200);
//    pros::delay(500);
//    intake.moveVelocity(0);
    scoreRoller(1000);
    intake.moveVelocity(-100);
    driveDist(-12_in, 200);
    intake.moveVelocity(600);

    //shoot
    turnToAngle(70_deg, 300);
    Shooter::setSpeed(2200);
    driveToPose(1_in, 39.5_in, 90_deg, 2, true);
//    turnToAngle(100_deg, 200);
//    driveDist(-5_in, 100);
//    turnToAngle(90_deg, 200);
//    driveDist(6_in, 100);
    turnToAngle(98_deg, 200);
    for(int i = 0; i < 10; i++){
        Shooter::shoot();
        pros::delay(700);
    }
    Shooter::shoot();
    turnToAngle(90_deg, 200);
    driveDist(-5_in, 200);
    turnToAngle(180_deg, 300);
    Shooter::setSpeed(2300);
    driveToPose(-53_in, 46_in, 135_deg, 2, true);
    driveDistRamsete(34_in, 2);
    turnToAngle(10_deg, 200);
    driveDistRamsete(24_in, 2);
//    driveDist(24_in, 300);
    for(int i = 0; i < 2; i++){
        Shooter::shoot();
        pros::delay(500);
    }
    Shooter::shoot();

    intake.moveVelocity(600);
    turnToAngle(180_deg, 300);
    driveToPose(-105_in, 80_in, 180_deg, 2, true);
    turnToAngle(90_deg, 300);
    driveDist(8_in, 200);
    intake.moveVelocity(0);
//    driveDistRamsete(7_in, 2);
    driveDist(6.4_in, 200);
    scoreRoller(1000);
    intake.moveVelocity(-100);

    driveDist(-27_in, 200);
    intake.moveVelocity(600);
    turnToAngle(180_deg, 300);
    intake.moveVelocity(0);
    driveDist(17.5_in, 200);
    scoreRoller(1000);
    intake.moveVelocity(-50);
    driveDist(-24_in, 400);
    turnToAngle(-45_deg, 400);
    driveDist(-20_in, 400);
    stringLauncher.set_value(1);

}

void red(){
    driveDist(1.5_in, 100);
    intake.moveAbsolute(-400, 100);
    pros::delay(5000);
    driveDist(-1.5_in, 200);
}

void autonomous() {
    pros::Task odom(odometry);
    pros::Task flywheel(Shooter::speedControl);
    opticalSensor.set_led_pwm(100);

    l1.setBrakeMode(AbstractMotor::brakeMode::hold);
    l2.setBrakeMode(AbstractMotor::brakeMode::hold);
    r1.setBrakeMode(AbstractMotor::brakeMode::hold);
    r2.setBrakeMode(AbstractMotor::brakeMode::hold);
    intake.setBrakeMode(AbstractMotor::brakeMode::hold);

    red();
//    prog();
//    if(SKILLS) prog();
//    else auton();

    l1.setBrakeMode(AbstractMotor::brakeMode::coast);
    l2.setBrakeMode(AbstractMotor::brakeMode::coast);
    r1.setBrakeMode(AbstractMotor::brakeMode::coast);
    r2.setBrakeMode(AbstractMotor::brakeMode::coast);
    intake.setBrakeMode(AbstractMotor::brakeMode::coast);
    flywheel.suspend();
}

double curve(double input, double exp){
    double sign = (input < 0) ? (-1.0) : (1.0);
    return sign * std::abs(std::pow(input, exp));
}

void opcontrol() {

    if(SKILLS){
        autonomous();
    }else{

        int driveSpeed = 600;
        pros::Task flywheel(Shooter::speedControl);

        bool indexerOn = false;
        bool indexerPrev = false;

        while (true) {

            if(master.getDigital(okapi::ControllerDigital::Y)){
                flywheel.suspend();
                autonomous();
            }

            /**Drive**/
            if(master.getDigital(ControllerDigital::up)){
                driveSpeed = 600;
            }else if(master.getDigital(ControllerDigital::right)){
                driveSpeed = 450;
            }else if(master.getDigital(ControllerDigital::down)){
                driveSpeed = 300;
            }

            double lv = curve(master.getAnalog(ControllerAnalog::leftY), 2);
            double rv = curve(master.getAnalog(ControllerAnalog::rightY), 2);

            l1.moveVelocity(lv*driveSpeed);
            l2.moveVelocity(lv*driveSpeed);
            r1.moveVelocity(rv*driveSpeed);
            r2.moveVelocity(rv*driveSpeed);

            /**Shooter**/
            if(partner.getDigital(ControllerDigital::up)){
                Shooter::setSpeed(3000);
            }else if(partner.getDigital(ControllerDigital::right)){
                Shooter::setSpeed(2400);
            }else if(partner.getDigital(ControllerDigital::down)){
                Shooter::setSpeed(2200);
            }else if(partner.getDigital(ControllerDigital::B)){
                Shooter::setSpeed(0);
            }

            if(partner.getDigital(ControllerDigital::R1) && !indexerPrev && Shooter::targetVel > 1){
                indexerOn = true;
            }else if(partner.getDigital(ControllerDigital::R2)){
                indexerOn = false;
            }
            indexerPrev = partner.getDigital(ControllerDigital::R1);

            if(indexer.getPosition() > 54){
                indexerOn = false;
            }

            indexer.moveAbsolute(indexerOn ? 55:-3, 100);

            /**Intake**/
            if(master.getDigital(ControllerDigital::L1)){
                intake.moveVelocity(600);
            }else if(master.getDigital(ControllerDigital::L2)){
                intake.moveVelocity(-600);
            }else if(master.getDigital(okapi::ControllerDigital::R1)){
                intake.moveVelocity(200);
            }else if(master.getDigital(okapi::ControllerDigital::R2)){
                intake.moveVelocity(-200);
            }else{
                intake.moveVelocity(0);
            }

            /**String launcher**/
            if(master.getDigital(ControllerDigital::left) &&
               partner.getDigital(okapi::ControllerDigital::left)){
                pros::lcd::set_text(3, "string launch");
                stringLauncher.set_value(1);
            }

            pros::delay(10);
        }
    }
}
