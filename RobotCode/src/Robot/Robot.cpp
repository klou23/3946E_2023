#include "Robot.hpp"

/**Controllers**/
Controller master(pros::E_CONTROLLER_MASTER);
Controller partner(pros::E_CONTROLLER_PARTNER);

/**Drive**/
pros::Motor fl(18, pros::E_MOTOR_GEARSET_06, true);
pros::Motor fr(6, pros::E_MOTOR_GEARSET_06, false);
pros::Motor bl(12, pros::E_MOTOR_GEARSET_06, true);
pros::Motor br(1, pros::E_MOTOR_GEARSET_06, false);
Encoder lEnc(21);
Encoder rEnc(21);
Encoder bEnc(21);

/**Shooter**/
pros::Motor flywheel1(8, pros::E_MOTOR_GEARSET_06, true);
pros::Motor flywheel2(9, pros::E_MOTOR_GEARSET_06, false);
pros::Motor indexer(13, pros::E_MOTOR_GEARSET_36, false);

/**Intake**/
pros::Motor intake(10, pros::E_MOTOR_GEARSET_18, false);

