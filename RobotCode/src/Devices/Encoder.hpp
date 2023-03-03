#ifndef INC_3946X_2023_ENCODER_HPP
#define INC_3946X_2023_ENCODER_HPP

#include "../Units/Units.hpp"
#include "../../include/api.h"

class Encoder : public pros::Rotation {
public:
    using pros::Rotation::Rotation;

    radian_t getPosition(){
        double ret = get_position()/100.0;
        return radian_t(ret * (M_PI/180));
    }

};


#endif //INC_3946X_2023_ENCODER_HPP
