#ifndef INC_3946X_2023_ENCODER_HPP
#define INC_3946X_2023_ENCODER_HPP

#include "../Units/Units.hpp"
#include "../../include/api.h"

class Encoder : public pros::Rotation {
public:
    using pros::Rotation::Rotation;

    radian_t getPosition(){
        return radian_t(convert<degree, radian>(((double)get_position())/100.0));
    }
};


#endif //INC_3946X_2023_ENCODER_HPP
