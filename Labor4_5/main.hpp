#ifndef MAIN_HPP_INCLUDED
#define MAIN_HPP_INCLUDED

#include "AMS_Robot.hpp"

using namespace AMS;
using namespace PlayerCc; // dtor, rtod, limit, normalize

void DriveRobot(AMS_Robot* robotp, double L1, double LK, double L2, int turndir);


#endif // MAIN_HPP_INCLUDED
