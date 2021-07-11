#pragma once

#include "RobotData.h"
#include "CANifier.h"

class LEDSubsystem {

    public:

        void RobotInit();
        void Periodic(RobotData &robotData);
        void DisabledInit();

    private:

        CANifier

};