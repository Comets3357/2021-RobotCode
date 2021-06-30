#include <frc/smartdashboard/SmartDashboard.h>

#include "Auton.h"
#include "Robot.h"

void Auton::Init()
{
    timer.Reset();
    timer.Start();
}

void Auton::Periodic(AutonSelect autonSelect, RobotData &robotData)
{
    frc::SmartDashboard::PutNumber("autonSelect", autonSelect);
    // robotData.autonSelect = autonSelect;    // currently not using the robotData.autonSelect

    updateTimer(robotData);
    // frc::SmartDashboard::PutNumber("timer", robotData.seconds);
    frc::SmartDashboard::PutNumber("autonStep", robotData.autonStep);
    frc::SmartDashboard::PutNumber("desired angle", robotData.desiredAngleDiff);
    frc::SmartDashboard::PutNumber("initial angle", robotData.initialAngle);


    switch (autonSelect)
    {

    case autonSelect_potato:
        robotData.driveMode = driveMode_potato;
        break;

    case autonSelect_exitInitLine:

        switch (robotData.autonStep)
        {
        case 0:
            robotData.desiredDBDist = 30;
            robotData.driveMode = driveMode_initDriveStraight;
            break;
        case 1:
            robotData.driveMode = driveMode_driveStraight;
            break;
        default:
            robotData.driveMode = driveMode_potato;
            break;
        }
        break;
    
    case autonSelect_shootAndDrive:

        switch (robotData.autonStep)
        {
        case 0:
            robotData.manualMode = false;
            startDelay(4.5, robotData);
            robotData.sDPad = 90;
            robotData.driveMode = driveMode_potato;
            robotData.autonStep++;
            break;
        case 1:
            //shoot and check delay
            checkDelay(robotData);
            robotData.driveMode = driveMode_potato;
            break;
        case 2:
            //turn shooter off before driving away
            robotData.sDPad = -1;
            robotData.driveMode = driveMode_potato;
            robotData.autonStep++;
            break;
        case 3:
            robotData.desiredDBDist = 30;
            robotData.driveMode = driveMode_initDriveStraight;
            break;
        case 4:
            robotData.driveMode = driveMode_driveStraight;
            break;
        default:
            robotData.driveMode = driveMode_potato;
            break;
        }
        break;

    // has not been tested with intake control
    // like trench run but bad
    case autonSelect_shootAndCollectBalls:
        switch (robotData.autonStep)
        {
        case 0:
            startDelay(7, robotData);
            robotData.sDPad = 90;
            robotData.autonStep++;
            break;
        case 1:
            //shoot and check delay
            checkDelay(robotData);
            break;
        case 2:
            //turn shooter off before driving away
            robotData.sDPad = -1;
            robotData.autonStep++;
            break;
        case 3:
            robotData.desiredDBDist = 15;
            robotData.driveMode = driveMode_initDriveStraight;
            break;
        case 4:
            //bring intake down and indexer into intaking mode while driving
            robotData.sABtn = true;
            robotData.driveMode = driveMode_driveStraight;
            break;
        default:
            robotData.sABtn = false;
            robotData.driveMode = driveMode_potato;
            break;
        }
        break;

    //has not been tested with balls
    case autonSelect_trenchRun:
        switch (robotData.autonStep)
        {
        case 0:
            //spin up flywheel
            robotData.sDPad = -1;
            robotData.sBBtn = true;
            //intake and indexer in intaking mode 
            robotData.sABtn = true;
            //start delay to give time to intake
            startDelay(1.5, robotData);
            //init drive forward
            robotData.desiredDBDist = 50;
            robotData.driveMode = driveMode_initDriveStraight;
            break;
        case 1:
            //wait for intake to come down
            robotData.driveMode = driveMode_potato;
            checkDelay(robotData);
            break;
        case 2:
            //drive forward
            robotData.driveMode = driveMode_driveStraight;
            //spin up flywheel
            robotData.sDPad = -1;
            robotData.sBBtn = true;
            //intake and indexer in intaking mode 
            robotData.sABtn = true;
            //move on when two balls have been collected (in encoder values)
            break;
        case 3:
            //start delay and go into shooting mode
            startDelay(4.5, robotData);
            robotData.sDPad = 90;
            robotData.sBBtn = false;
            //intake in shooting mode/not collecting balls
            robotData.sABtn = false; 
            robotData.driveMode = driveMode_potato;
            robotData.autonStep++;
            break;
        case 4:
            //check delay, shoot
            checkDelay(robotData);
            robotData.sDPad = 90;
            robotData.driveMode = driveMode_potato;
            break;
        case 5:
            //turn shooting off, drive to collect more balls
            robotData.sDPad = -1;
            robotData.sABtn = true;
            //wait for intake to come down 
            startDelay(1.5, robotData);
            robotData.desiredDBDist = 40;
            robotData.driveMode = driveMode_initDriveStraight;
            break;
        case 6:
            robotData.driveMode = driveMode_potato;
            checkDelay(robotData);
            break;
        case 7:
            robotData.driveMode = driveMode_driveStraight;
            break;
        
        default:
            // robotData.sABtn = false;
            robotData.driveMode = driveMode_potato;
            break;
        }
        break; 

    case autonSelect_stealBallsAndShoot:
        switch (robotData.autonStep)
        {
        case 0:
            //spin up flywheel
            robotData.sDPad = -1;
            robotData.sBBtn = true;
            //intake and indexer in intaking mode 
            robotData.sABtn = true;
            //start delay to give time to intake
            startDelay(1.5, robotData);
            //init drive forward
            robotData.desiredDBDist = 40;
            robotData.driveMode = driveMode_initDriveStraight;
            break;
        case 1:
            //wait for intake to come down
            robotData.driveMode = driveMode_potato;
            checkDelay(robotData);
            break;
        case 2:
            //drive forward to grab two balls
            robotData.driveMode = driveMode_driveStraight;
            //spin up flywheel
            robotData.sDPad = -1;
            robotData.sBBtn = true;
            //intake and indexer in intaking mode 
            robotData.sABtn = true;
            //move on when two balls have been collected (in encoder values)
            break;
        case 3:
            robotData.sABtn = false;
            // turn
            robotData.desiredAngleDiff = 45;
            robotData.arcRadius = -1;
            robotData.driveMode = driveMode_initArc;
            break;
        case 4:
            robotData.driveMode = driveMode_arc;
            break;
        case 5:
            // drive backwards
            robotData.desiredDBDist = -40;
            robotData.driveMode = driveMode_initDriveStraight;
            break;
        case 6:
            robotData.driveMode = driveMode_driveStraight;;
            break;
        case 7:
            //start delay and go into shooting mode
            startDelay(4.5, robotData);
            robotData.sDPad = 90;
            robotData.sBBtn = false;
            //intake in shooting mode/not collecting balls
            robotData.sABtn = false; 
            robotData.driveMode = driveMode_potato;
            robotData.autonStep++;
            break;
        case 8:
            //check delay, shoot
            checkDelay(robotData);
            robotData.sDPad = 90;
            robotData.driveMode = driveMode_potato;
            break;       
        
        default:
            // robotData.sABtn = false;
            robotData.driveMode = driveMode_potato;
            break;
        }
        break;

    // used for testing
    case autonSelect_goofy:
        switch(robotData.autonStep) {
        case 0:
            robotData.desiredAngleDiff = 90;
            robotData.arcRadius = -1;
            robotData.driveMode = driveMode_initArc;
            break;
        case 1:
            robotData.driveMode = driveMode_arc;
            break;
        /* case 0:
            robotData.desiredDBDist = 30;
            robotData.driveMode = driveMode_initDriveStraight;
            break;
        case 1:
            robotData.driveMode = driveMode_driveStraight;
            break; */
        default:
            robotData.driveMode = driveMode_potato;
            break;
        }
        break;

    default:
        robotData.driveMode = driveMode_potato;
        break;
    }

    // only for updating diagnostics
    robotData.autonSelect = autonSelect;
}


// updates the struct with the amount of time that has passed since the Timer object started keeping track of time
void Auton::updateTimer(RobotData &robotData)
{
    robotData.seconds = timer.Get();
}

// starts the delay function by recording when the delay should end
// duration is how long you want the delay to be in seconds
void Auton::startDelay(double duration, RobotData &robotData)
{
    delayFinal = robotData.seconds + duration;
}

// chekcs if the delay you started is over yet
// increments autonStep once your delay is done
void Auton::checkDelay(RobotData &robotData)
{
    if (robotData.seconds > delayFinal)
    {
        //wpi::outs() << "moving on";
        robotData.autonStep++;
    }
    
}