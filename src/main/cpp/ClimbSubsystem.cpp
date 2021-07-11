#include <frc/smartdashboard/SmartDashboard.h>

#include "Robot.h"
#include "ClimbSubsystem.h"

/** 
 * Controls:
 * 1 - Control panel button
 *      - Use to control the left arm actuator piston
 * 2 - Climb initiation button
 *      - Use to initialize the climb 
 *      - Make this toggleable
 * 3 - Climb start button
 *      - Begin the climb
 *      - Start automated sequence for climb leveling
**/

/**
 * Libraries:
 * Gyro
 * Pistons
**/

/** 
 * Variables:
 * Boolean lock right
 * Boolean lock left
 * Boolean right arm climb actuator
 * Boolean left arm climb actuator
 * Boolean level 
 * 
 * Double right arm motor
 * Double left arm motor
**/ 

/**
 * Components:
 * x4 pistons
 *  - One each for controlling the gear lock.
 *  - One each for controlling the arm initiation. 
 * x2 motors
 *  - One each for extending and retracting the climb arms.
 * x1 gyro
 *  - Provide level information for the climb. 
**/

/**
 * Pseudo:
 * Start of match
 *  - Zero gyro
 *  - Zero encoders
 * 
 * Control panel
 *  - Move up left arm
 * 
 * Start of climb
 *  When initiation button pressed
 *  - Actuate lock pistons (pull out of gear)
 *  - Actuate both climb actuator pistons to max length
 *  - Initialize motors to extend climb arms semi-slowly
 *  - Once arms are at desired extension
 *      - Stop motors
 *  When start button pressed
 *      - Compare currents of both motors (check to see if similar)
 *      - Start detecting increase in current of both motors
 *          - If left is higher, stop left and continue right
 *          - If right is higher, stop right and continur left
 *          - Continue whichever motors until both show higher current
 *      - Initiate both motors to begin climb
 *      - If climb is already level
 *          - Just keep climbing on both arms until climb height is reached
 *      - If climb is not level
 *          - Get gyro data and change climb motors actuations based on level
 *          - Make sure that climb doesnt retract or extend past limits
 *          - If limit is reached, start adjusting in opposite direction
 *      - If climb is complete and level
 *          - Set lock pistons to default position (push into gear)
 *  Climb deactivation
 *      - Actuate lock pistons (pull out of gear)
 *      - Start motors to reset climb until limit switches/ prox sensors are intiated
 *      - Set lock pistons to default position (push into gear)
**/

void ClimbSubsystem::RobotInit(){
    climbArmLPos.SetPosition(0);
    climbArmRPos.SetPosition(0);

    climbArmR.RestoreFactoryDefaults();
    climbArmR.SetInverted(true);
    climbArmR.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    climbArmR.SetSmartCurrentLimit(45);
    climbArmR.Set(0);
    
    climbArmL.RestoreFactoryDefaults();
    climbArmL.SetInverted(false);
    climbArmL.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    climbArmL.SetSmartCurrentLimit(45);
    climbArmL.Set(0);
    
    // imu.Reset();
}

void ClimbSubsystem::Periodic(RobotData &robotData){


    frc::SmartDashboard::PutNumber("left",  climbArmLLimit.Get());
    frc::SmartDashboard::PutNumber("right",  climbArmRLimit.Get());

    frc::SmartDashboard::PutNumber("climb mode",  robotData.climbMode);

    //manualMode(robotData);

    /* if (robotData.sBBtn) {
        solenoidArm.Set(solenoidArm.kForward);
    }
    if (robotData.sABtn) {
        solenoidArm.Set(solenoidArm.kReverse);
    } */

    if (!robotData.climbZeroing){
        
        if(robotData.manualMode){
            if (robotData.climbMode){
                startingPhase = 0;
                climbArmL.Set(0);
                climbArmR.Set(0);
                manualMode(robotData);
            }
        } else {
            if (robotData.climbMode){
                semiAutoMode(robotData);
            }
        }
    } else {
        zeroLoop += 1;
        if (robotData.sXBtn){
            climbArmLPos.SetPosition(0);
            climbArmRPos.SetPosition(0);
            climbArmL.Set(0);
            climbArmR.Set(0);
            solenoidLockL.Set(false);
            solenoidLockR.Set(false);
            robotData.climbZeroing = false;
        } else if (zeroLoop > 75 && robotData.autonEnabled){
            climbArmL.Set(0);
            climbArmR.Set(0);
        } else {
            if (!climbArmLLimit.Get()) {
                climbArmL.Set(0);
            }
            else {
                climbArmL.Set(0.1);
            }
            if (climbArmRLimit.Get()) {
                climbArmR.Set(0);
            } else {
                climbArmR.Set(0.1);
            }
            if (!climbArmLLimit.Get() && climbArmRLimit.Get()) {
                climbArmLPos.SetPosition(0);
                climbArmRPos.SetPosition(0);
                solenoidLockL.Set(false);
                solenoidLockR.Set(false);
                solenoidArm.Set(solenoidArm.kForward);
                robotData.climbZeroing = false;
            }
        }
        
        
    }
}

void ClimbSubsystem::manualMode(RobotData &robotData){
    if (robotData.sRBumper){
        if (solenoidLockL.Get() && lockToggle == true) {
            lockToggle = false;
            solenoidLockL.Set(false);
            solenoidLockR.Set(false);
        } else if (!solenoidLockL.Get() && lockToggle == true) {
            lockToggle = false;
            solenoidLockL.Set(true);

            solenoidLockR.Set(true);
        }
    } else {

        lockToggle = true;
    }

    if (robotData.sLYStick > 0.1 || robotData.sLYStick < -0.1) {
        climbArmL.Set(robotData.sLYStick*0.3);
        // 
        
    } else {
        climbArmL.Set(0);
    }
    if (robotData.sRYStick > 0.1 || robotData.sRYStick < -0.1) {
        climbArmR.Set(robotData.sRYStick*0.3);
        // if (climbArmRPos.GetPosition() > 5) {
        //     climbArmR.Set(0);
        // }
    } else {
        climbArmR.Set(0);
    }

    if (climbArmLPos.GetPosition() < -10 || climbArmRPos.GetPosition() < -10) {
        solenoidArm.Set(solenoidArm.kReverse);
    } else {
        solenoidArm.Set(solenoidArm.kForward);
    }
}

void ClimbSubsystem::semiAutoMode(RobotData &robotData){

    // Climb inititation

    if(robotData.sRBumper){
        if (!initiationRunning){
            initiationRunning = true;
        }
    }


    if (initiationRunning && !initiated) {
        solenoidLockR.Set(true);
        solenoidLockL.Set(true);
        timer += 1;
        climbArmR.Set(0.2);
        climbArmL.Set(0.2);
        if (timer > 20){
            if (climbArmRPos.GetPosition() > -80 || climbArmLPos.GetPosition() > -80) {//i dont know the exact numbers yet
                solenoidArm.Set(solenoidArm.kReverse);
                if (climbArmRPos.GetPosition() > -80) {
                    climbArmR.Set(-0.3);
                } else {
                    climbArmR.Set(0);
                }
                if (climbArmLPos.GetPosition() > -80) {
                    climbArmL.Set(-0.3);
                } else {
                    climbArmL.Set(0);
                }
            } else {
                solenoidLockR.Set(false);
                solenoidLockL.Set(false);
                climbArmR.Set(0);
                climbArmL.Set(0);
                initiated = true;
                initiationRunning = false;
            }
        }
    } else if (initiationRunning && initiated) {
        if (climbArmRPos.GetPosition() < 0 || climbArmLPos.GetPosition() < 0) {
            if (climbArmLPos.GetPosition() < 0){
                climbArmL.Set(0.3);
            } else {
                climbArmL.Set(0);
            }
            if (climbArmRPos.GetPosition() < 0){
                climbArmR.Set(0.3);
            } else {
                climbArmR.Set(0);
            }
        } else {
            climbArmR.Set(0);
            climbArmL.Set(0);
            solenoidArm.Set(solenoidArm.kForward);
            initiated = false;
            initiationRunning = false;
            timer = 0;
        }
    }

    // Climb start

    if (robotData.sYBtn && startingPhase == 0) {
        
        solenoidLockL.Set(false);
        solenoidLockR.Set(false);
        startingPhase = 1;
    }

    frc::SmartDashboard::PutNumber("RVol",  climbArmR.GetOutputCurrent());
    frc::SmartDashboard::PutNumber("LVol",  climbArmL.GetOutputCurrent());

    // frc::SmartDashboard::PutNumber("getX",  robotData.robotTiltAngle);
    // frc::SmartDashboard::PutNumber("getY",  imu.GetGyroAngleY());

   
    if (startingPhase == 1) {
        if (climbArmLPos.GetPosition() < -30 || climbArmRPos.GetPosition() < -30){
            if (robotData.robotTiltAngle < 4 && robotData.robotTiltAngle > -4){//dont know gyro axis or offset
                climbArmR.Set(0.3);
                climbArmL.Set(0.3);
            } else if (robotData.robotTiltAngle > 4){
                climbArmR.Set(0);
                climbArmL.Set(0.3);
            } else if (robotData.robotTiltAngle < -4){
                climbArmR.Set(0.3);
                climbArmL.Set(0);
            }
        } else{
            climbArmR.Set(0);
            climbArmL.Set(0);
            startingPhase = 2;
        }
    } else if (startingPhase == 2){
        if (robotData.robotTiltAngle < 4 && robotData.robotTiltAngle > -4){
            climbArmR.Set(0);
            climbArmL.Set(0);
        } else if (robotData.robotTiltAngle > 4){
            startingPhase = 3;
            timer = 0;
        } else if (robotData.robotTiltAngle < -4){
            startingPhase = 3;
            timer = 0;
        }
    } else if (startingPhase == 3){

        solenoidLockR.Set(true);
        climbArmR.Set(0.2);
        timer += 1;
        if (timer > 10) {
            climbArmR.Set(0);
            climbArmL.Set(0);
            if (robotData.robotTiltAngle > 4){
                climbArmR.Set(-0.2);//i dont know the exact numbers yet
            } else if (robotData.robotTiltAngle < -4){
                climbArmR.Set(0.5);//i dont know the exact numbers yet
            } else {
                startingPhase = 2;
                climbArmR.Set(0);//i dont know the exact numbers yet
                climbArmL.Set(0);//i dont know the exact numbers yet
                solenoidLockR.Set(false);
            }
        }
    }

    

}