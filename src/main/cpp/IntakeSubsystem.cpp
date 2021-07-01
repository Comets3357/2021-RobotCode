#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "IntakeSubsystem.h"



void IntakeSubsystem::Init(){

    rollers.RestoreFactoryDefaults();
    rollers.SetInverted(true);
    rollers.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    rollers.SetSmartCurrentLimit(45);
    setIntakeRollers(0);
    setPiston(false);
    setIntakeRollers(0);

}

void IntakeSubsystem::Periodic(RobotData &robotData, DiagnosticsData &diagnosticsData){
    //decide if in manual mode or auto mode
    if(robotData.manualMode){
        manualMode(robotData);
    } else {
        semiAutoMode(robotData);
    }
    //setPiston(true);

    updateDiagnostics(diagnosticsData);
}


void IntakeSubsystem::semiAutoMode(RobotData &robotData){

    shootPOV = robotData.sDPad;

    double pow = -0.4;

    if((robotData.Rdrive+robotData.Ldrive)/2 > 0.7){
        pow = -0.8;
    }else if((robotData.Rdrive+robotData.Ldrive)/2 > 0.6){
        pow = -0.7;
    }else if((robotData.Rdrive+robotData.Ldrive)/2 > 0.5){
        pow = -0.6;
    }else if((robotData.Rdrive+robotData.Ldrive)/2 > 0.4){
        pow = -0.5;
    }else{
        pow = -0.4;
    }

    frc::SmartDashboard::PutNumber("speed", pow);


    //if in shooting mode then you want manual control of the intake
    if (shootPOV == robotData.shootingBtn){
        //setPiston(true);
        manualMode(robotData);

    } else {
        //Intake balls
        if(robotData.sRTrigger){
            if(!getPiston()){ //if the piston is up put it down
                setPiston(true);
            }
            setIntakeRollers(pow);
        }else if(robotData.sLTrigger){
            setIntakeRollers(robotData.sLTrigger);
        }else{
            if(getPiston()){ //if the piston is down put it up
                setPiston(false);
            }
            setIntakeRollers(0);
        }

        
    

    }

    
}

void IntakeSubsystem::manualMode(RobotData &robotData){

    double pow = -0.4;

    if((robotData.Rdrive+robotData.Ldrive)/2 > 0.7){
        pow = -0.8;
    }else if((robotData.Rdrive+robotData.Ldrive)/2 > 0.6){
        pow = -0.7;
    }else if((robotData.Rdrive+robotData.Ldrive)/2 > 0.5){
        pow = -0.6;
    }else if((robotData.Rdrive+robotData.Ldrive)/2 > 0.4){
        pow = -0.5;
    }


    //if the shift is pressed reverse the intake roller
    if(robotData.shift){
        if(robotData.sRTrigger){
            setIntakeRollers(robotData.sRTrigger*-pow);
        } else {
            setIntakeRollers(0);
        }

    //else run the intake roller
    }else{
        setIntakeRollers(robotData.sRTrigger);

        if(robotData.sRBumper){ //if the piston is extended take it in, if its in take it out
            setPiston(!getPiston());
        }
    }
    
   

}

/**
 * @param direction true is retracted and false is extended
 */

void IntakeSubsystem::setPiston(bool direction){  
    if (direction){
        solenoidOne.Set(true);
    } else {
        solenoidOne.Set(false);
    }
    
}

bool IntakeSubsystem::getPiston(){
    if(solenoidOne.Get() == true){
        return true;
    }else{
        return false;
    }
}

void IntakeSubsystem::setIntakeRollers(double power){
    rollers.Set(power);
}

void IntakeSubsystem::Disabled(){
    setIntakeRollers(0);
    setPiston(false);
}



void IntakeSubsystem::updateDiagnostics(DiagnosticsData &diagnosticsData)
{
    /**
     * solenoidOne
     * intake rollers 32
     */
    diagnosticsData.solenoidOneValue = solenoidOne.Get();

    diagnosticsData.mControlCurrents.at(32) = rollers.GetOutputCurrent();
    diagnosticsData.mControlVoltages.at(32) = rollers.GetBusVoltage();
    diagnosticsData.mControlTemps.at(32) = rollers.GetMotorTemperature();

    diagnosticsData.mControlPositions.at(32) = rollersEncoder.GetPosition();
    diagnosticsData.mControlVelocities.at(32) = rollersEncoder.GetVelocity();

    diagnosticsData.mControlFaults.at(32) = rollers.GetFaults();
}