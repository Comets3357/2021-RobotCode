//Tells the compiler to not include the header files multiple times
#pragma once

//Header files from FRC
#include <frc/TimedRobot.h>
#include <frc/Joystick.h>
#include <rev/CANSparkMax.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SendableChooser.h>


//Header files from us
#include "Auton.h"
#include "IntakeSubsystem.h"
#include "DriveSubsystem.h"
#include "IndexerSubsystem.h"
#include "ShooterSubsystem.h"
#include "Controller.h"
#include "LimelightSubsystem.h"
#include "Auton.h"
#include "Diagnostics.h"
#include "ControlpanelSubsystem.h"
#include "ClimbSubsystem.h"
#include "LEDSubsystem.h"

#include "RobotData.h"




//Robot class inherits from TimedRobot
class Robot : public frc::TimedRobot {

  public:
      void RobotInit() override;
      void RobotPeriodic() override;
      void AutonomousInit() override;
      void AutonomousPeriodic() override;
      void TeleopPeriodic() override;
      void DisabledInit() override;
      void TestInit() override;
      void TestPeriodic() override;

  private:

    
    
      DriveSubsystem db{};
      ShooterSubsystem shooter{};
      IndexerSubsystem indexer{};
      IntakeSubsystem intake{};
      LimelightSubsystem limelight{};
      ControlpanelSubsystem controlpanel{};
      ClimbSubsystem climb{};
      LEDSubsystem LEDS{};

      Controller control{};
      Auton auton{};
      Diagnostics diagnostics{};
      DiagnosticsData diagnosticsData{};
      RobotData robotData{};

      






};






