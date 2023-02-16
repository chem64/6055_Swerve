#include "Robot.h"

void Robot::RobotInit() 
{
  ntBOSS = nt::NetworkTableInstance::GetDefault().GetTable("dashBOSS");
  ConfigMotors();
  frTurnPID.EnableContinuousInput(-180.0,180.0); //required for swerve
  flTurnPID.EnableContinuousInput(-180.0,180.0); //required for swerve
  rlTurnPID.EnableContinuousInput(-180.0,180.0); //required for swerve
  rrTurnPID.EnableContinuousInput(-180.0,180.0); //required for swerve
  try
  {
    ahrs = new AHRS(frc::SPI::Port::kMXP);
    printf("NAVX Initialized OK");
    HeadingOffset = ahrs->GetYaw();
  }
  catch(const std::exception e)
  {
    printf("!!! NAVX ERROR !!!");
    HeadingOffset = 0;
  }
  
  //FMSMatch = frc::DriverStation::IsFMSAttached(); //is this a real match with FMS
  AutoTimer = new frc::Timer();
  AutoTimer->Start();
  ModeTimer = new frc::Timer();
  ModeTimer->Start();

  ClockStart = frc::Timer::GetFPGATimestamp();

}

void Robot::RobotPeriodic() 
{
  //only update this until match starts
  if(!MatchStart)
  {
    static int counter1 = 0;
    counter1++;
    if(counter1 >= 50) //1 secs
    {
      counter1=0;
      //Read AutoSelect and write AutoSelected on BOSS dashboard
      dAutoSelect = (int) ntBOSS->GetNumber("AutoSelect", 0.0);
      ntBOSS->PutNumber("AutoSelected", dAutoSelect);
      //allow BOSS dashboard to reset sticky faults
      int ResetStickyFaults = (int) ntBOSS->GetNumber("ResetStickyFaults", 0);
      if(ResetStickyFaults == 1)
      {
        ntBOSS->PutNumber("ResetStickyFaults", 0);
      }
      ntBOSS->PutNumber("ClockMode", 0);
    }
  }
  static int counter2 = 0;
  counter2++;
  if(counter2 >= 13) //250 msecs
  {
      ClockNow = frc::Timer::GetFPGATimestamp();
      counter2 = 0;
      if(IsAutonomousEnabled())
      {
        ntBOSS->PutNumber("ClockMode", 1);
        ntBOSS->PutNumber("ClockElapsed", std::clamp(15.0 - (double)(ClockNow - ClockStart),0.0,15.0));
      }
      if(IsTeleopEnabled())
      {
        ntBOSS->PutNumber("ClockMode", 2);
        ntBOSS->PutNumber("ClockElapsed", std::clamp(75.0 - (double)(ClockNow - ClockStart),0.0,75.0));
      }
      ntBOSS->PutNumber("CurMode", CurMode);
  }

  static int counter3 = 0;
  counter3++;
  if(counter3 >= 50) //1 secs
  {
    counter3=0;
    ntBOSS->PutNumber("FR_DIR",frSwerve.turnPV);
    ntBOSS->PutNumber("FR_DIST", frSwerve.driveOUT);
    ntBOSS->PutNumber("FL_DIR",flSwerve.turnPV);
    ntBOSS->PutNumber("FL_DIST", flSwerve.driveOUT);
    ntBOSS->PutNumber("RL_DIR",rlSwerve.turnPV);
    ntBOSS->PutNumber("RL_DIST", rlSwerve.driveOUT);
    ntBOSS->PutNumber("RR_DIR",rrSwerve.turnPV);
    ntBOSS->PutNumber("RR_DIST", rrSwerve.driveOUT);
    ntBOSS->PutNumber("joy_FORWARD", forward);
    ntBOSS->PutNumber("joy_STRAFE", strafe);
    ntBOSS->PutNumber("joy_ROTATE", rotate);
    ntBOSS->PutNumber("HeadingOffset", HeadingOffset);
    ntBOSS->PutNumber("Heading", Heading);
    ntBOSS->PutNumber("SwerveOrientationToField", SwerveOrientationToField);
    try
    {
      ntBOSS->PutNumber("ahrs_PITCH", ahrs->GetPitch());
      ntBOSS->PutNumber("ahrs_ROLL", ahrs->GetRoll());
    }
    catch(const std::exception e)
    {
      ntBOSS->PutNumber("ahrs_PITCH", 0);
      ntBOSS->PutNumber("ahrs_ROLL", 0);
    }
  }
  ntBOSS->PutString("AutoStatus", "");
}

void Robot::AutonomousInit() 
{
  CurMode = 1;
}

void Robot::AutonomousPeriodic() 
{}

void Robot::TeleopInit() 
{
  CurMode = 2;
}

void Robot::TeleopPeriodic() 
{
  //xbox
  forward = -(m_driveController.GetRightY());
  strafe =  m_driveController.GetRightX();
  rotate = m_driveController.GetLeftX();
  
  if(fabs(forward)<.15) {forward = 0;}
  double fforward = spdFilter.Calculate(forward);
  if(forward != 0) forward = fforward;
  if(fabs(strafe)<.15) {strafe = 0;}
  if(fabs(rotate)<.2) {rotate = 0;}

  DriveSwerve(forward, strafe, rotate);

  //toggle robot/field orientation for swerve drive
  if(m_driveController.GetRawButtonPressed(1)) {SwerveOrientationToField = !SwerveOrientationToField;}
}

void Robot::DisabledInit() 
{
  StopAllDrives();
  CurMode = 0;
}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() 
{
  CurMode = 3;
}

void Robot::TestPeriodic() 
{}

double Robot::GetHeading()
{
  //subtract the offset recorded at init
  double yaw = 0.0; 
  try
  {
    yaw = ahrs->GetYaw() - HeadingOffset;
    //normalize angle
    yaw = CheckWrap(yaw);
  }
  catch(const std::exception e)
  {
    yaw = 0.0;
  }
  return yaw;
}

#ifndef RUNNING_FRC_TESTS
int main() 
{
  return frc::StartRobot<Robot>();
}
#endif

