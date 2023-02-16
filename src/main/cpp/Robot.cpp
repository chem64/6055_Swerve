#include "Robot.h"

void Robot::RobotInit() 
{
  ntBOSS = nt::NetworkTableInstance::GetDefault().GetTable("dashBOSS");
  ConfigMotors();
<<<<<<< HEAD
  frTurnPID.EnableContinuousInput(-180.0,180.0); //required for swerve
  flTurnPID.EnableContinuousInput(-180.0,180.0); //required for swerve
  rlTurnPID.EnableContinuousInput(-180.0,180.0); //required for swerve
  rrTurnPID.EnableContinuousInput(-180.0,180.0); //required for swerve
=======
  ArmBrake.SetBounds(2.0, 1.8, 1.5, 1.2, 1.0); 
  m_TurnPID.EnableContinuousInput(-180.0,180.0); //required for swerve
>>>>>>> 782c8896b3e564d9978b473e074ff2a6a50699ba
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
<<<<<<< HEAD
    ntBOSS->PutNumber("FR_DIR",frSwerve.turnPV);
    ntBOSS->PutNumber("FR_DIST", frSwerve.driveOUT);
    ntBOSS->PutNumber("FL_DIR",flSwerve.turnPV);
    ntBOSS->PutNumber("FL_DIST", flSwerve.driveOUT);
    ntBOSS->PutNumber("RL_DIR",rlSwerve.turnPV);
    ntBOSS->PutNumber("RL_DIST", rlSwerve.driveOUT);
    ntBOSS->PutNumber("RR_DIR",rrSwerve.turnPV);
    ntBOSS->PutNumber("RR_DIST", rrSwerve.driveOUT);
=======
    ntBOSS->PutNumber("FR_POS",CheckWrap(m_frEncoder.GetPosition()-constants::kFrontRightOffset));
    ntBOSS->PutNumber("FR_DIST", m_frDrive.GetSelectedSensorPosition() * constants::kDriveUnitsToFeet);
    ntBOSS->PutNumber("FL_POS",CheckWrap(m_flEncoder.GetPosition()-constants::kFrontLeftOffset));
    ntBOSS->PutNumber("FL_DIST", m_flDrive.GetSelectedSensorPosition() * constants::kDriveUnitsToFeet);
    ntBOSS->PutNumber("RL_POS",CheckWrap(m_rlEncoder.GetPosition()-constants::kRearLeftOffset));
    ntBOSS->PutNumber("RL_DIST", m_rlDrive.GetSelectedSensorPosition() * constants::kDriveUnitsToFeet);
    ntBOSS->PutNumber("RR_POS",CheckWrap(m_rrEncoder.GetPosition()-constants::kRearRightOffset));
    ntBOSS->PutNumber("RR_DIST", m_rrDrive.GetSelectedSensorPosition() * constants::kDriveUnitsToFeet);
    ntBOSS->PutNumber("Winch",m_winch1.GetSelectedSensorPosition());
>>>>>>> 782c8896b3e564d9978b473e074ff2a6a50699ba
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
<<<<<<< HEAD
{}
=======
{
  //Self_Level();
  
  /*To drive fully out, the position is set to 1.0 (2" stroke) 
  To drive half way, the position is set to 0.5 
  To drive fully in, the position is set to 0.0
  ArmBrake.SetBounds(2.0, 1.8, 1.5, 1.2, 1.0); */
  static bool BrakeOn;
  if(m_driveController.GetRawButtonPressed(1)) BrakeOn = !BrakeOn;
  if(BrakeOn) ArmBrake.SetPosition(0.0);  //retracted in
  if(!BrakeOn) ArmBrake.SetPosition(1.0); //extended out


}
>>>>>>> 782c8896b3e564d9978b473e074ff2a6a50699ba

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

