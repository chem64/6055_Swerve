#include "Robot.h"

enum Constants {kSlotIdx = 0, kPIDLoopIdx = 0, kTimeoutMs = 50};

//wraps -180 to/from 180
double Robot::CheckWrap(double pos)
{
  double ret = pos;
  ret -= 360. * std::floor((ret + 180.) * (1. / 360.)); 
  return ret;
}

void Robot::StopAllDrives()
{
  can_rrTurn.StopMotor();
  can_frTurn.StopMotor();
  can_rlTurn.StopMotor();
  can_flTurn.StopMotor();
  can_rrDrive.StopMotor();
  can_frDrive.StopMotor();
  can_rlDrive.StopMotor();
  can_flDrive.StopMotor();
}

double Robot::GetEffectiveAngle(double actAngle,double flip)
{
  //get value of actual turn angle
  double ret = actAngle;
  //now get the effective angle (actual direction of movement)
  if (flip < 0) (ret<=0.0)?ret+=180.0:ret-=180.0;
  return ret;
}

void Robot::DriveSwerve(double FWD, double STR, double RCW)
{
 
  double L = constants::kWheelbase; //wheelbase (from center of front wheel to center of back wheel)
  double W = constants::kWheelwidth; //wheelwidth (from center of left wheel to center of right wheel)
  double R = sqrt((L * L) + (W * W));

  //get current heading
  Heading = GetHeading();
  if(SwerveOrientationToField)
  {
    //convert to radians
    double yaw = Heading / constants::kRadtoDeg;
    //recalculate joystick inputs for field orientation
    double tmp = FWD * cos(yaw) + STR * sin(yaw);
    STR = -FWD * sin(yaw) + STR * cos(yaw);
    FWD = tmp;
  }

  double A = STR - RCW * (L / R);
  double B = STR + RCW * (L / R);
  double C = FWD - RCW * (W / R);
  double D = FWD + RCW * (W / R);

  //joystick values are (-180 to 180 degrees)
  double ws1 = sqrt((B * B) + (C * C));
  double wa1 = atan2(B, C) * 180 / M_PI;
  double ws2 = sqrt((B * B) + (D * D));
  double wa2 = atan2(B, D) * 180 / M_PI;
  double ws3 = sqrt((A * A) + (D * D));
  double wa3 = atan2(A, D) * 180 / M_PI;
  double ws4 = sqrt((A * A) + (C * C));
  double wa4 = atan2(A, C) * 180 / M_PI;
 
  //normalize wheel speeds to max speed
  double max = ws1;
  if (ws2 > max)max = ws2;
  if (ws3 > max)max = ws3;
  if (ws4 > max)max = ws4;
  if (max > 1) { ws1 /= max; ws2 /= max; ws3 /= max; ws4 /= max; }

  //if no joystick input - return without changing angles
  if (FWD == 0 && STR == 0 && RCW == 0)
  {
    StopAllDrives();
<<<<<<< HEAD
=======
    return;
  }
  //Front Right
  //set setpoint to joystick
  double turnSP = wa1;
  //actual angle is where encoder is pointing
  double ActAngle = CheckWrap(m_frEncoder.GetPosition()-constants::kFrontRightOffset);  //cancoder position accumulates on every turn - does not automatically wrap
  //get the effective angle - direction motor is moving
  double turnPV = GetEffectiveAngle(ActAngle,frFlip);
  //if desired setpoint change is greater than our setting - flip direction of drive
  if (fabs(turnSP - turnPV) > constants::kSwerveAngleBreak)
  {
    frFlip *= -1.0;
    turnPV = GetEffectiveAngle(ActAngle,frFlip);
  }
  //calculate PID base on effective angle and setpoint
  double turnOUT = std::clamp(m_TurnPID.Calculate(turnPV,turnSP),-1.0,1.0); 
  m_frTurn.Set(ControlMode::PercentOutput,turnOUT);
  m_frDrive.Set(ControlMode::PercentOutput,ws1 *= frFlip);

  //Front Left
  //set setpoint to joystick
  turnSP = wa2;
  //actual angle is where encoder is pointing
  ActAngle = CheckWrap(m_flEncoder.GetPosition()-constants::kFrontLeftOffset);  //cancoder position accumulates on every turn - does not automatically wrap
  //get the effective angle - direction motor is moving
  turnPV = GetEffectiveAngle(ActAngle,flFlip);
  //if desired setpoint change is greater than our setting - flip direction of drive
  if (fabs(turnSP - turnPV) > constants::kSwerveAngleBreak)
  {
    flFlip *= -1.0;
    turnPV = GetEffectiveAngle(ActAngle,flFlip);
  }
  //calculate PID base on effective angle and setpoint
  turnOUT = std::clamp(m_TurnPID.Calculate(turnPV,turnSP),-1.0,1.0); 
  m_flTurn.Set(ControlMode::PercentOutput,turnOUT);
  m_flDrive.Set(ControlMode::PercentOutput,ws2 *= flFlip);

  //Rear Left
  //set setpoint to joystick
  turnSP = wa3;
  //actual angle is where encoder is pointing
  ActAngle = CheckWrap(m_rlEncoder.GetPosition()-constants::kRearLeftOffset);  //cancoder position accumulates on every turn - does not automatically wrap
  //get the effective angle - direction motor is moving
  turnPV = GetEffectiveAngle(ActAngle,rlFlip);
  //if desired setpoint change is greater than our setting - flip direction of drive
  if (fabs(turnSP - turnPV) > constants::kSwerveAngleBreak)
  {
    rlFlip *= -1.0;
    turnPV = GetEffectiveAngle(ActAngle,rlFlip);
  }
  //calculate PID base on effective angle and setpoint
  turnOUT = std::clamp(m_TurnPID.Calculate(turnPV,turnSP),-1.0,1.0); 
  m_rlTurn.Set(ControlMode::PercentOutput,turnOUT);
  m_rlDrive.Set(ControlMode::PercentOutput,ws3 *= rlFlip);

  //Rear Right
  //set setpoint to joystick
  turnSP = wa4;
  //actual angle is where encoder is pointing
  ActAngle = CheckWrap(m_rrEncoder.GetPosition()-constants::kRearRightOffset);  //cancoder position accumulates on every turn - does not automatically wrap
  //get the effective angle - direction motor is moving
  turnPV = GetEffectiveAngle(ActAngle,rrFlip);
  //if desired setpoint change is greater than our setting - flip direction of drive
  if (fabs(turnSP - turnPV) > constants::kSwerveAngleBreak)
  {
    rrFlip *= -1.0;
    turnPV = GetEffectiveAngle(ActAngle,rrFlip);
  }
  //calculate PID based on effective angle and setpoint
  turnOUT = std::clamp(m_TurnPID.Calculate(turnPV,turnSP),-1.0,1.0); 
  m_rrTurn.Set(ControlMode::PercentOutput,turnOUT);
  m_rrDrive.Set(ControlMode::PercentOutput,ws4 *= rrFlip);
}

/*void Robot::DriveSwerve(double FWD, double STR, double RCW)
{
  
  L = constants::kWheelbase; //wheelbase (from center of front wheel to center of back wheel)
  W = constants::kWheelwidth; //wheelwidth (from center of left wheel to center of right wheel)
  R = sqrt((L * L) + (W * W));

  //get current heading
  Heading = GetHeading();
  if(SwerveOrientationToField)
  {
    //convert to radians
    double yaw = Heading / constants::kRadtoDeg;
    //recalculate joystick inputs
    double tmp = FWD * cos(yaw) + STR * sin(yaw);
    STR = -FWD * sin(yaw) + STR * cos(yaw);
    FWD = tmp;
  }

  A = STR - RCW * (L / R);
  B = STR + RCW * (L / R);
  C = FWD - RCW * (W / R);
  D = FWD + RCW * (W / R);

  ws1 = sqrt((B * B) + (C * C));
  wa1 = atan2(B, C) * 180 / M_PI;   //-180 to 180 degrees
  ws2 = sqrt((B * B) + (D * D));
  wa2 = atan2(B, D) * 180 / M_PI;
  ws3 = sqrt((A * A) + (D * D));
  wa3 = atan2(A, D) * 180 / M_PI;
  ws4 = sqrt((A * A) + (C * C));
  wa4 = atan2(A, C) * 180 / M_PI;
  
  double max = ws1;
  if (ws2 > max)max = ws2;
  if (ws3 > max)max = ws3;
  if (ws4 > max)max = ws4;
  if (max > 1) { ws1 /= max; ws2 /= max; ws3 /= max; ws4 /= max; }

  if (FWD == 0 && STR == 0 && RCW == 0)
  {
    StopAllDrives();
    UpdateSwerveSP();
>>>>>>> 782c8896b3e564d9978b473e074ff2a6a50699ba
    return;
  }

<<<<<<< HEAD
  //Front Right
  //set setpoint to joystick
  frSwerve.turnSP = wa1;
  //actual angle is where encoder is pointing
  frSwerve.actAngle = CheckWrap(can_frEncoder.GetPosition()-constants::kFrontRightOffset);  //cancoder position accumulates on every turn - does not automatically wrap
  //get the effective angle - direction wheel is moving
  frSwerve.turnPV = GetEffectiveAngle(frSwerve.actAngle,frSwerve.flip);
  //if desired setpoint change is greater than our setting - flip direction of drive
  if (CheckWrap(fabs(frSwerve.turnSP - frSwerve.turnPV)) > constants::kSwerveAngleBreak)
  {
    frSwerve.flip *= -1.0;
    frSwerve.turnPV = GetEffectiveAngle(frSwerve.actAngle,frSwerve.flip);
  }
  //calculate PID based on effective angle and setpoint
  frSwerve.turnOUT = std::clamp(frTurnPID.Calculate(frSwerve.turnPV,frSwerve.turnSP),-1.0,1.0);
  can_frTurn.Set(ControlMode::PercentOutput,frSwerve.turnOUT);
  frSwerve.driveOUT = ws1 * frSwerve.flip;
  can_frDrive.Set(ControlMode::PercentOutput,frSwerve.driveOUT);

=======
>>>>>>> 782c8896b3e564d9978b473e074ff2a6a50699ba
  //Front Left
  //set setpoint to joystick
  flSwerve.turnSP = wa2;
  //actual angle is where encoder is pointing
  flSwerve.actAngle = CheckWrap(can_flEncoder.GetPosition()-constants::kFrontLeftOffset);  //cancoder position accumulates on every turn - does not automatically wrap
  //get the effective angle - direction wheel is moving
  flSwerve.turnPV = GetEffectiveAngle(flSwerve.actAngle,flSwerve.flip);
  //if desired setpoint change is greater than our setting - flip direction of drive
  if (CheckWrap(fabs(flSwerve.turnSP - flSwerve.turnPV)) > constants::kSwerveAngleBreak)
  {
<<<<<<< HEAD
    flSwerve.flip *= -1.0;
    flSwerve.turnPV = GetEffectiveAngle(flSwerve.actAngle,flSwerve.flip);
=======
    if(flFlip < 0) (turnSP<=0.0)?turnSP+=180.0:turnSP-=180.0;
    if (fabs(turnSP - lastFL_SP) > constants::kSwerveAngleBreak) 
    {
      flFlip *= -1.0;
      (turnSP<=0.0)?turnSP+=180.0:turnSP-=180.0;
      lastFL_SP = turnSP;
    }
>>>>>>> 782c8896b3e564d9978b473e074ff2a6a50699ba
  }
  //calculate PID based on effective angle and setpoint
  flSwerve.turnOUT = std::clamp(flTurnPID.Calculate(flSwerve.turnPV,flSwerve.turnSP),-1.0,1.0);
  can_flTurn.Set(ControlMode::PercentOutput,flSwerve.turnOUT);
  flSwerve.driveOUT = ws2 * flSwerve.flip;
  can_flDrive.Set(ControlMode::PercentOutput,flSwerve.driveOUT);

  //Rear Left
  //set setpoint to joystick
  rlSwerve.turnSP = wa3;
  //actual angle is where encoder is pointing
  rlSwerve.actAngle = CheckWrap(can_rlEncoder.GetPosition()-constants::kRearLeftOffset);  //cancoder position accumulates on every turn - does not automatically wrap
  //get the effective angle - direction wheel is moving
  rlSwerve.turnPV = GetEffectiveAngle(rlSwerve.actAngle,rlSwerve.flip);
  //if desired setpoint change is greater than our setting - flip direction of drive
  if (CheckWrap(fabs(rlSwerve.turnSP - rlSwerve.turnPV)) > constants::kSwerveAngleBreak)
  {
<<<<<<< HEAD
    rlSwerve.flip *= -1.0;
    rlSwerve.turnPV = GetEffectiveAngle(rlSwerve.actAngle,rlSwerve.flip);
=======
    if(rlFlip < 0) (turnSP<=0.0)?turnSP+=180.0:turnSP-=180.0;
    if (fabs(turnSP - lastRL_SP) > constants::kSwerveAngleBreak) 
    {
      rlFlip *= -1.0;
      (turnSP<=0.0)?turnSP+=180.0:turnSP-=180.0;
      lastRL_SP = turnSP;
    }
>>>>>>> 782c8896b3e564d9978b473e074ff2a6a50699ba
  }
  //calculate PID based on effective angle and setpoint
  rlSwerve.turnOUT = std::clamp(rlTurnPID.Calculate(rlSwerve.turnPV,rlSwerve.turnSP),-1.0,1.0);
  can_rlTurn.Set(ControlMode::PercentOutput,rlSwerve.turnOUT);
  rlSwerve.driveOUT = ws3 * rlSwerve.flip;
  can_rlDrive.Set(ControlMode::PercentOutput,rlSwerve.driveOUT);

  //Rear Right
  //set setpoint to joystick
  rrSwerve.turnSP = wa4;
  //actual angle is where encoder is pointing
  rrSwerve.actAngle = CheckWrap(can_rrEncoder.GetPosition()-constants::kRearRightOffset);  //cancoder position accumulates on every turn - does not automatically wrap
  //get the effective angle - direction wheel is moving
  rrSwerve.turnPV = GetEffectiveAngle(rrSwerve.actAngle,rrSwerve.flip);
  //if desired setpoint change is greater than our setting - flip direction of drive
  if (CheckWrap(fabs(rrSwerve.turnSP - rrSwerve.turnPV)) > constants::kSwerveAngleBreak)
  {
<<<<<<< HEAD
    rrSwerve.flip *= -1.0;
    rrSwerve.turnPV = GetEffectiveAngle(rrSwerve.actAngle,rrSwerve.flip);
  }
  //calculate PID based on effective angle and setpoint
  rrSwerve.turnOUT = std::clamp(rrTurnPID.Calculate(rrSwerve.turnPV,rrSwerve.turnSP),-1.0,1.0);
  can_rrTurn.Set(ControlMode::PercentOutput,rrSwerve.turnOUT);
  rrSwerve.driveOUT = ws4 * rrSwerve.flip;
  can_rrDrive.Set(ControlMode::PercentOutput,rrSwerve.driveOUT);
}
=======
    if(rrFlip < 0) (turnSP<=0.0)?turnSP+=180.0:turnSP-=180.0;
    if (fabs(turnSP - lastRR_SP) > constants::kSwerveAngleBreak) 
    {
      rrFlip *= -1.0;
      (turnSP<=0.0)?turnSP+=180.0:turnSP-=180.0;
      lastRR_SP = turnSP;
    }
  }
  turnOUT = std::clamp(m_TurnPID.Calculate(turnPV,turnSP),-1.0,1.0); 
  m_rrTurn.Set(ControlMode::PercentOutput,turnOUT);
  m_rrDrive.Set(ControlMode::PercentOutput,ws4 *= rrFlip);
}*/
>>>>>>> 782c8896b3e564d9978b473e074ff2a6a50699ba

void Robot::ConfigMotors()
{
  driveSCLC = SupplyCurrentLimitConfiguration{true,constants::kDriveSupplyCurrentLimit,constants::kDrivePeakCurrentLimit,constants::kDrivePeakCurrentDuration};
  driveStatorSCLC = StatorCurrentLimitConfiguration{true,constants::kDriveStatorCurrentLimit,constants::kDriveStatorPeakCurrentLimit,constants::kDriveStatorPeakCurrentDuration};
  turnSCLC = SupplyCurrentLimitConfiguration{true,constants::kTurnSupplyCurrentLimit,constants::kTurnPeakCurrentLimit,constants::kTurnPeakCurrentDuration};
  turnStatorSCLC = StatorCurrentLimitConfiguration{true,constants::kTurnStatorCurrentLimit,constants::kTurnStatorPeakCurrentLimit,constants::kTurnStatorPeakCurrentDuration};

  can_frDrive.ConfigFactoryDefault(kTimeoutMs);
  can_frDrive.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, kTimeoutMs);
  can_frDrive.SetInverted(true);
  can_frDrive.ConfigNeutralDeadband(0.04, kTimeoutMs);
  can_frDrive.SetNeutralMode(NeutralMode::Brake);
  can_frDrive.ConfigNominalOutputForward(0, kTimeoutMs);
  can_frDrive.ConfigNominalOutputReverse(0, kTimeoutMs);
  can_frDrive.ConfigPeakOutputForward(constants::kDrivePeakOutputForward, kTimeoutMs);
  can_frDrive.ConfigPeakOutputReverse(constants::kDrivePeakOutputReverse, kTimeoutMs);
  can_frDrive.Config_kF(0, constants::kDrive_kF, kTimeoutMs);
  can_frDrive.Config_kP(0, constants::kDrive_kP, kTimeoutMs);
  can_frDrive.Config_kI(0, constants::kDrive_kI, kTimeoutMs);
  can_frDrive.Config_kD(0, constants::kDrive_kD, kTimeoutMs);
  can_frDrive.ConfigOpenloopRamp(constants::kDriveOpenLoopRamp,kTimeoutMs); 
  can_frDrive.Config_IntegralZone(0, 400, kTimeoutMs);
  can_frDrive.ConfigClosedLoopPeakOutput(0,1.0,kTimeoutMs);
  can_frDrive.ConfigSupplyCurrentLimit(driveSCLC);//CURRENT_LIMITING
  can_frDrive.ConfigStatorCurrentLimit(driveStatorSCLC);//CURRENT_LIMITING
  can_frDrive.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
  can_frDrive.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);
  can_frDrive.ConfigVoltageCompSaturation(constants::kDriveVoltageCompSaturation);
  can_frDrive.EnableVoltageCompensation(true);
  can_frDrive.SetSafetyEnabled(false);

  can_flDrive.ConfigFactoryDefault(kTimeoutMs);
  can_flDrive.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, kTimeoutMs);
  can_flDrive.SetInverted(true);
  can_flDrive.ConfigNeutralDeadband(0.04, kTimeoutMs);
  can_flDrive.SetNeutralMode(NeutralMode::Brake);
  can_flDrive.ConfigNominalOutputForward(0, kTimeoutMs);
  can_flDrive.ConfigNominalOutputReverse(0, kTimeoutMs);
  can_flDrive.ConfigPeakOutputForward(constants::kDrivePeakOutputForward, kTimeoutMs);
  can_flDrive.ConfigPeakOutputReverse(constants::kDrivePeakOutputReverse, kTimeoutMs);
  can_flDrive.Config_kF(0, constants::kDrive_kF, kTimeoutMs);
  can_flDrive.Config_kP(0, constants::kDrive_kP, kTimeoutMs);
  can_flDrive.Config_kI(0, constants::kDrive_kI, kTimeoutMs);
  can_flDrive.Config_kD(0, constants::kDrive_kD, kTimeoutMs);
  can_flDrive.ConfigOpenloopRamp(constants::kDriveOpenLoopRamp,kTimeoutMs); 
  can_flDrive.Config_IntegralZone(0, 400, kTimeoutMs);
  can_flDrive.ConfigClosedLoopPeakOutput(0,1.0,kTimeoutMs);
  can_flDrive.ConfigSupplyCurrentLimit(driveSCLC);//CURRENT_LIMITING
  can_flDrive.ConfigStatorCurrentLimit(driveStatorSCLC);//CURRENT_LIMITING
  can_flDrive.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
  can_flDrive.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);
  can_flDrive.ConfigVoltageCompSaturation(constants::kDriveVoltageCompSaturation);
  can_flDrive.EnableVoltageCompensation(true);
  can_flDrive.SetSafetyEnabled(false);

  can_rlDrive.ConfigFactoryDefault(kTimeoutMs);
  can_rlDrive.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, kTimeoutMs);
  can_rlDrive.SetInverted(true);
  can_rlDrive.ConfigNeutralDeadband(0.04, kTimeoutMs);
  can_rlDrive.SetNeutralMode(NeutralMode::Brake);
  can_rlDrive.ConfigNominalOutputForward(0, kTimeoutMs);
  can_rlDrive.ConfigNominalOutputReverse(0, kTimeoutMs);
  can_rlDrive.ConfigPeakOutputForward(constants::kDrivePeakOutputForward, kTimeoutMs);
  can_rlDrive.ConfigPeakOutputReverse(constants::kDrivePeakOutputReverse, kTimeoutMs);
  can_rlDrive.Config_kF(0, constants::kDrive_kF, kTimeoutMs);
  can_rlDrive.Config_kP(0, constants::kDrive_kP, kTimeoutMs);
  can_rlDrive.Config_kI(0, constants::kDrive_kI, kTimeoutMs);
  can_rlDrive.Config_kD(0, constants::kDrive_kD, kTimeoutMs);
  can_rlDrive.ConfigOpenloopRamp(constants::kDriveOpenLoopRamp,kTimeoutMs); 
  can_rlDrive.Config_IntegralZone(0, 400, kTimeoutMs);
  can_rlDrive.ConfigClosedLoopPeakOutput(0,1.0,kTimeoutMs);
  can_rlDrive.ConfigSupplyCurrentLimit(driveSCLC);//CURRENT_LIMITING
  can_rlDrive.ConfigStatorCurrentLimit(driveStatorSCLC);//CURRENT_LIMITING
  can_rlDrive.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
  can_rlDrive.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);
  can_rlDrive.ConfigVoltageCompSaturation(constants::kDriveVoltageCompSaturation);
  can_rlDrive.EnableVoltageCompensation(true);
  can_rlDrive.SetSafetyEnabled(false);

  can_rrDrive.ConfigFactoryDefault(kTimeoutMs);
  can_rrDrive.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, kTimeoutMs);
  can_rrDrive.SetInverted(true);
  can_rrDrive.ConfigNeutralDeadband(0.04, kTimeoutMs);
  can_rrDrive.SetNeutralMode(NeutralMode::Brake);
  can_rrDrive.ConfigNominalOutputForward(0, kTimeoutMs);
  can_rrDrive.ConfigNominalOutputReverse(0, kTimeoutMs);
  can_rrDrive.ConfigPeakOutputForward(constants::kDrivePeakOutputForward, kTimeoutMs);
  can_rrDrive.ConfigPeakOutputReverse(constants::kDrivePeakOutputReverse, kTimeoutMs);
  can_rrDrive.Config_kF(0, constants::kDrive_kF, kTimeoutMs);
  can_rrDrive.Config_kP(0, constants::kDrive_kP, kTimeoutMs);
  can_rrDrive.Config_kI(0, constants::kDrive_kI, kTimeoutMs);
  can_rrDrive.Config_kD(0, constants::kDrive_kD, kTimeoutMs);
  can_rrDrive.ConfigOpenloopRamp(constants::kDriveOpenLoopRamp,kTimeoutMs); 
  can_rrDrive.Config_IntegralZone(0, 400, kTimeoutMs);
  can_rrDrive.ConfigClosedLoopPeakOutput(0,1.0,kTimeoutMs);
  can_rrDrive.ConfigSupplyCurrentLimit(driveSCLC);//CURRENT_LIMITING
  can_rrDrive.ConfigStatorCurrentLimit(driveStatorSCLC);//CURRENT_LIMITING
  can_rrDrive.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
  can_rrDrive.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);
  can_rrDrive.ConfigVoltageCompSaturation(constants::kDriveVoltageCompSaturation);
  can_rrDrive.EnableVoltageCompensation(true);
  can_rrDrive.SetSafetyEnabled(false);

  can_frEncoder.ConfigFactoryDefault();
  can_frEncoder.ConfigSensorDirection(true,10);
  can_frEncoder.ConfigSensorInitializationStrategy(SensorInitializationStrategy::BootToAbsolutePosition);
  can_frEncoder.ConfigAbsoluteSensorRange(AbsoluteSensorRange::Signed_PlusMinus180);
  can_frEncoder.SetStatusFramePeriod(ctre::phoenix::sensors::CANCoderStatusFrame::CANCoderStatusFrame_SensorData, 20,10);
  
  can_frTurn.ConfigFactoryDefault();
  can_frTurn.SetNeutralMode(NeutralMode::Brake);
  can_frTurn.ConfigNominalOutputForward(0);
	can_frTurn.ConfigNominalOutputReverse(0);
	can_frTurn.ConfigPeakOutputForward(constants::kTurnPeakOutputForward);
	can_frTurn.ConfigPeakOutputReverse(constants::kTurnPeakOutputReverse);
  can_frTurn.SetSafetyEnabled(false);
  can_frTurn.SetInverted(true);
  can_frTurn.ConfigSupplyCurrentLimit(turnSCLC);//CURRENT_LIMITING
  can_frTurn.ConfigStatorCurrentLimit(turnStatorSCLC);//CURRENT_LIMITING
  can_frTurn.SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 255, 10);
  can_frTurn.SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, 255, 10);
  
  can_flEncoder.ConfigFactoryDefault();
  can_flEncoder.ConfigSensorDirection(true,10);
  can_flEncoder.ConfigSensorInitializationStrategy(SensorInitializationStrategy::BootToAbsolutePosition);
  can_flEncoder.ConfigAbsoluteSensorRange(AbsoluteSensorRange::Signed_PlusMinus180);
  can_flEncoder.SetStatusFramePeriod(ctre::phoenix::sensors::CANCoderStatusFrame::CANCoderStatusFrame_SensorData, 20,10);
  
  can_flTurn.ConfigFactoryDefault();
  can_flTurn.SetNeutralMode(NeutralMode::Brake);
  can_flTurn.ConfigNominalOutputForward(0);
	can_flTurn.ConfigNominalOutputReverse(0);
	can_flTurn.ConfigPeakOutputForward(constants::kTurnPeakOutputForward);
	can_flTurn.ConfigPeakOutputReverse(constants::kTurnPeakOutputReverse);
  can_flTurn.SetSafetyEnabled(false);
  can_flTurn.SetInverted(true);
  can_flTurn.ConfigSupplyCurrentLimit(turnSCLC);//CURRENT_LIMITING
  can_flTurn.ConfigStatorCurrentLimit(turnStatorSCLC);//CURRENT_LIMITING
  can_flTurn.SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 255, 10);
  can_flTurn.SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, 255, 10);

  can_rlEncoder.ConfigFactoryDefault();
  can_rlEncoder.ConfigSensorDirection(true,10);
  can_rlEncoder.ConfigSensorInitializationStrategy(SensorInitializationStrategy::BootToAbsolutePosition);
  can_rlEncoder.ConfigAbsoluteSensorRange(AbsoluteSensorRange::Signed_PlusMinus180);
  can_rlEncoder.SetStatusFramePeriod(ctre::phoenix::sensors::CANCoderStatusFrame::CANCoderStatusFrame_SensorData, 20,10);
  
<<<<<<< HEAD
  can_rlTurn.ConfigFactoryDefault();
  can_rlTurn.SetNeutralMode(NeutralMode::Brake);
  can_rlTurn.ConfigNominalOutputForward(0);
	can_rlTurn.ConfigNominalOutputReverse(0);
	can_rlTurn.ConfigPeakOutputForward(constants::kTurnPeakOutputForward);
	can_rlTurn.ConfigPeakOutputReverse(constants::kTurnPeakOutputReverse);
  can_rlTurn.SetSafetyEnabled(false);
  can_rlTurn.SetInverted(true);
  can_rlTurn.ConfigSupplyCurrentLimit(turnSCLC);//CURRENT_LIMITING
  can_rlTurn.ConfigStatorCurrentLimit(turnStatorSCLC);//CURRENT_LIMITING
  can_rlTurn.SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 255, 10);
  can_rlTurn.SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, 255, 10);

  can_rrEncoder.ConfigFactoryDefault();
  can_rrEncoder.ConfigSensorDirection(true,10);
  can_rrEncoder.ConfigSensorInitializationStrategy(SensorInitializationStrategy::BootToAbsolutePosition);
  can_rrEncoder.ConfigAbsoluteSensorRange(AbsoluteSensorRange::Signed_PlusMinus180);
  can_rrEncoder.SetStatusFramePeriod(ctre::phoenix::sensors::CANCoderStatusFrame::CANCoderStatusFrame_SensorData, 20,10);
  
  can_rrTurn.ConfigFactoryDefault();
  can_rrTurn.SetNeutralMode(NeutralMode::Brake);
  can_rrTurn.ConfigNominalOutputForward(0);
	can_rrTurn.ConfigNominalOutputReverse(0);
	can_rrTurn.ConfigPeakOutputForward(constants::kTurnPeakOutputForward);
	can_rrTurn.ConfigPeakOutputReverse(constants::kTurnPeakOutputReverse);
  can_rrTurn.SetSafetyEnabled(false);
  can_rrTurn.SetInverted(true);
  can_rrTurn.ConfigSupplyCurrentLimit(turnSCLC);//CURRENT_LIMITING
  can_rrTurn.ConfigStatorCurrentLimit(turnStatorSCLC);//CURRENT_LIMITING
  can_rrTurn.SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 255, 10);
  can_rrTurn.SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, 255, 10);
 
=======
  m_rrTurn.ConfigFactoryDefault();
  m_rrTurn.SetNeutralMode(NeutralMode::Brake);
  m_rrTurn.ConfigNominalOutputForward(0);
	m_rrTurn.ConfigNominalOutputReverse(0);
	m_rrTurn.ConfigPeakOutputForward(constants::kTurnPeakOutputForward);
	m_rrTurn.ConfigPeakOutputReverse(constants::kTurnPeakOutputReverse);
  m_rrTurn.SetSafetyEnabled(false);
  m_rrTurn.SetInverted(true);
  m_rrTurn.ConfigSupplyCurrentLimit(turnSCLC);//CURRENT_LIMITING
  m_rrTurn.ConfigStatorCurrentLimit(turnStatorSCLC);//CURRENT_LIMITING
  m_rrTurn.SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 255, 10);
  m_rrTurn.SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, 255, 10);

  m_winch1.ConfigFactoryDefault(kTimeoutMs);
  m_winch1.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0,kTimeoutMs);
  m_winch1.SetNeutralMode(NeutralMode::Brake);
  m_winch1.SetSensorPhase(false);
  m_winch1.SetInverted(false);
  m_winch1.SetSafetyEnabled(false);
  m_winch1.ConfigPeakCurrentLimit(constants::kPeakCurrentLimit, kTimeoutMs);
  m_winch1.ConfigPeakCurrentDuration(constants::kPeakCurrentDuration, kTimeoutMs);
  m_winch1.ConfigContinuousCurrentLimit(constants::kContinuousCurrentLimit, kTimeoutMs);
  m_winch1.EnableCurrentLimit(true);
  m_winch1.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
  m_winch1.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);
  m_winch1.ConfigNominalOutputForward(0, kTimeoutMs);
  m_winch1.ConfigNominalOutputReverse(0, kTimeoutMs);
  m_winch1.ConfigPeakOutputForward(1, kTimeoutMs);
  m_winch1.ConfigPeakOutputReverse(-1, kTimeoutMs);
  m_winch1.Config_kF(0, constants::kWinch_F, kTimeoutMs);
  m_winch1.Config_kP(0, constants::kWinch_P, kTimeoutMs);
  m_winch1.Config_kI(0, constants::kWinch_I, kTimeoutMs);
  m_winch1.Config_kD(0, constants::kWinch_D, kTimeoutMs);
  m_winch1.Config_IntegralZone(0, 400, kTimeoutMs);
  m_winch1.ConfigAllowableClosedloopError(0,constants::kWinch_AllowableError,kTimeoutMs); 
  m_winch1.ConfigMotionCruiseVelocity(constants::kWinch_MotionCruiseVelocity, kTimeoutMs);
  m_winch1.ConfigMotionAcceleration(constants::kWinch_MotionAcceleration, kTimeoutMs);
  m_winch1.ConfigMotionSCurveStrength(constants::kWinch_MotionSCurveStrength, kTimeoutMs);
  m_winch1.ConfigVoltageCompSaturation(constants::kVoltageCompSaturation);
  m_winch1.EnableVoltageCompensation(true);
  m_winch1.ConfigForwardSoftLimitThreshold(constants::kWinch_ExtendLimit);
  m_winch1.ConfigReverseSoftLimitThreshold(constants::kWinch_RetractLimit);
  m_winch1.ConfigForwardSoftLimitEnable(true);
  m_winch1.ConfigReverseSoftLimitEnable(true);
>>>>>>> 782c8896b3e564d9978b473e074ff2a6a50699ba
}
