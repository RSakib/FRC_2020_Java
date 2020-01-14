/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Input.*;
import frc.robot.utility.DefaultDriveTalonSRX;
import frc.robot.RobotMap;  
import frc.robot.Constants; 
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Solenoid;



/**
 * Author Fernando Matias (207)
 */
public class Drivebase extends SubsystemBase {

  private static final Drivebase instance = new Drivebase();
  public static Drivebase getInstance() {
    return instance;
  }   
  private static Solenoid mShifter_High, mShifter_Low;
  public static DifferentialDrive mDrive;
  public static DefaultDriveTalonSRX mDrive_Left_Master, mDrive_Left_B, mDrive_Left_C, mDrive_Right_Master, mDrive_Right_B, mDrive_Right_C;
  private double TurnrateCurved, mLastHeadingErrorDegrees, leftvelo_,  rightvelo_, left_distance, right_distance, time;

  public Drivebase() {
    mDrive_Left_Master = new DefaultDriveTalonSRX(RobotMap.mDrive_Left_A_ID);
    mDrive_Left_B = new DefaultDriveTalonSRX(RobotMap.mDrive_Left_B_ID);
    mDrive_Left_C = new DefaultDriveTalonSRX(RobotMap.mDrive_Left_C_ID);
    mDrive_Right_Master = new DefaultDriveTalonSRX(RobotMap.mDrive_Right_A_ID);
    mDrive_Right_B = new DefaultDriveTalonSRX(RobotMap.mDrive_Right_B_ID);
    mDrive_Right_C = new DefaultDriveTalonSRX(RobotMap.mDrive_Right_C_ID);

    mDrive_Left_B.set(ControlMode.Follower, RobotMap.mDrive_Left_A_ID);
    mDrive_Left_C.set(ControlMode.Follower, RobotMap.mDrive_Left_A_ID);
    mDrive_Right_B.set(ControlMode.Follower, RobotMap.mDrive_Right_A_ID);
    mDrive_Right_C.set(ControlMode.Follower, RobotMap.mDrive_Right_A_ID);
    
    mDrive = new DifferentialDrive(mDrive_Left_Master, mDrive_Right_Master);
    mDrive.setSafetyEnabled(false);

    mShifter_Low = new Solenoid(RobotMap.PCM_A, RobotMap.mShiftLow_ID);
    mShifter_High = new Solenoid(RobotMap.PCM_B, RobotMap.mShiftHigh_ID);
}

  public void UpShift() {
    mShifter_High.set(Constants.On);
   mShifter_Low.set(Constants.Off);
   Constants.CURRENT_GEAR = Constants.HIGH_GEAR;
  }

  public void DownShift() {
    mShifter_High.set(Constants.Off);
    mShifter_Low.set(Constants.On);
    Constants.CURRENT_GEAR = Constants.LOW_GEAR;
  }
  
/*   private void configureTalonsForSpeedControl() {
    mDrive_Left_Master.set(ControlMode.Velocity, 0.0);
    mDrive_Left_Master.selectProfileSlot(kVelocityControlSlot, 0);
    mDrive_Left_Master.configAllowableClosedloopError(0, Constants.kDriveVelocityAllowableError);
    mDrive_Right_Master.set(ControlMode.Velocity, 0.0);
    mDrive_Right_Master.selectProfileSlot(kVelocityControlSlot, 0);
    mDrive_Right_Master.configAllowableClosedloopError(0, Constants.kDriveVelocityAllowableError);
    setBrake();
  }
 */
public void setCoast() {
  mDrive_Left_Master.setNeutralMode(NeutralMode.Coast);
  mDrive_Left_B.setNeutralMode(NeutralMode.Coast);
  mDrive_Left_C.setNeutralMode(NeutralMode.Coast);
  mDrive_Right_Master.setNeutralMode(NeutralMode.Coast);
  mDrive_Right_B.setNeutralMode(NeutralMode.Coast);
  mDrive_Right_C.setNeutralMode(NeutralMode.Coast);
}
public void setBrake() {
  mDrive_Left_Master.setNeutralMode(NeutralMode.Brake);
  mDrive_Left_B.setNeutralMode(NeutralMode.Brake);
  mDrive_Left_C.setNeutralMode(NeutralMode.Brake);
  mDrive_Right_Master.setNeutralMode(NeutralMode.Brake);
  mDrive_Right_B.setNeutralMode(NeutralMode.Brake);
  mDrive_Right_C.setNeutralMode(NeutralMode.Brake);
}
public void StopDrivetrain() {
  mDrive_Left_Master.set(ControlMode.PercentOutput, 0.0);
  mDrive_Right_Master.set(ControlMode.PercentOutput, 0.0);
}
public void curvature(double throttleaxis, double turnaxis) {
  TurnrateCurved = (Constants.kTurnrateCurve * Math.pow(turnaxis, 3)+(1-Constants.kTurnrateCurve)*turnaxis*Constants.kTurnrateLimit);
  mDrive.curvatureDrive(throttleaxis, TurnrateCurved, true);
}
@Override
public void periodic() {
  // This method will be called once per scheduler run
}
}
