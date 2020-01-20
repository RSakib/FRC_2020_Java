/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj.PIDController;
//import edu.wpi.first.wpilibj.PIDSource;
//import frc.robot.Input.*;
import frc.robot.utility.DefaultDriveTalonSRX;
import frc.robot.RobotMap;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
//import com.ctre.phoenix.motorcontrol.FollowerType;
//import com.ctre.phoenix.motorcontrol.InvertType;
import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Solenoid;

import edu.wpi.first.wpilibj.controller.PIDController;

//import frc.robot.PID.DummyPIDOutPut;

import frc.robot.utility.PurePursuit.*;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;



/**
 * Author Fernando Matias (207)
 */
public class Drivebase extends SubsystemBase {

  //Creates new Drivebase
  private static final Drivebase instance = new Drivebase();
  //returnns returns value
  public static Drivebase getInstance() {
    return instance;
  }   
  //Defining Variables
  //Drive Train
  public static DefaultDriveTalonSRX mDrive_Left_Master, mDrive_Left_B, mDrive_Left_C, mDrive_Right_Master, mDrive_Right_B, mDrive_Right_C;
  //Drive Train Class
  public static DifferentialDrive mDrive;

 // public static DummyPIDOutput PIDturnOutput, PIDleftOutput, PIDrightOutput;
  
  protected static final int kVelocityControlSlot = 0;
  protected static final int kBaseLockControlSlot = 1;

  private static AdaptivePurePursuitController pathFollowingController_;
  private static SynchronousPID velocityHeadingPid_;

  private double TurnrateCurved, mLastHeadingErrorDegrees, leftvelo_,  rightvelo_, left_distance, right_distance, time;
  private static Solenoid mShifter_High, mShifter_Low;

  public static PIDController PIDturn;

  private AHRS ahrs;

  public Drivebase() {
    mDrive_Left_Master = new DefaultDriveTalonSRX(RobotMap.mDrive_Left_A_ID);
    mDrive_Left_B = new DefaultDriveTalonSRX(RobotMap.mDrive_Left_B_ID);
    mDrive_Left_C = new DefaultDriveTalonSRX(RobotMap.mDrive_Left_C_ID);
    mDrive_Right_Master = new DefaultDriveTalonSRX(RobotMap.mDrive_Right_A_ID);
    mDrive_Right_B = new DefaultDriveTalonSRX(RobotMap.mDrive_Right_B_ID);
    mDrive_Right_C = new DefaultDriveTalonSRX(RobotMap.mDrive_Right_C_ID);

    mDrive_Left_Master.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, Constants.kTimeoutms);
    mDrive_Left_Master.setSensorPhase(false);
    mDrive_Left_Master.setInverted(false);
    mDrive_Left_B.setInverted(false);
    mDrive_Left_C.setInverted(false);

    mDrive_Right_Master.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, Constants.kTimeoutms);
    mDrive_Right_Master.setSensorPhase(false);
    mDrive_Right_Master.setInverted(false);
    mDrive_Right_B.setInverted(false);
    mDrive_Right_C.setInverted(false);

    mDrive_Left_B.set(ControlMode.Follower, RobotMap.mDrive_Left_A_ID);
    mDrive_Left_C.set(ControlMode.Follower, RobotMap.mDrive_Left_A_ID);
    mDrive_Right_B.set(ControlMode.Follower, RobotMap.mDrive_Right_A_ID);
    mDrive_Right_C.set(ControlMode.Follower, RobotMap.mDrive_Right_A_ID);

    mDrive_Left_Master.config_kP(kVelocityControlSlot, Constants.kDriveVelocityKp);
    mDrive_Left_Master.config_kI(kVelocityControlSlot, Constants.kDriveVelocityKi);
    mDrive_Left_Master.config_kD(kVelocityControlSlot, Constants.kDriveVelocityKd);
    mDrive_Left_Master.config_kF(kVelocityControlSlot, Constants.kDriveVelocityKf);
    mDrive_Left_Master.config_IntegralZone(kVelocityControlSlot, Constants.Drive_kIzone);

    mDrive_Right_Master.config_kP(kVelocityControlSlot, Constants.kDriveVelocityKp);
    mDrive_Right_Master.config_kI(kVelocityControlSlot, Constants.kDriveVelocityKi);
    mDrive_Right_Master.config_kD(kVelocityControlSlot, Constants.kDriveVelocityKd);
    mDrive_Right_Master.config_kF(kVelocityControlSlot, Constants.kDriveVelocityKf);
    mDrive_Right_Master.config_IntegralZone(kVelocityControlSlot, Constants.Drive_kIzone);

    velocityHeadingPid_ = new SynchronousPID(Constants.kDriveHeadingVelocityKp, Constants.kDriveHeadingVelocityKi, Constants.kDriveHeadingVelocityKd);
    velocityHeadingPid_.setOutputRange(-30, 30);
    
    mDrive = new DifferentialDrive(mDrive_Left_Master, mDrive_Right_Master);
    mDrive.setSafetyEnabled(false);

    mShifter_Low = new Solenoid(RobotMap.PCM_A, RobotMap.mShiftLow_ID);
    mShifter_High = new Solenoid(RobotMap.PCM_B, RobotMap.mShiftHigh_ID);

    ahrs = new AHRS(SerialPort.Port.kMXP);
    
    /* PIDturnOutput = new DummyPIDOutPut();

    PIDturn = new PIDController(Constants.Turn_kP, Constants.Turn_kI, Constants.Turn_kD, Constants.Turn_kF, ahrs, PIDturnOutput, 0.02);
    PIDturn.setInputRange(-180.0,  180.0);
    PIDturn.setOutputRange(-0.65, 0.65);
    PIDturn.setAbsoluteTolerance(Constants.kToleranceDegrees);
    PIDturn.setContinuous(true);
   */
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
  
  public void tank(double left, double right) {
    mDrive.tankDrive(left, right);
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
public void setVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
  configureTalonsForSpeedControl();
  updateVelocitySetpoint(left_inches_per_sec, right_inches_per_sec);
}
public void setVelocityHeadingSetpoint(double forward_inches_per_sec, Rotation2d headingSetpoint) {
configureTalonsForSpeedControl();
velocityHeadingPid_.reset();
velocityHeadingSetpoint_ = new VelocityHeadingSetpoint(forward_inches_per_sec, forward_inches_per_sec, headingSetpoint);
updateVelocityHeadingSetpoint();
}
public void followPath(Path path, boolean reversed) {
configureTalonsForSpeedControl();
velocityHeadingPid_.reset();
pathFollowingController_ = new AdaptivePurePursuitController(Constants.kPathFollowingLookahead,
  Constants.kPathFollowingMaxAccel, Constants.kLooperDt, path, reversed, 0.25);
updatePathFollower();
}
private void configureTalonsForSpeedControl() {
  mDrive_Left_Master.set(ControlMode.Velocity, 0.0);
  mDrive_Left_Master.selectProfileSlot(kVelocityControlSlot, 0);
  mDrive_Left_Master.configAllowableClosedloopError(0, Constants.kDriveVelocityAllowableError);
  mDrive_Right_Master.set(ControlMode.Velocity, 0.0);
  mDrive_Right_Master.selectProfileSlot(kVelocityControlSlot, 0);
  mDrive_Right_Master.configAllowableClosedloopError(0, Constants.kDriveVelocityAllowableError);
  setBrake();
}
public void updateVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
  leftvelo_ = inchesPerSecondToVelo(left_inches_per_sec);
  rightvelo_ = inchesPerSecondToVelo(right_inches_per_sec);
  mDrive_Left_Master.set(ControlMode.Velocity, -leftvelo_);
  mDrive_Right_Master.set(ControlMode.Velocity, rightvelo_);
  SmartDashboard.putNumber("leftvelo", leftvelo_);
  SmartDashboard.putNumber("rightvelo", rightvelo_);
}
public Rotation2d getGyroAngle() {
  return Rotation2d.fromDegrees(ahrs.getAngle());
}
private void updateVelocityHeadingSetpoint() {
  Rotation2d actualGyroAngle = getGyroAngle();

  mLastHeadingErrorDegrees = velocityHeadingSetpoint_.getHeading().rotateBy(actualGyroAngle.inverse()).getDegrees();

  double deltaSpeed = velocityHeadingPid_.calculate(mLastHeadingErrorDegrees);
  updateVelocitySetpoint(velocityHeadingSetpoint_.getLeftSpeed() + deltaSpeed / 2, velocityHeadingSetpoint_.getRightSpeed() - deltaSpeed / 2);
}
public void updatePathFollower() {
RigidTransform2d robot_pose = RobotState.getInstance().getLatestFieldToVehicle().getValue();
RigidTransform2d.Delta command = pathFollowingController_.update(robot_pose, Timer.getFPGATimestamp());
Kinematics.DriveVelocity setpoint = Kinematics.inverseKinematics(command);

// Scale the command to respect the max velocity limits
double max_vel = 0.0;
max_vel = Math.max(max_vel, Math.abs(setpoint.left));
max_vel = Math.max(max_vel, Math.abs(setpoint.right));
if (max_vel > Constants.kPathFollowingMaxVel) {
    double scaling = Constants.kPathFollowingMaxVel / max_vel;
    setpoint = new Kinematics.DriveVelocity(setpoint.left * scaling, setpoint.right * scaling);
}
updateVelocitySetpoint(setpoint.left, setpoint.right);
SmartDashboard.putNumber("setpoint.left", setpoint.left);
SmartDashboard.putNumber("setpoint.right", setpoint.right);
}
public void updateRobotState() {
time = Timer.getFPGATimestamp();
left_distance = getLeftDistanceInches();
right_distance = getRightDistanceInches();
gyro_angle = getGyroAngle();
odometry = robotstate.generateOdometryFromSensors(left_distance - left_encoder_prev_distance_, right_distance - right_encoder_prev_distance_, gyro_angle);
velocity = Kinematics.forwardKinematics(getLeftVelocityInchesPerSec(mDrive_Left_Master.getSelectedSensorVelocity()), getRightVelocityInchesPerSec(mDrive_Right_Master.getSelectedSensorVelocity()));
robotstate.addObservations(time, odometry, velocity);
left_encoder_prev_distance_ = left_distance;
right_encoder_prev_distance_ = right_distance;
}
public void normalizeEncoders() {
left_encoder_prev_distance_ = getLeftDistanceInches();
right_encoder_prev_distance_ = getRightDistanceInches();
}
public boolean isFinishedPath() {
return pathFollowingController_.isDone();
}
public void zeroYaw() {
ahrs.zeroYaw();
}
public void resetGyro() {
ahrs.reset();
}
public double getYaw() {
return ahrs.getYaw();
}
public double getAngle() {
return ahrs.getAngle();
}
@Override
public void periodic() {
  // This method will be called once per scheduler run
}
}
