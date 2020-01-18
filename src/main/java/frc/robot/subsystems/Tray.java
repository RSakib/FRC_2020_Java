/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.RobotMap;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*;

public class Tray extends SubsystemBase {

  private static final Tray instance = new Tray();

    public static Tray getInstance() {
      return instance;
    }
    
    public Solenoid mTray_Extend, mTray_Retract, mTalons_Hold, mTalons_Release;
    public TalonSRX mShooter;
    public DigitalInput mCargo_Loaded_Sensor, mHatch_Loaded_Sensor;

    public boolean autograb;

  public Tray() {

    mTray_Extend = new Solenoid(RobotMap.PCM_B, RobotMap.mTrayExtended_ID);
    mTray_Retract = new Solenoid(RobotMap.PCM_A, RobotMap.mTrayRetract_ID);
    mTalons_Hold = new Solenoid(RobotMap.PCM_A, RobotMap.mTalonsHold_ID);
    mTalons_Release = new Solenoid(RobotMap.PCM_B, RobotMap.mTalonsRelease_ID);
    mShooter = new TalonSRX(RobotMap.mShooter_ID);

    mShooter.configFactoryDefault();
    mShooter.setNeutralMode(NeutralMode.Brake);
    mShooter.configContinuousCurrentLimit(30);
    mShooter.configPeakCurrentLimit(0);
    mShooter.enableCurrentLimit(true);
    mShooter.setInverted(false);

        mCargo_Loaded_Sensor = new DigitalInput(RobotMap.mCargoLoadedSensor_ID);
        mHatch_Loaded_Sensor = new DigitalInput(RobotMap.mHatchLoadedSensor_ID);

        autograb = false;
        RetractTray();
        TalonsRelease();
  }

  public void ShootCargo() {
    mShooter.set(ControlMode.PercentOutput, 1.0);
  }
  public void StopShootCargo() {
    mShooter.set(ControlMode.PercentOutput, 0.0);
  }
  public void IntakeCargo() {
    mShooter.set(ControlMode.PercentOutput, 1.0);
  }
  public void StopIntakeCargo() {
    mShooter.set(ControlMode.PercentOutput, 0.0);
  }
  public void ExtendTray() {
    mTray_Extend.set(Constants.On);
    mTray_Retract.set(Constants.Off);
    Constants.TRAY_STATE = Constants.TRAY_STATE_EXTENDED;
  }
  public void RetractTray() {
    mTray_Extend.set(Constants.Off);
    mTray_Retract.set(Constants.On);
    Constants.TRAY_STATE = Constants.TRAY_STATE_RETRACTED;
  }
  public void TalonsHold() {
    mTalons_Release.set(Constants.Off);
    mTalons_Hold.set(Constants.On);
    Constants.TALON_STATE = Constants.TALON_STATE_HOLDING;
  }
  public void TalonsRelease() {
    mTalons_Hold.set(Constants.Off);
    mTalons_Release.set(Constants.On);
    Constants.TALON_STATE = Constants.TALON_STATE_RELEASED;
  }
  public void TalonsAutoGrab() {
    if (Constants.WantHatch == true && !mHatch_Loaded_Sensor.get()) {
      autograb = false;
      TalonsHold();
      Constants.WantHatch = false;
    }
    else if (Constants.WantHatch == true && mHatch_Loaded_Sensor.get()){
      autograb = true;
      TalonsRelease();
    }
  }
  public void UpdateLoadState() {
    if (!mCargo_Loaded_Sensor.get()) {
      Constants.CARGO_STATE = Constants.CARGO_STATE_LOADED;
    }
    else if (mCargo_Loaded_Sensor.get()) {
      Constants.CARGO_STATE = Constants.CARGO_STATE_UNLOADED;
    }
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
