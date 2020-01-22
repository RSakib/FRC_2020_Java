/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.PID;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.utility.PurePursuit.*;

import frc.robot.Constants;
import frc.robot.subsystems.Drivebase;

public class PIDTurn extends PIDSubsystem {
  /**
   * Creates a new PIDTurnTest.
   */
  double output;

  public PIDTurn() {
    super(
         // The PIDController used by the subsystem
        new PIDController(Constants.Turn_kP, Constants.Turn_kI, Constants.Turn_kD));
        getController().setTolerance(Constants.kToleranceDegrees);
        getController().enableContinuousInput(-180, 180);
        //getOutput().enableContinuousInput(-0.65, 0.65); 
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    output = 0;
    

  }

  public void pidWrite(double output) {
    this.output = output;
  }

  public double getOutput() {
    return output;
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return 0;
  }
}
