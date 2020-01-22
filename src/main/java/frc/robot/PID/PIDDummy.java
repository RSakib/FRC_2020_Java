/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.PID;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class PIDDummy extends PIDSubsystem {
  /**
   * Creates a new PIDDummy.
   */
  double output;

  public PIDDummy() {
    super(
        // The PIDController used by the subsystem
        new PIDController(0, 0, 0));
        output = 0;
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    
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
