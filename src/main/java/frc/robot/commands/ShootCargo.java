/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Tray;

public class ShootCargo extends CommandBase {
  /**
   * Creates a new ShootCargo.
   */
  private Timer shootTimer = new Timer();
  private boolean doneshooting = false;
  private Tray tray = Tray.getInstance();

  public ShootCargo() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tray.ShootCargo();
    shootTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shootTimer.get() >= 1.0) {
      shootTimer.stop();
      shootTimer.reset();
      tray.StopShootCargo();
      doneshooting = true;
    }
    else {
      doneshooting = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (doneshooting == true) {
      return true;
    }
    else {
      return false;
    }
  }
}
