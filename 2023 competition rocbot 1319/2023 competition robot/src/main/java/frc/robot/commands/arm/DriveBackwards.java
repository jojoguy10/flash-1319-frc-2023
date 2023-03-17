// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveBackwards extends CommandBase {
  /** Creates a new DriveBackwards. */
  Drivetrain drivetrain;
  double distance;
  public DriveBackwards(Drivetrain drivetrain, double distance) {
    this.drivetrain = drivetrain;
    this.distance = distance;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void initialize() {
      drivetrain.resetEncoder();
  }

  @Override
  public void execute() {
    drivetrain.arcadeDrive(-0.6, 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.arcadeDrive(0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drivetrain.getDistance() < -distance;
  }
}
