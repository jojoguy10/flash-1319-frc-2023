// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class RunArm extends CommandBase {
  /** Creates a new RunArm. */
  Arm arm;
  XboxController controller;
  public RunArm(Arm arm, XboxController controller) {
    this.arm = arm;
    this.controller = controller;
    addRequirements(arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double shoulderSensitivity = 0.4;
    if(Math.abs(controller.getLeftY()) > 0.2) {
      double newTarget = arm.getArmTarget() + controller.getLeftY()*shoulderSensitivity;
      // Is the arm retracted or is the arm above the intake
      if(!arm.isExtended() || arm.getArmTarget() > arm.extendLimitThreshold) {
        arm.setShoulderAngle(newTarget);
      }

    }
    double teleSensitivity = 0.4;
    if(Math.abs(controller.getRightY()) > 0.2) {
      if(arm.getArmTarget() > arm.extendLimitThreshold) {
        arm.setTelescopePos(arm.getTeleTarget() + controller.getRightY()*teleSensitivity);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
