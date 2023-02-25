package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ShoulderDriveToPosition extends CommandBase {
    Arm arm;
    double pos;
    // Creates a new ShoulderDriveToPosition Command
    // arm - reference to the arm subsystem
    // position - what position the command will drive
    public ShoulderDriveToPosition(Arm arm, double pos) {
        this.arm = arm;
        this.pos = pos;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setShoulderAngle(pos);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(arm.getArmAngle()-pos) < 4;
    }


}
