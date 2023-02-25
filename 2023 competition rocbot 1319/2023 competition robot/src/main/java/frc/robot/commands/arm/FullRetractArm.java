package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class FullRetractArm extends CommandBase {
    Arm arm;

    public FullRetractArm(Arm arm) {
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setTelescopePos(0);
    }

    @Override
    public boolean isFinished() {
        return !arm.isExtended();
    }
}
