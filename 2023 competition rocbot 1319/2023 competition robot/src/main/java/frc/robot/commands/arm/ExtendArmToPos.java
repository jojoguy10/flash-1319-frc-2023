package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.TelePreset;

public class ExtendArmToPos extends CommandBase {
    Arm arm;
    double pos;
    public ExtendArmToPos(Arm arm, TelePreset pos) {
        this.arm = arm;
        this.pos = pos.telePos;
    }

    @Override
    public void initialize() {
        arm.setTelescopePos(pos);
    }

   // @Override
    //public boolean isFinished() {
        // Is the arm within 4 units of the goal
        //return Math.abs(arm.getTelePos()-pos) < 4;
    //}
}
