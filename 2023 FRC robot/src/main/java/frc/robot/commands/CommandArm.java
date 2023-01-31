package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;

public class CommandArm extends InstantCommand {
    public CommandArm(ArmSubsystem armSubsystem){
        super(() -> {
            armSubsystem.toggleArm();
        },armSubsystem);

    }
    

}
