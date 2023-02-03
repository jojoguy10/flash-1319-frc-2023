package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBaseSubsystem;

public class CommandDriveBase extends CommandBase{
    private DriveBaseSubsystem m_Base;
    private XboxController m_XboxController;
    
    public CommandDriveBase(DriveBaseSubsystem base, XboxController xbox){
        m_Base = base;
        m_XboxController = xbox;
    }

    public void execute(){
        double x;
        double y;
        x = m_XboxController.getLeftY();
        y = m_XboxController.getRightX();
        m_Base.drive(x, y);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
