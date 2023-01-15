package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.CommandDriveBase;
import frc.robot.subsystems.DriveBaseSubsystem;

public class RobotContainer {
    private DriveBaseSubsystem m_DriveBaseSubsystem = new DriveBaseSubsystem();
    private final XboxController m_controller = new XboxController(0);
 public RobotContainer() {
    m_DriveBaseSubsystem.setDefaultCommand(new CommandDriveBase(m_DriveBaseSubsystem, m_controller));
 }
}

