package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.CommandDriveBase;
import frc.robot.subsystems.DriveBaseSubsystem;

public class RobotContainer {
  private final DriveBaseSubsystem m_DriveBaseSubsystem = new DriveBaseSubsystem();
  private final XboxController m_Driver = new XboxController(0);
  private final JoystickButton m_Driver_b = new JoystickButton(m_Driver, XboxController.Button.kB.value);
  private final JoystickButton m_Driver_y= new JoystickButton(m_Driver, XboxController.Button.kY.value);

  public RobotContainer() {
    final CommandDriveBase driveBaseCommand = new CommandDriveBase(m_DriveBaseSubsystem, m_Driver);
    m_DriveBaseSubsystem.setDefaultCommand(driveBaseCommand);
  }
	
  public Command getAutonomousCommand() {
		return null;
	}
}

