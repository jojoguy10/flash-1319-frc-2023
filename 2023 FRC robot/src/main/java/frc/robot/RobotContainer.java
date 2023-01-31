package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.CommandArm;
import frc.robot.commands.CommandDriveBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveBaseSubsystem;

public class RobotContainer {
    private DriveBaseSubsystem m_DriveBaseSubsystem = new DriveBaseSubsystem();
    private ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
    private final XboxController m_Driver = new XboxController(0);
    private final XboxController m_Operator = new XboxController(0);
    ////private final JoystickButton m_Driver_b = new JoystickButton(m_Driver, XboxController.Button.kB.value);
    //private final JoystickButton m_Driver_y= new JoystickButton(m_Driver, XboxController.Button.kY.value);

   // private final JoystickButton m_Operator_x= new JoystickButton(m_Operator, XboxController.Button.kX.value);
    public RobotContainer() {
    m_DriveBaseSubsystem.setDefaultCommand(new CommandDriveBase(m_DriveBaseSubsystem, m_Driver));
   // m_Operator_x.3a+-onTrue(new CommandArm(m_ArmSubsystem));
    //m_Driver_b.onTrue(new CommandDriveBase(m_DriveBaseSubsystem, m_Driver));
   // m_Driver_y.onTrue(new CommandDriveBase(m_DriveBaseSubsystem, m_Driver));
        

 }

 
}

