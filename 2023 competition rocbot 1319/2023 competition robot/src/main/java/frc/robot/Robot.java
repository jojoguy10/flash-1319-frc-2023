// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.sql.Driver;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private final static String kDefaultAuto = "Default";
  private final static String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private SendableChooser<String> m_chooser = new SendableChooser<>();
  //Drive base Motors

  //Shoulder Motors
  
  //private CANSparkMax shoulderM2 = new CANSparkMax(6, MotorType.kBrushless);

//Intake Motors
  private final CANSparkMax intakeM1 = new CANSparkMax(7, MotorType.kBrushed);
  private final CANSparkMax intakeM2 = new CANSparkMax(8, MotorType.kBrushed);

  

  // Compressor and Pneumatics
  private final Compressor  m_Compressor = new Compressor(10,PneumaticsModuleType.REVPH);

  
  private Solenoid m_intake = new Solenoid(PneumaticsModuleType.REVPH, 2);
  private Solenoid m_gripper  = new Solenoid(PneumaticsModuleType.REVPH, 3);
  private Solenoid m_brake = new Solenoid(PneumaticsModuleType.REVPH, 4);
  //private Solenoid m_SPARE = new Solenoid(PneumaticsModuleType.REVPH, 5);


private boolean intake = false;
private boolean gripper = false;
private boolean brake = false;
private double intakeToggle = 0.5;
  private Drivetrain drivetrain = new Drivetrain();
  private Arm arm = new Arm();
  private final XboxController Driver = new XboxController(0);
  private final XboxController Operator = new XboxController(1);
  private final CommandXboxController OperatorCommand = new CommandXboxController(1);
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    intakeM2.setInverted(true);



    m_Compressor.enableDigital();
    OperatorCommand.povDown().onTrue(arm.createDriveArmCommand(0, 0));
    OperatorCommand.povUp().onTrue(arm.createDriveArmCommand(125, 120));
    OperatorCommand.povRight().onTrue(arm.createDriveArmCommand(110, 23));
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    m_Compressor.enableDigital();
    CommandScheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    drivetrain.teleopPeriodic(Driver, Operator);
    arm.teleopPeriodic(Driver, Operator);
    //Y Button = Intake On
    if(Operator.getYButton()){
      if(intake == true){
        intake = false;
       }else{
        intake = true;
       }
       m_intake.set(intake);
    }

    //Operator Controls

    
    //Right Bumper=Brake On 
     if(Operator.getRightBumperPressed()){
      brake = true;
    //Left Bumper=Brake Off
     }else if(Operator.getLeftBumperPressed()){
      brake = false;
    }
    m_brake.set(brake);

    if(Operator.getBButtonPressed()){
      gripper = true;
    //X Button = Gripper Off
      }else if(Operator.getXButtonPressed()){
      gripper = false;
      }
    //Right Button On D-Pad = 90 Degrees 
      if (Operator.getAButton()){
      if(intakeToggle == 0.5 ){
        intakeToggle = 0;
      }else{
        intakeToggle = 0.5;
      }
      intakeM1.set(intakeToggle);
      intakeM2.set(intakeToggle);
      }

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
