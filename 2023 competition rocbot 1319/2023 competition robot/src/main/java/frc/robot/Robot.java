// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.sql.Driver;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
//Drive base Motors
  private final CANSparkMax leftMotor1 = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax leftMotor2 = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax rightMotor1 = new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax rightMotor2 = new CANSparkMax(4, MotorType.kBrushless);
//Shoulder Motors
  private final CANSparkMax shoulderM1 = new CANSparkMax(5, MotorType.kBrushless);
//private final CANSparkMax shoulderM2 = new CANSparkMax(6, MotorType.kBrushless);

//Intake Motors
  private final CANSparkMax intakeM1 = new CANSparkMax(7, MotorType.kBrushless);
  private final CANSparkMax intakeM2 = new CANSparkMax(8, MotorType.kBrushless);

// Telescoping Motor
  private final CANSparkMax telescopingM1 = new CANSparkMax(9, MotorType.kBrushless);

  // Compressor and Pneumatics
  private final Compressor  m_Compressor = new Compressor(0,PneumaticsModuleType.CTREPCM);

private final Solenoid m_shifter = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
private final Solenoid m_intake = new Solenoid(PneumaticsModuleType.CTREPCM, 2);
private final Solenoid m_gripper  = new Solenoid(PneumaticsModuleType.CTREPCM, 3);
private final Solenoid m_brake = new Solenoid(PneumaticsModuleType.CTREPCM, 4);

private boolean shifter = false; 
private boolean intake = false;
private boolean gripper = false;
private boolean brake = false;

  private DifferentialDrive robotDrive = new DifferentialDrive(leftMotor1,rightMotor1);

  private final XboxController Driver = new XboxController(0);
  private final XboxController Operator = new XboxController(1);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    leftMotor1.setInverted(true);
    leftMotor2.setInverted(true);

    leftMotor2.follow(leftMotor1);
    rightMotor2.follow(rightMotor1);


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
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    robotDrive.arcadeDrive(-Driver.getLeftY(),-Driver.getRightX());
    
     if(Driver.getLeftBumperPressed()){
      shifter = true;
     }else if(Driver.getRightBumperPressed()){
      shifter = false;
     }

     if(Operator.getYButtonPressed()){
      intake = true;
     }else if(Operator.getAButtonPressed()){
      intake = false;
     }

     if(Operator.getRightBumperPressed()){
      brake = true;
     }else if(Operator.getLeftBumperPressed()){
      brake = false;
      }

      if(Operator.getBButtonPressed()){
      gripper = true;
      }else if(Operator.getXButtonPressed()){
      gripper = false;
      }

      if (Operator.getPOV() == 90){
      intakeM1.set(0.75);
      intakeM2.set(0.75);
     }else if(Operator.getPOV() == 270){
      intakeM1.set(-0.75);
      intakeM2.set(-0.75);
     }else{
      intakeM1.set(0);
      intakeM2.set(0);
      }

     if(Operator.getStartButton()){
      telescopingM1.set(0.75);
     }else if(Operator.getBackButton()){
      telescopingM1.set(-0.75);
      }else{
      telescopingM1.set(0);
      }
      
      if(Operator.getPOV() == 0){
        shoulderM1.set(0.75);
      }else if(Operator.getPOV() == 180){
        shoulderM1.set(-0.75);
      }else{
        shoulderM1.set(0);
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
