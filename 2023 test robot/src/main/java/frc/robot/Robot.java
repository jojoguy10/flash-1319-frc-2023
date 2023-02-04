// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.print.attribute.standard.Compression;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.WPILibVersion;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private final Double COUNTS_PER_INCH = 841.0;
  private final Double INCHES_PER_COUNT = 0.0012;

  private final WPI_TalonFX m_leftMotor1 = new WPI_TalonFX(1);
  private final WPI_TalonFX m_leftMotor2 = new WPI_TalonFX(2);
  private final WPI_TalonFX m_rightMotor1 = new WPI_TalonFX(3);
  private final WPI_TalonFX m_rightMotor2 = new WPI_TalonFX(4);
  private final WPI_TalonFX spinerMotor1 = new WPI_TalonFX(9);
  private final WPI_TalonFX spinerMotor2 = new WPI_TalonFX(10);
  private final WPI_TalonFX intake = new WPI_TalonFX(6);
  private final WPI_TalonFX intake2 = new WPI_TalonFX(7);

  private final WPI_TalonSRX levelsMotor = new WPI_TalonSRX(12);
  private final WPI_TalonSRX theTurret = new WPI_TalonSRX(8);

  private DifferentialDrive m_robotDrive ;

  private final XboxController m_Joystick = new XboxController(0);

  private final Compressor m_Compressor = new Compressor(0,PneumaticsModuleType.CTREPCM);

  private final Solenoid m_shifter = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
 
  private boolean shift = false; 

  private double autoSpeed = 0.25;

  private final DigitalInput bottomlimitSwitch = new DigitalInput(4);
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
  
    m_leftMotor1.setInverted(true);
    m_leftMotor2.setInverted(true);

    m_rightMotor1.setInverted(true);
    m_rightMotor2.setInverted(true);

    spinerMotor2.setInverted(true);
    intake.setInverted(true);


    m_leftMotor2.follow(m_leftMotor1);
    m_rightMotor2.follow(m_rightMotor1);

    m_robotDrive = new DifferentialDrive(m_leftMotor1, m_rightMotor1);

    m_Compressor.enableDigital();

    m_leftMotor1.setSelectedSensorPosition(0,0,0);
    m_leftMotor2.setSelectedSensorPosition(0,0,0);
    m_rightMotor1.setSelectedSensorPosition(0,0,0);
    m_rightMotor2.setSelectedSensorPosition(0,0,0);

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
    


    SmartDashboard.putNumber("Joystick x Left value.", m_Joystick.getLeftX());
    SmartDashboard.putNumber("Joystick x Right value.", m_Joystick.getRightX());
    SmartDashboard.putNumber("Joystick y Right value.", m_Joystick.getRightY());
    SmartDashboard.putNumber("Joystick y Left value.", m_Joystick.getLeftY());
    SmartDashboard.putNumber("L motor 1 encoder.", m_leftMotor1.getSelectedSensorPosition(0));
    SmartDashboard.putNumber("L motor 2  encoder.", m_leftMotor2.getSelectedSensorPosition(0));
    SmartDashboard.putNumber("R motor 1  encoder.", m_rightMotor1.getSelectedSensorPosition(0));
    SmartDashboard.putNumber("R motor 2 encoder.", m_rightMotor2.getSelectedSensorPosition(0));
    SmartDashboard.putNumber("levels Motor", levelsMotor.getSelectedSensorPosition(0));

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
    m_leftMotor1.setSelectedSensorPosition(0,0,0);
    m_leftMotor2.setSelectedSensorPosition(0,0,0);
    m_rightMotor1.setSelectedSensorPosition(0,0,0);
    m_rightMotor2.setSelectedSensorPosition(0,0,0);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    SmartDashboard.putNumber("Auto speed ", autoSpeed);
     if (m_leftMotor1.getSelectedSensorPosition(0) >= 40368 ){
      autoSpeed = 0;
     }else{
      autoSpeed = 0.25;
     }
     //m_robotDrive.arcadeDrive(autoSpeed, 0);
     m_leftMotor1.set(ControlMode.PercentOutput, autoSpeed);
     m_leftMotor2.set(ControlMode.PercentOutput, autoSpeed);
     m_rightMotor1.set(ControlMode.PercentOutput, -autoSpeed);
     m_rightMotor2.set(ControlMode.PercentOutput, -autoSpeed);


  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    m_robotDrive.arcadeDrive(m_Joystick.getRightX(),m_Joystick.getLeftY());
    m_shifter.set(m_Joystick.getAButton());
    if (m_Joystick.getRightBumperPressed()){
      shift = false;
    }else if (m_Joystick.getLeftBumperPressed()){
      shift = true;
    }
    m_shifter.set(shift);

    
    if (m_Joystick.getAButton()){
      //levelsMotor.set(ControlMode.PercentOutput, 0.25);
      setMotorSpeed(0.25);

    }else if (m_Joystick.getYButton()){
      //levelsMotor.set(ControlMode.PercentOutput, 0);
      setMotorSpeed(-0.25);
    }else{
      setMotorSpeed(0);
    }

    if (m_Joystick.getXButton()){
      theTurret.set(0.25);
    } 
    else if(m_Joystick.getBButton()){
      theTurret.set(-0.25);
    }else{
      theTurret.set(0);
    }

    if(m_Joystick.getStartButton()){
      spinerMotor1.set(0.50);
      spinerMotor2.set(0.50);
      intake.set(1);
      intake2.set(1);
    }else{
      spinerMotor1.set(0);
      spinerMotor2.set(0);
      intake.set(0);
      intake2.set(0);
    }

  }

  public void setMotorSpeed(double speed) {
    if (speed > 0) {
      if (!bottomlimitSwitch.get()) {
      levelsMotor.set(0);
      
      }else{
        levelsMotor.set(speed);
      }
    }else if(speed<0){
      levelsMotor.set(speed);
    }else{
      levelsMotor.set(0);
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