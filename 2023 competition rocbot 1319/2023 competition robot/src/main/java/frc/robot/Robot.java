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

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CameraServerJNI;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.arm.RunArm;
import frc.robot.commands.arm.RunIntake;
import frc.robot.commands.arm.ToggleBrakeCommand;
import frc.robot.commands.arm.ToggleGripper;
import frc.robot.commands.arm.ToggleIntake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Arm.TelePreset;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private final static String kDefaultAuto = "Drive Forward";
  private final static String kCustomAuto = "Balance Auto";
  private final static String kCubeAuto = "Cube Auto";
  private String m_autoSelected;
  private SendableChooser<String> m_chooser = new SendableChooser<>();
  // Drive base Motors

  // Shoulder Motors

  // private CANSparkMax shoulderM2 = new CANSparkMax(6, MotorType.kBrushless);



  // Compressor and Pneumatics
  private final Compressor m_Compressor = new Compressor(10, PneumaticsModuleType.REVPH);


  // private Solenoid m_SPARE = new Solenoid(PneumaticsModuleType.REVPH, 5);

  private boolean useCameraA = false;
  private Drivetrain drivetrain = new Drivetrain();
  private Arm arm = new Arm();
  private IntakeSubsystem intake = new IntakeSubsystem();
  private final XboxController Driver = new XboxController(0);
  private final XboxController Operator = new XboxController(1);
  private final CommandXboxController OperatorCommand = new CommandXboxController(1);
  private final CommandXboxController DriverCommand = new CommandXboxController(0);
  //private final UsbCamera cameraA = CameraServer.startAutomaticCapture(0);
  //private final UsbCamera cameraB = CameraServer.startAutomaticCapture(1);
  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    m_Compressor.enableDigital();
    OperatorCommand.povDown().onTrue(arm.fullLowerCommands(intake));
    OperatorCommand.povUp().onTrue(arm.createDriveArmCommand(intake, Arm.armHighPos, TelePreset.HIGH));
    OperatorCommand.povRight().onTrue(arm.createDriveArmCommand(intake, Arm.armMidPos, TelePreset.MID));
    OperatorCommand.rightTrigger(0.75).onTrue(new ToggleGripper(intake));
    OperatorCommand.rightBumper().onTrue(new ToggleIntake(intake));
    DriverCommand.a().onTrue(new ToggleBrakeCommand(drivetrain));
    /*OperatorCommand.start().onTrue(new InstantCommand(() -> {
      if(useCameraA) {
        CameraServer.getServer().setSource(cameraA);
      } else {
        CameraServer.getServer().setSource(cameraB);
      }
      useCameraA = !useCameraA;
    })); */
    intake.setDefaultCommand(new RunIntake(intake, Operator));
    arm.setDefaultCommand(new RunArm(arm, Operator));
    UsbCamera camera = CameraServer.startAutomaticCapture();
    //cameraA.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    //cameraB.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    //CameraServer.getServer().setSource(cameraA);

    m_chooser.addOption(kCustomAuto, kCustomAuto);
    m_chooser.setDefaultOption(kDefaultAuto, kDefaultAuto);
    m_chooser.addOption(kCubeAuto, kCubeAuto);
    SmartDashboard.putData(m_chooser);

    drivetrain.startPitch = drivetrain.imu.getRoll();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    if(arm.getArmAngle() > 2.0 && arm.getArmAngle() < 35) {
      intake.closeGripper();
    }
    //m_Compressor.enableDigital();
    CommandScheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    
    //drivetrain.balanceAuto().schedule();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        (drivetrain.balanceAuto()).schedule();
        break;
      case kDefaultAuto:
      default:
      (new DriveDistance(drivetrain, 350)).schedule();
        // Put default auto code here
        break;
      case kCubeAuto:
        (drivetrain.cubeAuto(arm, intake)).schedule();
        break;
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

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
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
