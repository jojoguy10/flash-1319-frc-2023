package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.arm.BalanceCommand;
import frc.robot.commands.arm.DriveBackwards;
import frc.robot.commands.arm.ExtendArmToPos;
import frc.robot.commands.arm.ToggleGripper;
import frc.robot.subsystems.Arm.TelePreset;

public class Drivetrain extends SubsystemBase {
    private CANSparkMax leftMotor1 = new CANSparkMax(1, MotorType.kBrushless);
    private CANSparkMax leftMotor2 = new CANSparkMax(2, MotorType.kBrushless);
    private CANSparkMax rightMotor1 = new CANSparkMax(3, MotorType.kBrushless);
    private CANSparkMax rightMotor2 = new CANSparkMax(4, MotorType.kBrushless);

    private DifferentialDrive robotDrive = new DifferentialDrive(leftMotor1, rightMotor1);
    private SlewRateLimiter xLimiter = new SlewRateLimiter(2.0), zLimiter = new SlewRateLimiter(99.0);

    private boolean brake = false;
    private boolean shifter = false;
    private Solenoid m_shifter = new Solenoid(10, PneumaticsModuleType.REVPH, 1);
    //private Solenoid m_brake = new Solenoid(10, PneumaticsModuleType.REVPH, 4);

    public PigeonIMU imu;
    public double startPitch;

    public Drivetrain() {
        rightMotor1.setIdleMode(IdleMode.kCoast);
        rightMotor2.setIdleMode(IdleMode.kCoast);
        leftMotor1.setInverted(true);
        leftMotor1.setIdleMode(IdleMode.kCoast);
        leftMotor2.setInverted(true);
        leftMotor2.setIdleMode(IdleMode.kCoast);

        double p = 0;
        double i = 0;
        double d = 0;

        leftMotor2.follow(leftMotor1);
        rightMotor2.follow(rightMotor1);

        SparkMaxPIDController pidL1 = leftMotor1.getPIDController();
        pidL1.setP(p);
        pidL1.setI(i);
        pidL1.setD(d);

        SparkMaxPIDController pidR1 = rightMotor1.getPIDController();
        pidR1.setP(p);
        pidR1.setI(i);
        pidR1.setD(d);

        imu = new PigeonIMU(25);
        leftMotor1.getEncoder().setPosition(0);
    }

    public void driveToPosition(double pos) {
        leftMotor1.getPIDController().setReference(pos, CANSparkMax.ControlType.kPosition);
        rightMotor1.getPIDController().setReference(pos, CANSparkMax.ControlType.kPosition);
    }

    public void arcadeDrive(double x, double r, boolean b) {
        robotDrive.arcadeDrive(x, r, b);
    }

    public double getDistance() {
        double ratio = 3.14192 * 2 * 3 * 2.54 / 17.78;
        return leftMotor1.getEncoder().getPosition() * ratio;
    }

    public void resetEncoder() {
        leftMotor1.getEncoder().setPosition(0.0);
    }

    public void teleopPeriodic(XboxController Driver, XboxController Operator) {
        double x = xLimiter.calculate(-Driver.getLeftY());
        double z = zLimiter.calculate(-Driver.getRightX()*0.6);
        robotDrive.arcadeDrive(x, z);

        // Driver Controls

        // Left Bumper=Shifter On
        if (Driver.getLeftTriggerAxis() >= 0.9) {
            shifter = true;
            // Right Bumper=Shifter Off
        } else if (Driver.getRightTriggerAxis() >= 0.9) {
            shifter = false;
        }
        m_shifter.set(shifter);

         
     /*if(Driver.getBButtonPressed()){
        if(brake == true){
            brake = false;
        }else{
            brake = true;
        }
      }*/
      //m_brake.set(brake);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Base Encoder", rightMotor1.getEncoder().getPosition());
        SmartDashboard.putNumber("Base Distance", getDistance());
        SmartDashboard.putNumber("Corrected pitch", imu.getRoll() - startPitch);
    }

    public Command balanceAuto() {
        return new DriveBackwards(this, 200).andThen(new BalanceCommand(this));
    }

    public Command cubeAuto(Arm arm, IntakeSubsystem intakeSubsystem) {
        return new DriveBackwards(this, 50)
        .andThen(arm.createDriveArmCommand(intakeSubsystem, 120, TelePreset.HIGH))
        .andThen(new ExtendArmToPos(arm, 40))
        .andThen(new ToggleGripper(intakeSubsystem))
        .andThen(arm.fullLowerCommands(intakeSubsystem));
    }
}
