package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
    private CANSparkMax leftMotor1 = new CANSparkMax(1, MotorType.kBrushless);
    private CANSparkMax leftMotor2 = new CANSparkMax(2, MotorType.kBrushless);
    private CANSparkMax rightMotor1 = new CANSparkMax(3, MotorType.kBrushless);
    private CANSparkMax rightMotor2 = new CANSparkMax(4, MotorType.kBrushless);

    private DifferentialDrive robotDrive = new DifferentialDrive(leftMotor1, rightMotor1);
    private SlewRateLimiter xLimiter = new SlewRateLimiter(1.0), zLimiter = new SlewRateLimiter(1.0);

    private boolean brake = false;
    private boolean shifter = false;
    private Solenoid m_shifter = new Solenoid(10, PneumaticsModuleType.REVPH, 1);
    private Solenoid m_brake = new Solenoid(10, PneumaticsModuleType.REVPH, 4);


    public Drivetrain() {
        leftMotor1.setInverted(true);
        leftMotor2.setInverted(true);

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
    }

    public void driveToPosition(double pos) {
        leftMotor1.getPIDController().setReference(pos, CANSparkMax.ControlType.kPosition);
        rightMotor1.getPIDController().setReference(pos, CANSparkMax.ControlType.kPosition);
    }

    public void teleopPeriodic(XboxController Driver, XboxController Operator) {
        double x = xLimiter.calculate(-Driver.getLeftY());
        double z = zLimiter.calculate(-Driver.getRightX());
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

         
     if(Driver.getBButtonPressed()){
        if(brake == true){
            brake = false;
        }else{
            brake = true;
        }
      }
      m_brake.set(brake);
    }
}
