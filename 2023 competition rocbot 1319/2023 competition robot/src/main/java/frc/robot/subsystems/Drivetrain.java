package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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

    private boolean shifter = false;
    private Solenoid m_shifter = new Solenoid(PneumaticsModuleType.REVPH, 1);

    public Drivetrain() {
        leftMotor1.setInverted(true);
        leftMotor2.setInverted(true);

        leftMotor2.follow(leftMotor1);
        rightMotor2.follow(rightMotor1);
    }

    public void teleopPeriodic(XboxController Driver, XboxController Operator) {
        robotDrive.arcadeDrive(-Driver.getLeftY(), -Driver.getRightX());

        // Driver Controls

        // Left Bumper=Shifter On
        if (Driver.getLeftBumperPressed()) {
            shifter = true;
            // Right Bumper=Shifter Off
        } else if (Driver.getRightBumperPressed()) {
            shifter = false;
        }
        m_shifter.set(shifter);
    }
}
