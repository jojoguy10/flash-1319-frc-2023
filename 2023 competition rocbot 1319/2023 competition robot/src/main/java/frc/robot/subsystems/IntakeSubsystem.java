package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private Solenoid m_intake = new Solenoid(10, PneumaticsModuleType.REVPH, 2);
    private Solenoid m_gripper = new Solenoid(10, PneumaticsModuleType.REVPH, 3);

        // Intake Motors
    private final CANSparkMax intakeM1 = new CANSparkMax(7, MotorType.kBrushed);
    private final CANSparkMax intakeM2 = new CANSparkMax(8, MotorType.kBrushed);
    boolean intakeRunning;
    public IntakeSubsystem() {
        intakeM2.setInverted(true);
    }

    public void toggleIntake() {
        m_intake.toggle();
    }

    public void toggleGripper() {
        m_gripper.toggle();
    }

    public void closeGripper() {
        m_gripper.set(false);
    }

    public void lowerIntake() {
        m_intake.set(true);
    }

    public void runIntakeForwards() {
        intakeM1.set(1);
        intakeM2.set(1);
    }

    public void runIntakeBackwards() {
        intakeM1.set(-1);
        intakeM2.set(-1);
    }

    public void stopIntake() {
        intakeM1.set(0);
        intakeM2.set(0);
    }
}
